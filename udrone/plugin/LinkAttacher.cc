/*
 * CoRAL latch: Open-UAV gazebo_ros_link_attacher semantics on gz-sim8.
 * Baseline: https://github.com/Open-UAV/gazebo_ros_link_attacher
 *   - world plugin creates a fixed joint between two links
 *   - String "ATTACH" / "DETACH" (plus Empty attach/detach topics)
 *   - gates before (re)attach: distance to the nest and angle
 *
 * Differences from the baseline, all for a repeatable dock:
 *   - the child is moved into <nest_pose> one tick before the weld
 *   - state is published every <state_period> seconds, not only on change
 *   - if either model is removed, the latch forgets the joint and docks the
 *     respawned child again
 */
#include "LinkAttacher.hh"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <functional>
#include <iostream>

#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/DetachableJoint.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/PoseCmd.hh>
#include <gz/sim/components/World.hh>

#include <sdf/Element.hh>

using namespace gz;
using namespace sim;

namespace udrone
{
namespace
{
// gz sim runs at --verbose=1 under PX4, which hides gzmsg/gzwarn. Latch
// transitions and refusals are worth a line in the simulator log.
void Log(const std::string &_text)
{
  std::cerr << "[LinkAttacher] " << _text << std::endl;
}

// Ticks between the nest move and the weld. The physics system applies the
// pose command during the tick it is set in; the joint goes in on the next.
constexpr int kWeldDelayTicks = 1;
// Ticks between searches for the two models while one is missing.
constexpr int kResolveBackoffTicks = 250;
}  // namespace

/////////////////////////////////////////////////
void LinkAttacher::Configure(const Entity &_entity,
                             const std::shared_ptr<const sdf::Element> &_sdf,
                             EntityComponentManager &_ecm,
                             EventManager &)
{
  if (!_ecm.EntityHasComponentType(_entity, components::World::typeId)) {
    gzerr << "LinkAttacher must be attached to a <world>.\n";
    return;
  }

  auto get = [&](const char *key, std::string &out) {
    if (_sdf->HasElement(key)) out = _sdf->Get<std::string>(key);
  };
  get("parent_model", this->parentModel);
  get("parent_link", this->parentLink);
  get("child_model", this->childModel);
  get("child_link", this->childLink);
  get("command_topic", this->commandTopic);
  get("attach_topic", this->attachTopic);
  get("detach_topic", this->detachTopic);
  get("state_topic", this->stateTopic);
  if (_sdf->HasElement("max_attach_distance"))
    this->maxAttachDistance = _sdf->Get<double>("max_attach_distance");
  if (_sdf->HasElement("max_attach_angle"))
    this->maxAttachAngle = _sdf->Get<double>("max_attach_angle");
  if (_sdf->HasElement("auto_attach_on_start"))
    this->autoAttachOnStart = _sdf->Get<bool>("auto_attach_on_start");
  if (_sdf->HasElement("require_proximity"))
    this->requireProximity = _sdf->Get<bool>("require_proximity");
  if (_sdf->HasElement("nest_pose"))
    this->nestPose = _sdf->Get<math::Pose3d>("nest_pose");
  if (_sdf->HasElement("snap_to_nest"))
    this->snapToNest = _sdf->Get<bool>("snap_to_nest");
  if (_sdf->HasElement("state_period"))
    this->statePeriod = _sdf->Get<double>("state_period");

  std::function<void(const msgs::StringMsg &)> cmdCb =
      [this](const msgs::StringMsg &m) { this->OnCommand(m); };
  std::function<void(const msgs::Empty &)> attachCb =
      [this](const msgs::Empty &m) { this->OnAttachEmpty(m); };
  std::function<void(const msgs::Empty &)> detachCb =
      [this](const msgs::Empty &m) { this->OnDetachEmpty(m); };

  if (!this->node.Subscribe(this->commandTopic, cmdCb))
    gzerr << "LinkAttacher: failed to subscribe " << this->commandTopic << "\n";
  if (!this->node.Subscribe(this->attachTopic, attachCb))
    gzerr << "LinkAttacher: failed to subscribe " << this->attachTopic << "\n";
  if (!this->node.Subscribe(this->detachTopic, detachCb))
    gzerr << "LinkAttacher: failed to subscribe " << this->detachTopic << "\n";

  this->statePub = this->node.Advertise<msgs::StringMsg>(this->stateTopic);
  this->validConfig = true;
  Log("ready: " + this->parentModel + "::" + this->parentLink + " <-> " +
      this->childModel + "::" + this->childLink +
      " (max " + std::to_string(this->maxAttachDistance) + " m, " +
      std::to_string(this->maxAttachAngle) + " rad)");
}

/////////////////////////////////////////////////
void LinkAttacher::OnCommand(const msgs::StringMsg &_msg)
{
  std::string s = _msg.data();
  std::transform(s.begin(), s.end(), s.begin(),
                 [](unsigned char c) { return std::toupper(c); });
  std::lock_guard<std::mutex> g(this->requestMutex);
  if (s == "ATTACH")
    this->attachRequested = true;
  else if (s == "DETACH")
    this->detachRequested = true;
  else
    Log("unknown command '" + _msg.data() + "'");
}

/////////////////////////////////////////////////
void LinkAttacher::OnAttachEmpty(const msgs::Empty &)
{
  std::lock_guard<std::mutex> g(this->requestMutex);
  this->attachRequested = true;
}

/////////////////////////////////////////////////
void LinkAttacher::OnDetachEmpty(const msgs::Empty &)
{
  std::lock_guard<std::mutex> g(this->requestMutex);
  this->detachRequested = true;
}

/////////////////////////////////////////////////
void LinkAttacher::PublishState()
{
  msgs::StringMsg m;
  m.set_data(this->isAttached ? "attached" : "detached");
  this->statePub.Publish(m);
  this->lastStatePub = std::chrono::steady_clock::now();
}

/////////////////////////////////////////////////
bool LinkAttacher::EntitiesAlive(const EntityComponentManager &_ecm) const
{
  return this->parentModelEntity != kNullEntity &&
         this->childModelEntity != kNullEntity &&
         this->parentLinkEntity != kNullEntity &&
         this->childLinkEntity != kNullEntity &&
         _ecm.HasEntity(this->parentModelEntity) &&
         _ecm.HasEntity(this->childModelEntity) &&
         _ecm.HasEntity(this->parentLinkEntity) &&
         _ecm.HasEntity(this->childLinkEntity);
}

/////////////////////////////////////////////////
bool LinkAttacher::ResolveEntities(EntityComponentManager &_ecm)
{
  auto findModel = [&](const std::string &name) -> Entity {
    Entity found = kNullEntity;
    _ecm.Each<components::Model, components::Name>(
        [&](const Entity &_e, const components::Model *,
            const components::Name *_n) -> bool {
          if (_n->Data() == name) {
            found = _e;
            return false;
          }
          return true;
        });
    return found;
  };

  this->parentModelEntity = findModel(this->parentModel);
  this->childModelEntity = findModel(this->childModel);
  this->parentLinkEntity = kNullEntity;
  this->childLinkEntity = kNullEntity;
  if (this->parentModelEntity == kNullEntity ||
      this->childModelEntity == kNullEntity)
    return false;

  this->parentLinkEntity = _ecm.EntityByComponents(
      components::Link(), components::ParentEntity(this->parentModelEntity),
      components::Name(this->parentLink));
  this->childLinkEntity = _ecm.EntityByComponents(
      components::Link(), components::ParentEntity(this->childModelEntity),
      components::Name(this->childLink));
  return this->parentLinkEntity != kNullEntity &&
         this->childLinkEntity != kNullEntity;
}

/////////////////////////////////////////////////
math::Pose3d LinkAttacher::NestWorldPose(
    const EntityComponentManager &_ecm) const
{
  const math::Pose3d pp = worldPose(this->parentModelEntity, _ecm);
  return math::Pose3d(
      pp.Pos() + pp.Rot().RotateVector(this->nestPose.Pos()),
      pp.Rot() * this->nestPose.Rot());
}

/////////////////////////////////////////////////
bool LinkAttacher::GatesOk(const EntityComponentManager &_ecm,
                           std::string &_why) const
{
  if (!this->requireProximity)
    return true;
  const math::Pose3d nest = this->NestWorldPose(_ecm);
  const math::Pose3d cp = worldPose(this->childModelEntity, _ecm);
  const double dist = (nest.Pos() - cp.Pos()).Length();
  if (dist > this->maxAttachDistance) {
    _why = std::to_string(dist) + " m from the nest (limit " +
           std::to_string(this->maxAttachDistance) + " m)";
    return false;
  }
  if (this->maxAttachAngle > 0.0) {
    const math::Quaterniond dq = nest.Rot().Inverse() * cp.Rot();
    const double angle =
        2.0 * std::acos(std::min(1.0, std::abs(dq.W())));
    if (angle > this->maxAttachAngle) {
      _why = std::to_string(angle) + " rad off the nest orientation (limit " +
             std::to_string(this->maxAttachAngle) + " rad)";
      return false;
    }
  }
  return true;
}

/////////////////////////////////////////////////
void LinkAttacher::BeginAttach(EntityComponentManager &_ecm, bool _skipGates)
{
  if (this->isAttached || this->weldCountdown >= 0)
    return;
  std::string why;
  if (!_skipGates && !this->GatesOk(_ecm, why)) {
    Log("ATTACH refused: " + why);
    this->PublishState();
    return;
  }
  if (this->snapToNest) {
    _ecm.SetComponentData<components::WorldPoseCmd>(
        this->childModelEntity, this->NestWorldPose(_ecm));
    this->weldCountdown = kWeldDelayTicks;
  } else {
    this->weldCountdown = 0;
  }
}

/////////////////////////////////////////////////
void LinkAttacher::CreateJoint(EntityComponentManager &_ecm)
{
  this->jointEntity = _ecm.CreateEntity();
  _ecm.CreateComponent(
      this->jointEntity,
      components::DetachableJoint({this->parentLinkEntity,
                                   this->childLinkEntity, "fixed"}));
  this->isAttached = true;
  this->PublishState();
  Log("ATTACH " + this->parentModel + " <-> " + this->childModel);
}

/////////////////////////////////////////////////
void LinkAttacher::DoDetach(EntityComponentManager &_ecm)
{
  this->weldCountdown = -1;
  if (!this->isAttached)
    return;
  if (this->jointEntity != kNullEntity) {
    _ecm.RequestRemoveEntity(this->jointEntity);
    this->jointEntity = kNullEntity;
  }
  this->isAttached = false;
  this->PublishState();
  Log("DETACH " + this->parentModel + " <-> " + this->childModel);
}

/////////////////////////////////////////////////
void LinkAttacher::Forget(EntityComponentManager &_ecm)
{
  // One of the models went away. The joint cannot outlive its links.
  if (this->jointEntity != kNullEntity && _ecm.HasEntity(this->jointEntity))
    _ecm.RequestRemoveEntity(this->jointEntity);
  const bool was = this->isAttached || this->weldCountdown >= 0;
  this->jointEntity = kNullEntity;
  this->isAttached = false;
  this->weldCountdown = -1;
  this->triedAutoAttach = false;  // a respawned child docks again
  this->parentModelEntity = this->childModelEntity = kNullEntity;
  this->parentLinkEntity = this->childLinkEntity = kNullEntity;
  if (was) {
    this->PublishState();
    Log("a docked model was removed; latch released");
  }
}

/////////////////////////////////////////////////
void LinkAttacher::PreUpdate(const UpdateInfo &_info,
                             EntityComponentManager &_ecm)
{
  if (!this->validConfig)
    return;

  if (this->statePeriod > 0.0 &&
      std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                    this->lastStatePub).count() >=
          this->statePeriod)
    this->PublishState();

  if (_info.paused)
    return;

  bool wantAttach = false;
  bool wantDetach = false;
  {
    std::lock_guard<std::mutex> g(this->requestMutex);
    wantAttach = this->attachRequested;
    wantDetach = this->detachRequested;
    this->attachRequested = false;
    this->detachRequested = false;
  }

  if (!this->EntitiesAlive(_ecm)) {
    if (this->parentModelEntity != kNullEntity ||
        this->childModelEntity != kNullEntity || this->isAttached)
      this->Forget(_ecm);
    if (this->resolveBackoff > 0 && !wantAttach) {
      --this->resolveBackoff;
      return;
    }
    if (!this->ResolveEntities(_ecm)) {
      this->resolveBackoff = kResolveBackoffTicks;
      this->parentModelEntity = this->childModelEntity = kNullEntity;
      if (wantAttach)
        Log("ATTACH refused: " + this->parentModel + " or " +
            this->childModel + " is not in the world");
      return;
    }
  }

  // Docked spawn: seat and weld once both vehicles exist (no gates).
  if (this->autoAttachOnStart && !this->triedAutoAttach) {
    this->triedAutoAttach = true;
    this->BeginAttach(_ecm, /*skipGates=*/true);
  }

  if (wantDetach)
    this->DoDetach(_ecm);
  if (wantAttach)
    this->BeginAttach(_ecm, /*skipGates=*/false);

  if (this->weldCountdown == 0) {
    this->weldCountdown = -1;
    this->CreateJoint(_ecm);
  } else if (this->weldCountdown > 0) {
    --this->weldCountdown;
  }
}
}  // namespace udrone

GZ_ADD_PLUGIN(udrone::LinkAttacher,
              gz::sim::System,
              udrone::LinkAttacher::ISystemConfigure,
              udrone::LinkAttacher::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(udrone::LinkAttacher, "gz::sim::systems::LinkAttacher")
GZ_ADD_PLUGIN_ALIAS(udrone::LinkAttacher, "udrone::LinkAttacher")
