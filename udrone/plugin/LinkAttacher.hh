/*
 * CoRAL / aquatic port of Open-UAV gazebo_ros_link_attacher for gz-sim8.
 * Creates a fixed DetachableJoint between two model links on ATTACH;
 * removes it on DETACH. Gates on re-attach: distance to the nest and angle.
 * The child is seated in a fixed nest pose before the weld, so the docked
 * offset is the same on every boot and after every re-attach.
 */
#ifndef UDRONE_LINK_ATTACHER_HH_
#define UDRONE_LINK_ATTACHER_HH_

#include <chrono>
#include <memory>
#include <mutex>
#include <string>

#include <gz/math/Pose3.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/empty.pb.h>
#include <gz/msgs/stringmsg.pb.h>

namespace udrone
{
class LinkAttacher final :
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
public:
  LinkAttacher() = default;
  ~LinkAttacher() override = default;

  void Configure(const gz::sim::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 gz::sim::EntityComponentManager &_ecm,
                 gz::sim::EventManager &_eventMgr) override;

  void PreUpdate(const gz::sim::UpdateInfo &_info,
                 gz::sim::EntityComponentManager &_ecm) override;

private:
  void OnCommand(const gz::msgs::StringMsg &_msg);
  void OnAttachEmpty(const gz::msgs::Empty &);
  void OnDetachEmpty(const gz::msgs::Empty &);
  void PublishState();

  bool EntitiesAlive(const gz::sim::EntityComponentManager &_ecm) const;
  bool ResolveEntities(gz::sim::EntityComponentManager &_ecm);
  gz::math::Pose3d NestWorldPose(
      const gz::sim::EntityComponentManager &_ecm) const;
  bool GatesOk(const gz::sim::EntityComponentManager &_ecm,
               std::string &_why) const;
  void BeginAttach(gz::sim::EntityComponentManager &_ecm, bool _skipGates);
  void CreateJoint(gz::sim::EntityComponentManager &_ecm);
  void DoDetach(gz::sim::EntityComponentManager &_ecm);
  void Forget(gz::sim::EntityComponentManager &_ecm);

  gz::transport::Node node;
  gz::transport::Node::Publisher statePub;

  std::string parentModel{"heron_0"};
  std::string parentLink{"base_link"};
  std::string childModel{"xudrone_1"};
  std::string childLink{"base_link"};
  std::string commandTopic{"/udrone_dock/command"};
  std::string attachTopic{"/model/heron_0/udrone_dock/attach"};
  std::string detachTopic{"/model/heron_0/udrone_dock/detach"};
  std::string stateTopic{"/model/heron_0/udrone_dock/state"};
  /// Largest distance from the nest at which ATTACH is accepted (m).
  double maxAttachDistance{0.6};
  /// Largest rotation away from the nest orientation (rad); <= 0 disables.
  double maxAttachAngle{0.5236};
  bool autoAttachOnStart{true};
  bool requireProximity{true};
  /// Child model pose in the parent model frame while docked.
  gz::math::Pose3d nestPose{0, 0, 0, 0, 0, 0};
  /// Seat the child in the nest before welding.
  bool snapToNest{true};
  /// Seconds between state messages (also sent on every change).
  double statePeriod{1.0};

  gz::sim::Entity parentModelEntity{gz::sim::kNullEntity};
  gz::sim::Entity childModelEntity{gz::sim::kNullEntity};
  gz::sim::Entity parentLinkEntity{gz::sim::kNullEntity};
  gz::sim::Entity childLinkEntity{gz::sim::kNullEntity};
  gz::sim::Entity jointEntity{gz::sim::kNullEntity};

  std::mutex requestMutex;
  bool attachRequested{false};
  bool detachRequested{false};
  bool isAttached{false};
  bool validConfig{false};
  bool triedAutoAttach{false};
  /// >= 0 while a weld is pending: ticks left until the joint is created.
  int weldCountdown{-1};
  /// Ticks until the next attempt to find the two models.
  int resolveBackoff{0};
  std::chrono::steady_clock::time_point lastStatePub{};
};
}  // namespace udrone

#endif
