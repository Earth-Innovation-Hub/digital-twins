// PX4 UUV motor commands -> uDrone T200 thrust (otter-style Thruster plugins).
//
//   in   /<model>/command/motor_speed                  gz.msgs.Actuators
//        PX4's ESC output (SIM_GZ_EC). Note: no /model prefix. PX4 also
//        advertises /model/<model>/command/motor_speed, but that is its wheel
//        output (SIM_GZ_WH) and stays silent for this airframe.
//   out  /model/<model>/joint/rotor_N_joint/cmd_thrust  gz.msgs.Double (N)
//
// Model name from UDRONE_GZ_MODEL (default xudrone_0; CoRAL dual uses xudrone_1).
// SIM_GZ_EC publishes PWM-ish values (1100..1900); stopped/disarmed is 0.
// One adapter per model: a second copy exits at once (lock in /tmp).
#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>

#include <gz/msgs/actuators.pb.h>
#include <gz/msgs/double.pb.h>
#include <gz/transport/Node.hh>

namespace {
constexpr int kN = 4;
constexpr double kMaxN = 36.0;       // T200 ~36 N at 12 V
constexpr double kTimeConstant = 0.05;
constexpr double kCommandTimeout = 0.5;

double to_thrust(double raw) {
  if (!std::isfinite(raw)) {
    return 0.0;
  }
  if (std::abs(raw) < 1e-3) {
    return 0.0;
  }
  if (raw > 500.0) {
    const double u = std::clamp((raw - 1500.0) / 400.0, -1.0, 1.0);
    return u * kMaxN;
  }
  if (raw >= -1.0 && raw <= 1.0) {
    return raw * kMaxN;
  }
  return 0.0;
}
}

int main() {
  using Clock = std::chrono::steady_clock;
  const char *env_model = std::getenv("UDRONE_GZ_MODEL");
  const std::string model = (env_model && env_model[0]) ? env_model : "xudrone_0";
  const std::string prefix = "/model/" + model;

  // Two adapters on one model would both drive the thrusters.
  const std::string lock_path = "/tmp/udrone_thrust_adapter." + model + ".lock";
  const int lock_fd = open(lock_path.c_str(), O_CREAT | O_RDWR, 0644);
  if (lock_fd < 0 || flock(lock_fd, LOCK_EX | LOCK_NB) != 0) {
    std::cerr << "uDrone thrust adapter already running for " << model
              << "; exiting" << std::endl;
    return 0;
  }

  gz::transport::Node node;
  std::array<gz::transport::Node::Publisher, kN> pubs = {
      node.Advertise<gz::msgs::Double>(prefix + "/joint/rotor_0_joint/cmd_thrust"),
      node.Advertise<gz::msgs::Double>(prefix + "/joint/rotor_1_joint/cmd_thrust"),
      node.Advertise<gz::msgs::Double>(prefix + "/joint/rotor_2_joint/cmd_thrust"),
      node.Advertise<gz::msgs::Double>(prefix + "/joint/rotor_3_joint/cmd_thrust"),
  };

  std::mutex lock;
  std::array<double, kN> target{};
  Clock::time_point last = Clock::now() - std::chrono::hours(1);

  auto cb = [&](const gz::msgs::Actuators &msg) {
    if (msg.velocity_size() < kN) return;
    std::lock_guard<std::mutex> g(lock);
    for (int i = 0; i < kN; ++i) target[i] = to_thrust(msg.velocity(i));
    last = Clock::now();
  };
  const std::string cmd_topic = "/" + model + "/command/motor_speed";
  if (!node.Subscribe<gz::msgs::Actuators>(cmd_topic, cb)) {
    std::cerr << "failed to subscribe to " << cmd_topic << std::endl;
    return 1;
  }
  std::cout << "uDrone thrust adapter ready on " << model << ": " << cmd_topic
            << " -> " << prefix << "/joint/rotor_N_joint/cmd_thrust"
            << " (T200 ±" << kMaxN << " N x4)" << std::endl;

  constexpr double dt = 0.01;
  const double alpha = dt / (kTimeConstant + dt);
  std::array<double, kN> out{};
  while (true) {
    {
      std::lock_guard<std::mutex> g(lock);
      const bool stale = std::chrono::duration<double>(Clock::now() - last).count() > kCommandTimeout;
      for (int i = 0; i < kN; ++i) {
        if (stale) target[i] = 0.0;
        out[i] += alpha * (target[i] - out[i]);
        if (std::abs(out[i]) < 1e-6) out[i] = 0.0;
      }
    }
    for (int i = 0; i < kN; ++i) {
      gz::msgs::Double m;
      m.set_data(out[i]);
      pubs[i].Publish(m);
    }
    std::this_thread::sleep_for(std::chrono::duration<double>(dt));
  }
}
