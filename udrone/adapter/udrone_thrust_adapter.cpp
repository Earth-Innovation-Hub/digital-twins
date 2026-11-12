// PX4 UUV motor commands -> uDrone T200 thrust (otter-style Thruster plugins).
// Subscribes to /xudrone_0/command/motor_speed (SIM_GZ_EC).
// SIM_GZ_EC publishes PWM-ish values (1100..1900); stopped/disarmed is 0.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <mutex>
#include <thread>
#include <array>

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
  // Stopped / disarmed / NaN-mapped-zero from SIM_GZ
  if (std::abs(raw) < 1e-3) {
    return 0.0;
  }
  // PWM center 1500, span 400 -> [-1,1] (SIM_GZ_EC)
  if (raw > 500.0) {
    const double u = std::clamp((raw - 1500.0) / 400.0, -1.0, 1.0);
    return u * kMaxN;
  }
  // Already normalized [-1,1]
  if (raw >= -1.0 && raw <= 1.0) {
    return raw * kMaxN;
  }
  return 0.0;
}
}

int main() {
  using Clock = std::chrono::steady_clock;
  gz::transport::Node node;
  const char *joints[kN] = {
      "/model/xudrone_0/joint/rotor_0_joint/cmd_thrust",
      "/model/xudrone_0/joint/rotor_1_joint/cmd_thrust",
      "/model/xudrone_0/joint/rotor_2_joint/cmd_thrust",
      "/model/xudrone_0/joint/rotor_3_joint/cmd_thrust",
  };
  std::array<gz::transport::Node::Publisher, kN> pubs = {
      node.Advertise<gz::msgs::Double>(joints[0]),
      node.Advertise<gz::msgs::Double>(joints[1]),
      node.Advertise<gz::msgs::Double>(joints[2]),
      node.Advertise<gz::msgs::Double>(joints[3]),
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
  if (!node.Subscribe<gz::msgs::Actuators>("/xudrone_0/command/motor_speed", cb)) {
    std::cerr << "failed to subscribe to /xudrone_0/command/motor_speed" << std::endl;
    return 1;
  }
  std::cout << "uDrone thrust adapter ready (T200 ±" << kMaxN << " N x4)" << std::endl;

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
