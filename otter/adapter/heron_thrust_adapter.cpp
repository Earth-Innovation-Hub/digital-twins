// PX4 differential-rover motor commands -> Heron jet thrust.
//
// PX4's rover controller treats a motor command as proportional to effort: its speed and
// yaw-rate feed-forwards are straight lines through zero. This adapter therefore maps the
// normalised command u in [-1, 1] to thrust linearly, 33.6 N per unit, and clips each jet
// to the Clearpath Heron's range of -19.88 N (astern) to +33.6 N (ahead), with the same
// 0.05 s first-order response as before. The slope is the same ahead and astern, so a
// differential command turns the boat without also pushing it forward until the astern
// jet reaches its limit (u = -0.59).
//
// Until 2026-11-11 this file applied Clearpath's command-to-thrust table
// (heron_description snippets.xacro: 0.2 -> 2.2 N, 0.4 -> 9.5 N, 0.6 -> 21.3 N, ...).
// That table is the Heron's own motor-driver curve. Fed with PX4's linear commands it
// gave a dead zone around zero and a loop gain that changed five-fold with throttle;
// the old source is /root/heron_thrust_adapter.cpp.bak-20261111.
//
// rover_differential's Motor 1 (control[0] = throttle - speed_diff) is the right side:
// a positive (clockwise) yaw rate raises Motor 2. PX4 SIM_GZ_WH_FUNC1 = 101 arrives as
// motor_speed velocity[0] and drives the starboard jet; Motor 2 is velocity[1] and drives
// port. Both jets push forward for positive commands (SIM_GZ_WH_REV 0), and Gazebo's
// Thruster cmd_thrust is positive along the jet axis (+x, forward).
//
// PX4 sends whole numbers offset by 100, so SIM_GZ_WH_MIN/MAX 0..200 gives 100 steps
// each way (0.34 N). The earlier 75..125 gave 25 steps (1.3 N).
//
// If PX4 stops sending (it died, or the bridge dropped), the jets go to zero after
// kCommandTimeout instead of holding their last thrust.
//
// build: g++ -O2 -std=c++17 heron_thrust_adapter.cpp -o heron_thrust_adapter \
//            $(pkg-config --cflags --libs gz-transport13 gz-msgs10)
#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <iostream>
#include <mutex>
#include <thread>

#include <gz/msgs/actuators.pb.h>
#include <gz/msgs/double.pb.h>
#include <gz/transport/Node.hh>

namespace
{
constexpr double kInputScale = 100.0;    // SIM_GZ_WH_MIN/MAX 0..200 around 100
constexpr double kMaxAhead = 33.6;       // N per jet, Clearpath
constexpr double kMaxAstern = -19.88;    // N per jet, Clearpath
constexpr double kTimeConstant = 0.05;   // s
constexpr double kCommandTimeout = 0.5;  // s without a PX4 message before the jets stop

double thrust(double raw)
{
  const double u = std::clamp(raw / kInputScale, -1.0, 1.0);
  return std::clamp(u * kMaxAhead, kMaxAstern, kMaxAhead);
}
}

int main()
{
  using Clock = std::chrono::steady_clock;
  gz::transport::Node node;
  auto port = node.Advertise<gz::msgs::Double>("/model/heron_0/joint/port_jet_joint/cmd_thrust");
  auto starboard = node.Advertise<gz::msgs::Double>("/model/heron_0/joint/starboard_jet_joint/cmd_thrust");

  std::mutex lock;
  double target[2] = {0.0, 0.0};
  Clock::time_point last_command = Clock::now() - std::chrono::hours(1);

  std::function<void(const gz::msgs::Actuators &)> callback =
      [&](const gz::msgs::Actuators &_msg)
      {
        if (_msg.velocity_size() < 2)
          return;
        std::lock_guard<std::mutex> guard(lock);
        target[0] = thrust(_msg.velocity(1));  // port
        target[1] = thrust(_msg.velocity(0));  // starboard
        last_command = Clock::now();
      };
  if (!node.Subscribe<gz::msgs::Actuators>("/model/heron_0/command/motor_speed", callback))
  {
    std::cerr << "failed to subscribe to PX4 Heron actuator topic" << std::endl;
    return 1;
  }

  std::cout << "Heron thrust adapter ready (linear, 33.6 N per unit command, "
            << kMaxAstern << ".." << kMaxAhead << " N per jet)" << std::endl;
  constexpr double dt = 0.01;
  const double alpha = dt / (kTimeConstant + dt);
  double out[2] = {0.0, 0.0};
  unsigned long ticks = 0;
  bool stale = false;
  while (true)
  {
    {
      std::lock_guard<std::mutex> guard(lock);
      const bool now_stale = std::chrono::duration<double>(Clock::now() - last_command).count() > kCommandTimeout;
      if (now_stale && !stale && (std::abs(target[0]) > 0.1 || std::abs(target[1]) > 0.1))
        std::cout << "no PX4 motor command for " << kCommandTimeout << " s: jets to zero" << std::endl;
      stale = now_stale;
      for (int i = 0; i < 2; ++i)
      {
        if (stale)
          target[i] = 0.0;
        out[i] += alpha * (target[i] - out[i]);
        if (std::abs(out[i]) < 1e-6)
          out[i] = 0.0;
      }
    }
    gz::msgs::Double portMsg, starboardMsg;
    portMsg.set_data(out[0]);
    starboardMsg.set_data(out[1]);
    port.Publish(portMsg);
    starboard.Publish(starboardMsg);
    // One line a minute while the jets are idle, one every 10 s while they push.
    const bool pushing = std::abs(out[0]) > 0.1 || std::abs(out[1]) > 0.1;
    if (++ticks % (pushing ? 1000 : 6000) == 1)
      std::cout << "jets port=" << out[0] << " starboard=" << out[1] << " N" << std::endl;
    std::this_thread::sleep_for(std::chrono::duration<double>(dt));
  }
}
