#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <functional>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>

#include <gz/msgs/double.pb.h>
#include <gz/msgs/pose_v.pb.h>
#include <gz/transport/Node.hh>

namespace
{
std::atomic_bool interrupted{false};

void OnSignal(int)
{
  interrupted = true;
}

double Wrap(double value)
{
  return std::atan2(std::sin(value), std::cos(value));
}

struct Sample
{
  double x{};
  double y{};
  double yaw{};
  std::chrono::steady_clock::time_point received;
};
}

int main(int argc, char **argv)
{
  const double thrust = argc > 1 ? std::stod(argv[1]) : 40.0;
  const double duration = argc > 2 ? std::stod(argv[2]) : 1.0;
  const double rate = argc > 3 ? std::stod(argv[3]) : 20.0;
  if (thrust < 0.0 || thrust > 100.0 || duration <= 0.0 || duration > 2.0 ||
      rate < 1.0 || rate > 100.0)
  {
    std::cerr << "Unsafe calibration arguments\n";
    return 2;
  }

  std::signal(SIGINT, OnSignal);
  std::signal(SIGTERM, OnSignal);
  const std::string poseTopic =
      "/world/scan_dock_deliver_task/dynamic_pose/info";
  const std::string leftTopic = "/wamv/thrusters/left/thrust";
  const std::string rightTopic = "/wamv/thrusters/right/thrust";

  gz::transport::Node node;
  auto left = node.Advertise<gz::msgs::Double>(leftTopic);
  auto right = node.Advertise<gz::msgs::Double>(rightTopic);
  std::mutex sampleMutex;
  std::optional<Sample> latest;
  std::function<void(const gz::msgs::Pose_V &)> poseCallback =
      [&](const gz::msgs::Pose_V &_msg)
  {
    for (const auto &pose : _msg.pose())
    {
      if (pose.name() != "wamv")
        continue;
      const auto &q = pose.orientation();
      const double yaw = std::atan2(
          2.0 * (q.w() * q.z() + q.x() * q.y()),
          1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
      std::lock_guard<std::mutex> lock(sampleMutex);
      latest = Sample{pose.position().x(), pose.position().y(), yaw,
                      std::chrono::steady_clock::now()};
    }
  };
  if (!node.Subscribe<gz::msgs::Pose_V>(poseTopic, poseCallback))
  {
    std::cerr << "Failed to subscribe to " << poseTopic << "\n";
    return 3;
  }

  auto getSample = [&]() -> std::optional<Sample>
  {
    std::lock_guard<std::mutex> lock(sampleMutex);
    return latest;
  };
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::seconds(10);
  while ((!left.HasConnections() || !right.HasConnections() || !getSample()) &&
         std::chrono::steady_clock::now() < deadline && !interrupted)
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  if (!left.HasConnections() || !right.HasConnections() || !getSample())
  {
    std::cerr << "Missing thruster subscriber or WAM-V pose\n";
    return 4;
  }

  std::cout << std::boolalpha
            << "left_subscriber=" << left.HasConnections() << '\n'
            << "right_subscriber=" << right.HasConnections() << '\n';

  // Let post-release motion settle, then estimate initial world-frame velocity.
  std::this_thread::sleep_for(std::chrono::seconds(3));
  const auto velocityStart = getSample();
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  const auto initial = getSample();
  if (!velocityStart || !initial)
    return 5;
  const double initialDt = std::chrono::duration<double>(
      initial->received - velocityStart->received).count();
  const double vx = (initial->x - velocityStart->x) / initialDt;
  const double vy = (initial->y - velocityStart->y) / initialDt;
  const double yawRate = Wrap(initial->yaw - velocityStart->yaw) / initialDt;

  gz::msgs::Double command;
  gz::msgs::Double zero;
  command.set_data(thrust);
  zero.set_data(0.0);
  bool zeroOk = false;
  double maxSkewUs = 0.0;
  int cycles = 0;
  const auto period = std::chrono::duration<double>(1.0 / rate);
  const auto pulseStart = std::chrono::steady_clock::now();
  auto next = pulseStart;
  try
  {
    while (!interrupted &&
           std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                         pulseStart).count() < duration)
    {
      const auto tLeft = std::chrono::steady_clock::now();
      const bool leftOk = left.Publish(command);
      const auto tRight = std::chrono::steady_clock::now();
      const bool rightOk = right.Publish(command);
      if (!leftOk || !rightOk)
        throw std::runtime_error("thrust publish failed");
      maxSkewUs = std::max(maxSkewUs,
          std::chrono::duration<double, std::micro>(tRight - tLeft).count());
      ++cycles;
      next += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
      std::this_thread::sleep_until(next);
    }
  }
  catch (const std::exception &error)
  {
    std::cerr << error.what() << '\n';
    interrupted = true;
  }

  const auto pulseEnd = std::chrono::steady_clock::now();
  // Publish zero repeatedly in one loop so both subscribers receive it even if
  // the first transport packet is delayed.
  for (int i = 0; i < 3; ++i)
  {
    zeroOk = left.Publish(zero) && right.Publish(zero);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  const double actualDuration = std::chrono::duration<double>(
      pulseEnd - pulseStart).count();
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  const auto final = getSample();
  if (!final)
    return 6;

  const double dx = final->x - initial->x;
  const double dy = final->y - initial->y;
  const double forward = dx * std::cos(initial->yaw) +
                         dy * std::sin(initial->yaw);
  const double lateral = -dx * std::sin(initial->yaw) +
                          dy * std::cos(initial->yaw);

  std::cout << std::fixed << std::setprecision(9)
            << "initial_x=" << initial->x << '\n'
            << "initial_y=" << initial->y << '\n'
            << "initial_yaw=" << initial->yaw << '\n'
            << "initial_vx_est=" << vx << '\n'
            << "initial_vy_est=" << vy << '\n'
            << "initial_yaw_rate_est=" << yawRate << '\n'
            << "command_left=" << thrust << '\n'
            << "command_right=" << thrust << '\n'
            << "requested_duration=" << duration << '\n'
            << "actual_duration=" << actualDuration << '\n'
            << "publish_cycles=" << cycles << '\n'
            << "max_left_right_skew_us=" << maxSkewUs << '\n'
            << "final_x=" << final->x << '\n'
            << "final_y=" << final->y << '\n'
            << "final_yaw=" << final->yaw << '\n'
            << "displacement=" << std::hypot(dx, dy) << '\n'
            << "forward_component=" << forward << '\n'
            << "lateral_component=" << lateral << '\n'
            << "delta_yaw=" << Wrap(final->yaw - initial->yaw) << '\n'
            << "zero_published=" << zeroOk << '\n';
  return interrupted ? 7 : 0;
}
