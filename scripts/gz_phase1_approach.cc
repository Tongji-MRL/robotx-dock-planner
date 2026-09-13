#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <optional>
#include <regex>
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

double JsonNumber(const std::string &json, const std::string &object,
                  const std::string &field)
{
  const std::regex pattern(
      "\\\"" + object + "\\\"\\s*:\\s*\\{[^}]*\\\"" + field +
      "\\\"\\s*:\\s*([-+0-9.eE]+)");
  std::smatch match;
  if (!std::regex_search(json, match, pattern))
    throw std::runtime_error("Missing numeric field " + object + "." + field);
  return std::stod(match[1].str());
}
}

int main(int argc, char **argv)
{
  const std::string targetFile = argc > 1 ? argv[1] : "generated/phase1_target.json";
  std::ifstream input(targetFile);
  if (!input)
  {
    std::cerr << "Cannot read target file: " << targetFile << '\n';
    return 2;
  }
  const std::string json((std::istreambuf_iterator<char>(input)),
                         std::istreambuf_iterator<char>());
  double targetX{};
  double targetY{};
  double targetYaw{};
  try
  {
    targetX = JsonNumber(json, "approach_pose", "x");
    targetY = JsonNumber(json, "approach_pose", "y");
    targetYaw = JsonNumber(json, "approach_pose", "yaw");
  }
  catch (const std::exception &error)
  {
    std::cerr << error.what() << '\n';
    return 2;
  }

  constexpr double kRateHz = 20.0;
  constexpr double kBaseThrust = 30.0;
  constexpr double kMaxThrust = 50.0;
  constexpr double kTurnGain = 30.0;
  constexpr double kMaxTurn = 25.0;
  constexpr double kAcceptanceRadius = 1.5;
  constexpr double kTimeoutSeconds = 180.0;
  constexpr double kPoseStaleSeconds = 1.0;

  std::signal(SIGINT, OnSignal);
  std::signal(SIGTERM, OnSignal);
  const std::string poseTopic =
      "/world/scan_dock_deliver_task/dynamic_pose/info";
  const std::string leftTopic = "/wamv/thrusters/left/thrust";
  const std::string rightTopic = "/wamv/thrusters/right/thrust";

  gz::transport::Node node;
  auto leftPublisher = node.Advertise<gz::msgs::Double>(leftTopic);
  auto rightPublisher = node.Advertise<gz::msgs::Double>(rightTopic);
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
    std::cerr << "Failed to subscribe to " << poseTopic << '\n';
    return 3;
  }

  auto getSample = [&]() -> std::optional<Sample>
  {
    std::lock_guard<std::mutex> lock(sampleMutex);
    return latest;
  };
  const auto discoveryDeadline = std::chrono::steady_clock::now() +
                                 std::chrono::seconds(15);
  while ((!leftPublisher.HasConnections() ||
          !rightPublisher.HasConnections() || !getSample()) &&
         std::chrono::steady_clock::now() < discoveryDeadline && !interrupted)
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  if (!leftPublisher.HasConnections() || !rightPublisher.HasConnections() ||
      !getSample())
  {
    std::cerr << "Missing thruster subscriber or WAM-V pose\n";
    return 4;
  }

  // The launcher is started only after the official release log is observed.
  // This extra delay lets residual spawn/release motion settle.
  std::this_thread::sleep_for(std::chrono::seconds(3));
  const auto initial = getSample();
  if (!initial)
    return 5;

  std::cout << std::fixed << std::setprecision(9) << std::boolalpha
            << "left_subscriber=" << leftPublisher.HasConnections() << '\n'
            << "right_subscriber=" << rightPublisher.HasConnections() << '\n'
            << "initial_x=" << initial->x << '\n'
            << "initial_y=" << initial->y << '\n'
            << "initial_yaw=" << initial->yaw << '\n'
            << "target_x=" << targetX << '\n'
            << "target_y=" << targetY << '\n'
            << "target_yaw=" << targetYaw << '\n'
            << "acceptance_radius=" << kAcceptanceRadius << '\n'
            << "base_thrust=" << kBaseThrust << '\n';

  gz::msgs::Double leftMessage;
  gz::msgs::Double rightMessage;
  gz::msgs::Double zeroMessage;
  zeroMessage.set_data(0.0);
  bool zeroOk = false;
  bool reached = false;
  bool timedOut = false;
  bool publishFailed = false;
  int cycles = 0;
  double maxPublishSkewUs = 0.0;
  double lastDistance = std::hypot(targetX - initial->x, targetY - initial->y);
  double lastBearingError = 0.0;
  double lastLeft = 0.0;
  double lastRight = 0.0;
  const auto start = std::chrono::steady_clock::now();
  auto next = start;
  auto nextLog = start;
  const auto period = std::chrono::duration<double>(1.0 / kRateHz);

  try
  {
    while (!interrupted)
    {
      const auto now = std::chrono::steady_clock::now();
      const double elapsed = std::chrono::duration<double>(now - start).count();
      if (elapsed >= kTimeoutSeconds)
      {
        timedOut = true;
        break;
      }
      const auto sample = getSample();
      if (!sample || std::chrono::duration<double>(now - sample->received).count() >
                         kPoseStaleSeconds)
        throw std::runtime_error("WAM-V pose became stale");

      const double dx = targetX - sample->x;
      const double dy = targetY - sample->y;
      lastDistance = std::hypot(dx, dy);
      if (lastDistance <= kAcceptanceRadius)
      {
        reached = true;
        break;
      }
      const double bearing = std::atan2(dy, dx);
      lastBearingError = Wrap(bearing - sample->yaw);
      const double absError = std::abs(lastBearingError);

      double base = kBaseThrust;
      if (lastDistance < 10.0)
        base *= std::clamp((lastDistance - kAcceptanceRadius) / 8.5, 0.20, 1.0);
      if (absError > 0.80)
        base = 0.0;
      else if (absError > 0.45)
        base *= 0.25;
      else if (absError > 0.25)
        base *= 0.55;

      const double turn = std::clamp(kTurnGain * lastBearingError,
                                     -kMaxTurn, kMaxTurn);
      // Positive yaw error needs CCW torque: increase the starboard (right)
      // thrust and decrease port (left). Both commands are published here.
      lastLeft = std::clamp(base - turn, -kMaxThrust, kMaxThrust);
      lastRight = std::clamp(base + turn, -kMaxThrust, kMaxThrust);
      leftMessage.set_data(lastLeft);
      rightMessage.set_data(lastRight);
      const auto leftTime = std::chrono::steady_clock::now();
      const bool leftOk = leftPublisher.Publish(leftMessage);
      const auto rightTime = std::chrono::steady_clock::now();
      const bool rightOk = rightPublisher.Publish(rightMessage);
      if (!leftOk || !rightOk)
        throw std::runtime_error("Thrust publish failed");
      maxPublishSkewUs = std::max(
          maxPublishSkewUs,
          std::chrono::duration<double, std::micro>(rightTime - leftTime).count());
      ++cycles;

      if (now >= nextLog)
      {
        std::cout << "sample elapsed=" << elapsed
                  << " distance=" << lastDistance
                  << " heading_error=" << lastBearingError
                  << " left=" << lastLeft
                  << " right=" << lastRight << '\n';
        nextLog += std::chrono::seconds(1);
      }
      next += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
      std::this_thread::sleep_until(next);
    }
  }
  catch (const std::exception &error)
  {
    std::cerr << "controller_error=" << error.what() << '\n';
    publishFailed = true;
  }

  // Safety stop for every normal, timeout, signal, and exception path after
  // publishers have connected.
  for (int i = 0; i < 5; ++i)
  {
    zeroOk = leftPublisher.Publish(zeroMessage) &&
             rightPublisher.Publish(zeroMessage);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  const auto stopTime = std::chrono::steady_clock::now();
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  const auto final = getSample();
  if (!final)
    return 6;

  const double finalDx = targetX - final->x;
  const double finalDy = targetY - final->y;
  const double finalDistance = std::hypot(finalDx, finalDy);
  const double finalBearingError = finalDistance > 1e-6
      ? Wrap(std::atan2(finalDy, finalDx) - final->yaw) : 0.0;
  const double finalPoseHeadingError = Wrap(targetYaw - final->yaw);
  const double runtime = std::chrono::duration<double>(stopTime - start).count();

  std::cout << "result_reached=" << reached << '\n'
            << "result_timed_out=" << timedOut << '\n'
            << "result_interrupted=" << interrupted.load() << '\n'
            << "result_publish_failed=" << publishFailed << '\n'
            << "runtime=" << runtime << '\n'
            << "control_cycles=" << cycles << '\n'
            << "max_left_right_skew_us=" << maxPublishSkewUs << '\n'
            << "final_x=" << final->x << '\n'
            << "final_y=" << final->y << '\n'
            << "final_yaw=" << final->yaw << '\n'
            << "final_distance=" << finalDistance << '\n'
            << "final_bearing_heading_error=" << finalBearingError << '\n'
            << "final_approach_pose_heading_error=" << finalPoseHeadingError << '\n'
            << "last_command_left=" << lastLeft << '\n'
            << "last_command_right=" << lastRight << '\n'
            << "zero_published=" << zeroOk << '\n';

  return reached && zeroOk && !timedOut && !interrupted && !publishFailed ? 0 : 7;
}
