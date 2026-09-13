#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <mutex>
#include <optional>
#include <regex>
#include <stdexcept>
#include <string>
#include <thread>

#include <gz/msgs/contacts.pb.h>
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
  const std::string targetFile =
      argc > 1 ? argv[1] : "generated/phase1_target.json";
  std::ifstream input(targetFile);
  if (!input)
  {
    std::cerr << "Cannot read target file: " << targetFile << '\n';
    return 2;
  }
  const std::string json((std::istreambuf_iterator<char>(input)),
                         std::istreambuf_iterator<char>());

  double entranceX{};
  double entranceY{};
  double entranceYaw{};
  double stopX{};
  double stopY{};
  double stopYaw{};
  double bayWidth{};
  double bayDepth{};
  try
  {
    entranceX = JsonNumber(json, "entrance_pose", "x");
    entranceY = JsonNumber(json, "entrance_pose", "y");
    entranceYaw = JsonNumber(json, "entrance_pose", "yaw");
    stopX = JsonNumber(json, "stop_pose", "x");
    stopY = JsonNumber(json, "stop_pose", "y");
    stopYaw = JsonNumber(json, "stop_pose", "yaw");
    bayWidth = JsonNumber(json, "dock_spec", "bay_width");
    bayDepth = JsonNumber(json, "dock_spec", "bay_depth");
  }
  catch (const std::exception &error)
  {
    std::cerr << error.what() << '\n';
    return 2;
  }

  const double headingCos = std::cos(entranceYaw);
  const double headingSin = std::sin(entranceYaw);
  const double stopRelativeX = stopX - entranceX;
  const double stopRelativeY = stopY - entranceY;
  const double stopLongitudinal =
      stopRelativeX * headingCos + stopRelativeY * headingSin;
  const double stopLateral =
      -stopRelativeX * headingSin + stopRelativeY * headingCos;

  constexpr double kRateHz = 20.0;
  constexpr double kBaseThrust = 6.0;
  constexpr double kMaxThrust = 10.0;
  constexpr double kTurnGain = 18.0;
  constexpr double kMaxTurn = 3.0;
  constexpr double kHeadingOffsetGain = 0.20;
  constexpr double kHeadingOffsetLimit = 0.10;
  constexpr double kAcceptanceRadius = 0.45;
  constexpr double kAcceptanceLongitudinal = 0.40;
  constexpr double kAcceptanceLateral = 0.35;
  constexpr double kHeadingTolerance = 7.0 * M_PI / 180.0;
  constexpr double kStableSeconds = 1.0;
  constexpr double kInitialSettleSeconds = 1.5;
  constexpr double kStopZeroForwardDistance = 0.40;
  constexpr double kTimeoutSeconds = 90.0;
  constexpr double kPoseStaleSeconds = 1.0;
  constexpr double kStartEntranceDistanceLimit = 1.10;
  constexpr double kStartHeadingLimit = 8.0 * M_PI / 180.0;
  constexpr double kWamvCollisionHalfWidthWithMargin = 1.30;
  constexpr double kWamvForwardExtentWithMargin = 2.70;
  constexpr double kMinimumBackWallClearance = 0.70;
  constexpr double kMaximumStopOvershoot = 0.35;
  const double corridorLimit =
      bayWidth / 2.0 - kWamvCollisionHalfWidthWithMargin;
  const double plannedBackWallClearance =
      bayDepth - stopLongitudinal - kWamvForwardExtentWithMargin;

  if (corridorLimit <= 0.0 || stopLongitudinal <= 0.0 ||
      std::abs(stopLateral) > 1e-6 ||
      std::abs(Wrap(stopYaw - entranceYaw)) > 1e-6 ||
      plannedBackWallClearance <= kMinimumBackWallClearance)
  {
    std::cerr << "Unsafe or inconsistent planner-generated stop geometry\n";
    return 2;
  }

  std::signal(SIGINT, OnSignal);
  std::signal(SIGTERM, OnSignal);
  const std::string poseTopic =
      "/world/scan_dock_deliver_task/dynamic_pose/info";
  const std::string leftTopic = "/wamv/thrusters/left/thrust";
  const std::string rightTopic = "/wamv/thrusters/right/thrust";
  const std::string contactsTopic = "/vrx/contacts";

  gz::transport::Node node;
  auto leftPublisher = node.Advertise<gz::msgs::Double>(leftTopic);
  auto rightPublisher = node.Advertise<gz::msgs::Double>(rightTopic);
  std::mutex sampleMutex;
  std::optional<Sample> latest;
  std::atomic_bool collision{false};
  std::mutex collisionMutex;
  std::string collisionPair;

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

  std::function<void(const gz::msgs::Contacts &)> contactsCallback =
      [&](const gz::msgs::Contacts &_msg)
  {
    for (const auto &contact : _msg.contact())
    {
      const std::string first = contact.collision1().name();
      const std::string second = contact.collision2().name();
      if (first.find("wamv") == std::string::npos &&
          second.find("wamv") == std::string::npos)
        continue;
      {
        std::lock_guard<std::mutex> lock(collisionMutex);
        collisionPair = first + " || " + second;
      }
      collision = true;
      break;
    }
  };
  if (!node.Subscribe<gz::msgs::Contacts>(contactsTopic, contactsCallback))
  {
    std::cerr << "Failed to subscribe to " << contactsTopic << '\n';
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

  const auto initial = getSample();
  if (!initial)
    return 5;

  auto longitudinalAt = [&](const Sample &sample)
  {
    return (sample.x - entranceX) * headingCos +
           (sample.y - entranceY) * headingSin;
  };
  auto lateralAt = [&](const Sample &sample)
  {
    return -(sample.x - entranceX) * headingSin +
           (sample.y - entranceY) * headingCos;
  };
  auto wallClearanceAt = [&](const Sample &sample)
  {
    return bayDepth - longitudinalAt(sample) -
           kWamvForwardExtentWithMargin;
  };

  const double initialEntranceDistance =
      std::hypot(initial->x - entranceX, initial->y - entranceY);
  const double initialDistance =
      std::hypot(stopX - initial->x, stopY - initial->y);
  const double initialLongitudinal = longitudinalAt(*initial);
  const double initialLateral = lateralAt(*initial);
  const double initialHeadingError = Wrap(entranceYaw - initial->yaw);
  const bool startValid =
      initialEntranceDistance <= kStartEntranceDistanceLimit &&
      initialLongitudinal >= -kStartEntranceDistanceLimit &&
      initialLongitudinal <= kMaximumStopOvershoot &&
      std::abs(initialLateral) <= corridorLimit &&
      std::abs(initialHeadingError) <= kStartHeadingLimit;

  std::cout << std::fixed << std::setprecision(9) << std::boolalpha
            << "state=STOP\n"
            << "left_subscriber=" << leftPublisher.HasConnections() << '\n'
            << "right_subscriber=" << rightPublisher.HasConnections() << '\n'
            << "phase3_initial_x=" << initial->x << '\n'
            << "phase3_initial_y=" << initial->y << '\n'
            << "phase3_initial_yaw=" << initial->yaw << '\n'
            << "entrance_x=" << entranceX << '\n'
            << "entrance_y=" << entranceY << '\n'
            << "entrance_heading=" << entranceYaw << '\n'
            << "stop_x=" << stopX << '\n'
            << "stop_y=" << stopY << '\n'
            << "stop_yaw=" << stopYaw << '\n'
            << "stop_longitudinal=" << stopLongitudinal << '\n'
            << "phase3_initial_entrance_distance=" << initialEntranceDistance << '\n'
            << "phase3_initial_stop_distance=" << initialDistance << '\n'
            << "phase3_initial_longitudinal=" << initialLongitudinal << '\n'
            << "phase3_initial_lateral=" << initialLateral << '\n'
            << "phase3_initial_heading_error=" << initialHeadingError << '\n'
            << "phase3_start_valid=" << startValid << '\n'
            << "base_thrust=" << kBaseThrust << '\n'
            << "max_thrust=" << kMaxThrust << '\n'
            << "turn_gain=" << kTurnGain << '\n'
            << "max_turn=" << kMaxTurn << '\n'
            << "acceptance_radius=" << kAcceptanceRadius << '\n'
            << "acceptance_lateral=" << kAcceptanceLateral << '\n'
            << "heading_tolerance=" << kHeadingTolerance << '\n'
            << "corridor_limit=" << corridorLimit << '\n'
            << "maximum_stop_overshoot=" << kMaximumStopOvershoot << '\n'
            << "wamv_forward_extent_with_margin="
            << kWamvForwardExtentWithMargin << '\n'
            << "minimum_back_wall_clearance_limit="
            << kMinimumBackWallClearance << '\n'
            << "planned_back_wall_clearance="
            << plannedBackWallClearance << '\n';

  gz::msgs::Double leftMessage;
  gz::msgs::Double rightMessage;
  gz::msgs::Double zeroMessage;
  zeroMessage.set_data(0.0);
  bool reached = false;
  bool timedOut = false;
  bool poseStale = false;
  bool publishFailed = false;
  bool corridorExceeded = false;
  bool stopOvershot = false;
  bool backWallTooClose = false;
  bool stopZoneLatched = false;
  bool oscillationDetected = false;
  bool zeroOk = true;
  int cycles = 0;
  int headingSignChanges = 0;
  int previousHeadingSign = 0;
  double maximumAbsThrust = 0.0;
  double maximumPublishSkewUs = 0.0;
  double minimumBackWallClearance = wallClearanceAt(*initial);
  double lastLeft = 0.0;
  double lastRight = 0.0;

  const auto start = std::chrono::steady_clock::now();
  auto next = start;
  auto nextLog = start;
  const auto period = std::chrono::duration<double>(1.0 / kRateHz);
  std::optional<std::chrono::steady_clock::time_point> stableSince;

  if (startValid)
  {
    while (!interrupted)
    {
      const auto now = std::chrono::steady_clock::now();
      const double elapsed =
          std::chrono::duration<double>(now - start).count();
      if (elapsed >= kTimeoutSeconds)
      {
        timedOut = true;
        break;
      }
      if (collision)
        break;
      const auto sample = getSample();
      if (!sample ||
          std::chrono::duration<double>(now - sample->received).count() >
              kPoseStaleSeconds)
      {
        poseStale = true;
        break;
      }

      const double longitudinal = longitudinalAt(*sample);
      const double lateral = lateralAt(*sample);
      const double remainingAlong = stopLongitudinal - longitudinal;
      const double distance =
          std::hypot(stopX - sample->x, stopY - sample->y);
      const double headingError = Wrap(entranceYaw - sample->yaw);
      const double wallClearance = wallClearanceAt(*sample);
      minimumBackWallClearance =
          std::min(minimumBackWallClearance, wallClearance);

      if (std::abs(lateral) > corridorLimit)
      {
        corridorExceeded = true;
        break;
      }
      if (longitudinal > stopLongitudinal + kMaximumStopOvershoot)
      {
        stopOvershot = true;
        break;
      }
      if (wallClearance < kMinimumBackWallClearance)
      {
        backWallTooClose = true;
        break;
      }

      const bool withinTolerance =
          distance <= kAcceptanceRadius &&
          std::abs(remainingAlong) <= kAcceptanceLongitudinal &&
          std::abs(lateral) <= kAcceptanceLateral &&
          std::abs(headingError) <= kHeadingTolerance;
      if (withinTolerance)
      {
        stopZoneLatched = true;
        if (!stableSince)
          stableSince = now;
        if (std::chrono::duration<double>(now - *stableSince).count() >=
            kStableSeconds)
        {
          reached = true;
          break;
        }
      }
      else
      {
        stableSince.reset();
      }
      if (remainingAlong <= kStopZeroForwardDistance)
        stopZoneLatched = true;

      double base = 0.0;
      double turn = 0.0;
      double controlHeadingError = headingError;
      if (!stopZoneLatched && elapsed >= kInitialSettleSeconds)
      {
        const double desiredHeading = entranceYaw - std::clamp(
            kHeadingOffsetGain * lateral,
            -kHeadingOffsetLimit, kHeadingOffsetLimit);
        controlHeadingError = Wrap(desiredHeading - sample->yaw);
        base = kBaseThrust;
        if (remainingAlong < 2.0)
          base *= std::clamp(
              (remainingAlong - kStopZeroForwardDistance) / 1.45,
              0.15, 1.0);
        if (std::abs(controlHeadingError) > 0.30)
          base = 0.0;
        else if (std::abs(controlHeadingError) > 0.18)
          base *= 0.35;
        turn = std::clamp(kTurnGain * controlHeadingError,
                          -kMaxTurn, kMaxTurn);
      }

      lastLeft = std::clamp(base - turn, -kMaxThrust, kMaxThrust);
      lastRight = std::clamp(base + turn, -kMaxThrust, kMaxThrust);
      maximumAbsThrust = std::max(
          maximumAbsThrust,
          std::max(std::abs(lastLeft), std::abs(lastRight)));
      const int headingSign = controlHeadingError > 0.0174533
                                  ? 1
                                  : (controlHeadingError < -0.0174533 ? -1 : 0);
      if (headingSign != 0 && previousHeadingSign != 0 &&
          headingSign != previousHeadingSign)
        ++headingSignChanges;
      if (headingSign != 0)
        previousHeadingSign = headingSign;
      oscillationDetected = headingSignChanges >= 6;

      leftMessage.set_data(lastLeft);
      rightMessage.set_data(lastRight);
      const auto leftTime = std::chrono::steady_clock::now();
      const bool leftOk = leftPublisher.Publish(leftMessage);
      const auto rightTime = std::chrono::steady_clock::now();
      const bool rightOk = rightPublisher.Publish(rightMessage);
      if (!leftOk || !rightOk)
      {
        publishFailed = true;
        break;
      }
      maximumPublishSkewUs = std::max(
          maximumPublishSkewUs,
          std::chrono::duration<double, std::micro>(rightTime - leftTime).count());
      ++cycles;

      if (now >= nextLog)
      {
        std::cout << "stop elapsed=" << elapsed
                  << " distance=" << distance
                  << " remaining_along=" << remainingAlong
                  << " lateral=" << lateral
                  << " yaw=" << sample->yaw
                  << " heading_error=" << headingError
                  << " control_heading_error=" << controlHeadingError
                  << " back_wall_clearance=" << wallClearance
                  << " stop_zone_latched=" << stopZoneLatched
                  << " left=" << lastLeft
                  << " right=" << lastRight << '\n';
        nextLog += std::chrono::seconds(1);
      }
      next +=
          std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
      std::this_thread::sleep_until(next);
    }
  }

  for (int i = 0; i < 10; ++i)
  {
    zeroOk = leftPublisher.Publish(zeroMessage) && zeroOk;
    zeroOk = rightPublisher.Publish(zeroMessage) && zeroOk;
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  const auto stopTime = std::chrono::steady_clock::now();
  std::this_thread::sleep_for(std::chrono::seconds(1));
  const auto final = getSample();
  if (!final)
    return 6;

  const double finalLongitudinal = longitudinalAt(*final);
  const double finalRemainingAlong = stopLongitudinal - finalLongitudinal;
  const double finalLateral = lateralAt(*final);
  const double finalDistance =
      std::hypot(stopX - final->x, stopY - final->y);
  const double finalHeadingError = Wrap(entranceYaw - final->yaw);
  const double finalBackWallClearance = wallClearanceAt(*final);
  minimumBackWallClearance =
      std::min(minimumBackWallClearance, finalBackWallClearance);
  if (std::abs(finalLateral) > corridorLimit)
    corridorExceeded = true;
  if (finalLongitudinal > stopLongitudinal + kMaximumStopOvershoot)
    stopOvershot = true;
  if (finalBackWallClearance < kMinimumBackWallClearance)
    backWallTooClose = true;
  const bool finalWithinTolerance =
      finalDistance <= kAcceptanceRadius &&
      std::abs(finalRemainingAlong) <= kAcceptanceLongitudinal &&
      std::abs(finalLateral) <= kAcceptanceLateral &&
      std::abs(finalHeadingError) <= kHeadingTolerance;
  const double runtime =
      std::chrono::duration<double>(stopTime - start).count();
  std::string finalCollisionPair;
  {
    std::lock_guard<std::mutex> lock(collisionMutex);
    finalCollisionPair = collisionPair;
  }

  std::cout << "result_phase3_start_valid=" << startValid << '\n'
            << "result_phase3_reached=" << reached << '\n'
            << "result_final_within_tolerance=" << finalWithinTolerance << '\n'
            << "result_collision=" << collision.load() << '\n'
            << "result_corridor_exceeded=" << corridorExceeded << '\n'
            << "result_stop_overshot=" << stopOvershot << '\n'
            << "result_back_wall_too_close=" << backWallTooClose << '\n'
            << "result_timed_out=" << timedOut << '\n'
            << "result_pose_stale=" << poseStale << '\n'
            << "result_interrupted=" << interrupted.load() << '\n'
            << "result_publish_failed=" << publishFailed << '\n'
            << "runtime=" << runtime << '\n'
            << "control_cycles=" << cycles << '\n'
            << "max_left_right_skew_us=" << maximumPublishSkewUs << '\n'
            << "maximum_absolute_thruster_command=" << maximumAbsThrust << '\n'
            << "heading_sign_changes=" << headingSignChanges << '\n'
            << "oscillation_detected=" << oscillationDetected << '\n'
            << "final_x=" << final->x << '\n'
            << "final_y=" << final->y << '\n'
            << "final_yaw=" << final->yaw << '\n'
            << "final_stop_distance=" << finalDistance << '\n'
            << "final_remaining_along=" << finalRemainingAlong << '\n'
            << "final_lateral=" << finalLateral << '\n'
            << "final_heading_error=" << finalHeadingError << '\n'
            << "final_back_wall_clearance=" << finalBackWallClearance << '\n'
            << "minimum_back_wall_clearance=" << minimumBackWallClearance << '\n'
            << "collision_pair=" << finalCollisionPair << '\n'
            << "last_command_left=" << lastLeft << '\n'
            << "last_command_right=" << lastRight << '\n'
            << "zero_published=" << zeroOk << '\n';

  return startValid && reached && finalWithinTolerance && zeroOk &&
                 !collision && !corridorExceeded && !stopOvershot &&
                 !backWallTooClose && !timedOut && !poseStale &&
                 !interrupted && !publishFailed
             ? 0
             : 7;
}
