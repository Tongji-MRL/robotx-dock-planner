#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
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
#include <vector>

#include <gz/msgs/contacts.pb.h>
#include <gz/msgs/double.pb.h>
#include <gz/msgs/param.pb.h>
#include <gz/msgs/pose.pb.h>
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

enum class State
{
  GO_TO_APPROACH,
  ALIGN,
  GO_TO_ENTRANCE,
  GO_TO_STOP,
  DONE,
  FAILED,
};

const char *StateName(State state)
{
  switch (state)
  {
    case State::GO_TO_APPROACH: return "GO_TO_APPROACH";
    case State::ALIGN: return "ALIGN";
    case State::GO_TO_ENTRANCE: return "GO_TO_ENTRANCE";
    case State::GO_TO_STOP: return "GO_TO_STOP";
    case State::DONE: return "DONE";
    case State::FAILED: return "FAILED";
  }
  return "UNKNOWN";
}

struct Transition
{
  State from;
  State to;
  double duration{};
  Sample pose;
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

std::int64_t SteadyNanoseconds()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
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

  double approachX{};
  double approachY{};
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
    approachX = JsonNumber(json, "approach_pose", "x");
    approachY = JsonNumber(json, "approach_pose", "y");
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
  constexpr double kPoseStaleSeconds = 1.0;
  // The official 300 s baseline spent 138.5 s in GO_TO_APPROACH. Keep the
  // validated controller unchanged apart from this minimal speed increase.
  constexpr double kApproachBaseThrust = 45.0;
  constexpr double kApproachMaxThrust = 50.0;
  constexpr double kApproachTurnGain = 30.0;
  constexpr double kApproachMaxTurn = 25.0;
  constexpr double kApproachAcceptanceRadius = 1.5;
  constexpr double kApproachTimeoutSeconds = 180.0;
  constexpr double kAlignTurnGain = 25.0;
  constexpr double kAlignMaxTurn = 12.0;
  constexpr double kAlignTolerance = 5.0 * M_PI / 180.0;
  constexpr double kAlignStableSeconds = 1.5;
  constexpr double kAlignSettleSeconds = 2.0;
  constexpr double kAlignTimeoutSeconds = 60.0;
  constexpr double kMaxAlignDrift = 2.0;
  constexpr double kEntranceBaseThrust = 12.0;
  constexpr double kEntranceMaxThrust = 20.0;
  constexpr double kEntranceTurnGain = 25.0;
  constexpr double kEntranceMaxTurn = 8.0;
  constexpr double kEntranceAcceptanceRadius = 0.9;
  constexpr double kEntranceHeadingTolerance = 7.0 * M_PI / 180.0;
  constexpr double kEntranceTimeoutSeconds = 100.0;
  constexpr double kEntranceHeadingOffsetLimit = 0.18;
  constexpr double kStopBaseThrust = 6.0;
  constexpr double kStopMaxThrust = 10.0;
  constexpr double kStopTurnGain = 18.0;
  constexpr double kStopMaxTurn = 3.0;
  constexpr double kStopHeadingOffsetGain = 0.20;
  constexpr double kStopHeadingOffsetLimit = 0.10;
  constexpr double kStopAcceptanceRadius = 0.45;
  constexpr double kStopAcceptanceLongitudinal = 0.40;
  constexpr double kStopAcceptanceLateral = 0.35;
  constexpr double kStopHeadingTolerance = 7.0 * M_PI / 180.0;
  constexpr double kStopStableSeconds = 1.0;
  constexpr double kStopSettleSeconds = 1.5;
  constexpr double kStopZeroForwardDistance = 0.40;
  constexpr double kStopTimeoutSeconds = 90.0;
  constexpr double kDockedSeconds = 10.0;
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
    std::cerr << "Unsafe or inconsistent planner-generated geometry\n";
    return 2;
  }

  std::signal(SIGINT, OnSignal);
  std::signal(SIGTERM, OnSignal);
  const std::string poseTopic =
      "/world/scan_dock_deliver_task/dynamic_pose/info";
  const std::string leftTopic = "/wamv/thrusters/left/thrust";
  const std::string rightTopic = "/wamv/thrusters/right/thrust";
  const std::string contactsTopic = "/vrx/contacts";
  const std::string taskInfoTopic = "/vrx/task/info";
  const std::string bay2InternalTopic = "/vrx/dock_2022/bay_2/contain";

  gz::transport::Node node;
  auto leftPublisher = node.Advertise<gz::msgs::Double>(leftTopic);
  auto rightPublisher = node.Advertise<gz::msgs::Double>(rightTopic);
  std::mutex sampleMutex;
  std::optional<Sample> latest;
  std::atomic_bool collision{false};
  std::mutex collisionMutex;
  std::string collisionPair;
  std::atomic<double> taskElapsed{-1.0};
  std::atomic<double> taskRemaining{-1.0};
  std::atomic_bool taskTimedOut{false};
  std::mutex taskStateMutex;
  std::string taskState;
  std::atomic_bool internalZone{false};
  std::atomic<std::int64_t> internalEnterNanoseconds{-1};

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
  std::function<void(const gz::msgs::Param &)> taskCallback =
      [&](const gz::msgs::Param &_msg)
  {
    const auto &params = _msg.params();
    auto elapsed = params.find("elapsed_time");
    if (elapsed != params.end())
      taskElapsed = elapsed->second.double_value();
    auto remaining = params.find("remaining_time");
    if (remaining != params.end())
      taskRemaining = remaining->second.double_value();
    auto timedOut = params.find("timed_out");
    if (timedOut != params.end())
      taskTimedOut = timedOut->second.bool_value();
    auto state = params.find("state");
    if (state != params.end())
    {
      std::lock_guard<std::mutex> lock(taskStateMutex);
      taskState = state->second.string_value();
    }
  };
  std::function<void(const gz::msgs::Pose &)> containCallback =
      [&](const gz::msgs::Pose &_msg)
  {
    if (_msg.name() != "wamv")
      return;
    for (const auto &data : _msg.header().data())
    {
      if (data.key() != "state" || data.value().empty())
        continue;
      const bool inside = data.value(0) == "1";
      if (inside && !internalZone.exchange(true))
        internalEnterNanoseconds = SteadyNanoseconds();
      else if (!inside)
      {
        internalZone = false;
        internalEnterNanoseconds = -1;
      }
      break;
    }
  };

  if (!node.Subscribe<gz::msgs::Pose_V>(poseTopic, poseCallback) ||
      !node.Subscribe<gz::msgs::Contacts>(contactsTopic, contactsCallback) ||
      !node.Subscribe<gz::msgs::Param>(taskInfoTopic, taskCallback) ||
      !node.Subscribe<gz::msgs::Pose>(bay2InternalTopic, containCallback))
  {
    std::cerr << "Failed to subscribe to a required topic\n";
    return 3;
  }

  auto getSample = [&]() -> std::optional<Sample>
  {
    std::lock_guard<std::mutex> lock(sampleMutex);
    return latest;
  };
  auto getTaskState = [&]() -> std::string
  {
    std::lock_guard<std::mutex> lock(taskStateMutex);
    return taskState;
  };
  const auto discoveryDeadline = std::chrono::steady_clock::now() +
                                 std::chrono::seconds(90);
  while ((!leftPublisher.HasConnections() ||
          !rightPublisher.HasConnections() || !getSample() ||
          taskElapsed.load() < 0.0 || getTaskState() != "running") &&
         std::chrono::steady_clock::now() < discoveryDeadline && !interrupted)
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  if (!leftPublisher.HasConnections() || !rightPublisher.HasConnections() ||
      !getSample() || taskElapsed.load() < 0.0 ||
      getTaskState() != "running")
  {
    std::cerr << "Missing subscriber/pose or task did not enter running\n";
    return 4;
  }

  std::this_thread::sleep_for(std::chrono::seconds(3));
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

  gz::msgs::Double leftMessage;
  gz::msgs::Double rightMessage;
  gz::msgs::Double zeroMessage;
  zeroMessage.set_data(0.0);
  State state = State::GO_TO_APPROACH;
  std::vector<Transition> transitions;
  std::string failureReason;
  bool zeroPublished = true;
  bool corridorExceeded = false;
  bool poseStale = false;
  bool publishFailed = false;
  bool backWallRisk = false;
  bool stopOvershot = false;
  bool stageTimedOut = false;
  bool stopZoneLatched = false;
  bool stopReached = false;
  bool dockedObserved = false;
  double minimumBackWallClearance = wallClearanceAt(*initial);
  double maximumAbsoluteThrust = 0.0;
  double finalStopDistance = std::numeric_limits<double>::infinity();
  double finalHeadingError = 0.0;
  double finalLateral = 0.0;
  double lastLeft = 0.0;
  double lastRight = 0.0;
  Sample alignInitial{};
  std::optional<std::chrono::steady_clock::time_point> stableSince;
  const auto controllerStart = std::chrono::steady_clock::now();
  auto stageStart = controllerStart;
  auto next = controllerStart;
  auto nextLog = controllerStart;
  const auto period = std::chrono::duration<double>(1.0 / kRateHz);
  const double taskElapsedAtStart = taskElapsed.load();

  auto transitionTo = [&](State nextState, const Sample &sample,
                          const std::chrono::steady_clock::time_point now)
  {
    const double duration =
        std::chrono::duration<double>(now - stageStart).count();
    transitions.push_back(Transition{state, nextState, duration, sample});
    std::cout << "transition_from=" << StateName(state)
              << " transition_to=" << StateName(nextState)
              << " stage_duration=" << duration
              << " transition_x=" << sample.x
              << " transition_y=" << sample.y
              << " transition_yaw=" << sample.yaw << '\n';
    state = nextState;
    stageStart = now;
    stableSince.reset();
  };
  auto fail = [&](const std::string &reason, const Sample &sample,
                  const std::chrono::steady_clock::time_point now)
  {
    if (state == State::FAILED)
      return;
    failureReason = reason;
    transitionTo(State::FAILED, sample, now);
  };

  std::cout << std::fixed << std::setprecision(9) << std::boolalpha
            << "state=" << StateName(state) << '\n'
            << "initial_x=" << initial->x << '\n'
            << "initial_y=" << initial->y << '\n'
            << "initial_yaw=" << initial->yaw << '\n'
            << "approach_x=" << approachX << '\n'
            << "approach_y=" << approachY << '\n'
            << "entrance_x=" << entranceX << '\n'
            << "entrance_y=" << entranceY << '\n'
            << "entrance_heading=" << entranceYaw << '\n'
            << "stop_x=" << stopX << '\n'
            << "stop_y=" << stopY << '\n'
            << "stop_yaw=" << stopYaw << '\n'
            << "task_elapsed_at_start=" << taskElapsedAtStart << '\n'
            << "approach_base_thrust=" << kApproachBaseThrust << '\n'
            << "entrance_base_thrust=" << kEntranceBaseThrust << '\n'
            << "stop_base_thrust=" << kStopBaseThrust << '\n'
            << "corridor_limit=" << corridorLimit << '\n'
            << "planned_back_wall_clearance="
            << plannedBackWallClearance << '\n';

  while (state != State::DONE && state != State::FAILED && !interrupted)
  {
    const auto now = std::chrono::steady_clock::now();
    const auto sample = getSample();
    if (!sample)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
      continue;
    }
    const double stageElapsed =
        std::chrono::duration<double>(now - stageStart).count();
    const double totalElapsed =
        std::chrono::duration<double>(now - controllerStart).count();
    lastLeft = 0.0;
    lastRight = 0.0;

    if (collision)
      fail("collision", *sample, now);
    else if (std::chrono::duration<double>(now - sample->received).count() >
             kPoseStaleSeconds)
    {
      poseStale = true;
      fail("pose_stale", *sample, now);
    }
    else if (taskTimedOut)
    {
      stageTimedOut = true;
      fail("official_running_timeout", *sample, now);
    }

    if (state == State::GO_TO_APPROACH)
    {
      if (stageElapsed >= kApproachTimeoutSeconds)
      {
        stageTimedOut = true;
        fail("approach_timeout", *sample, now);
      }
      else
      {
        const double dx = approachX - sample->x;
        const double dy = approachY - sample->y;
        const double distance = std::hypot(dx, dy);
        if (distance <= kApproachAcceptanceRadius)
        {
          alignInitial = *sample;
          transitionTo(State::ALIGN, *sample, now);
        }
        else
        {
          const double error = Wrap(std::atan2(dy, dx) - sample->yaw);
          double base = kApproachBaseThrust;
          if (distance < 10.0)
            base *= std::clamp(
                (distance - kApproachAcceptanceRadius) / 8.5, 0.20, 1.0);
          if (std::abs(error) > 0.80)
            base = 0.0;
          else if (std::abs(error) > 0.45)
            base *= 0.25;
          else if (std::abs(error) > 0.25)
            base *= 0.55;
          const double turn = std::clamp(
              kApproachTurnGain * error,
              -kApproachMaxTurn, kApproachMaxTurn);
          lastLeft = std::clamp(
              base - turn, -kApproachMaxThrust, kApproachMaxThrust);
          lastRight = std::clamp(
              base + turn, -kApproachMaxThrust, kApproachMaxThrust);
        }
      }
    }
    else if (state == State::ALIGN)
    {
      const double drift = std::hypot(
          sample->x - alignInitial.x, sample->y - alignInitial.y);
      if (stageElapsed >= kAlignTimeoutSeconds)
      {
        stageTimedOut = true;
        fail("align_timeout", *sample, now);
      }
      else if (drift > kMaxAlignDrift)
        fail("align_drift_exceeded", *sample, now);
      else
      {
        const double error = Wrap(entranceYaw - sample->yaw);
        if (std::abs(error) < kAlignTolerance)
        {
          if (!stableSince)
            stableSince = now;
          if (std::chrono::duration<double>(now - *stableSince).count() >=
              kAlignStableSeconds)
            transitionTo(State::GO_TO_ENTRANCE, *sample, now);
        }
        else
          stableSince.reset();
        if (state == State::ALIGN && stageElapsed >= kAlignSettleSeconds)
        {
          const double turn = std::clamp(
              kAlignTurnGain * error, -kAlignMaxTurn, kAlignMaxTurn);
          lastLeft = -turn;
          lastRight = turn;
        }
      }
    }
    else if (state == State::GO_TO_ENTRANCE)
    {
      const double relX = sample->x - entranceX;
      const double relY = sample->y - entranceY;
      const double longitudinal = relX * headingCos + relY * headingSin;
      const double lateral = -relX * headingSin + relY * headingCos;
      const double remainingAlong = -longitudinal;
      const double distance = std::hypot(relX, relY);
      const double headingError = Wrap(entranceYaw - sample->yaw);
      if (stageElapsed >= kEntranceTimeoutSeconds)
      {
        stageTimedOut = true;
        fail("entrance_timeout", *sample, now);
      }
      else if (std::abs(lateral) > bayWidth / 2.0 ||
               (remainingAlong < 4.0 && remainingAlong > -1.0 &&
                std::abs(lateral) > corridorLimit))
      {
        corridorExceeded = true;
        fail("corridor_exceeded", *sample, now);
      }
      else if (remainingAlong < -1.0)
        fail("passed_entrance", *sample, now);
      else if (distance <= kEntranceAcceptanceRadius &&
               std::abs(headingError) <= kEntranceHeadingTolerance)
        transitionTo(State::GO_TO_STOP, *sample, now);
      else
      {
        double desiredHeading = entranceYaw - std::clamp(
            0.25 * lateral,
            -kEntranceHeadingOffsetLimit, kEntranceHeadingOffsetLimit);
        double error = Wrap(desiredHeading - sample->yaw);
        double base = kEntranceBaseThrust;
        if (distance < 5.0)
          base *= std::clamp(
              (distance - kEntranceAcceptanceRadius) / 4.1, 0.25, 1.0);
        if (std::abs(error) > 0.45)
          base = 0.0;
        else if (std::abs(error) > 0.25)
          base *= 0.30;
        if (distance <= kEntranceAcceptanceRadius)
        {
          base = 0.0;
          desiredHeading = entranceYaw;
          error = Wrap(desiredHeading - sample->yaw);
        }
        double turn = std::clamp(
            kEntranceTurnGain * error,
            -kEntranceMaxTurn, kEntranceMaxTurn);
        if (stageElapsed < 1.0)
        {
          base = 0.0;
          turn = 0.0;
        }
        lastLeft = std::clamp(
            base - turn, -kEntranceMaxThrust, kEntranceMaxThrust);
        lastRight = std::clamp(
            base + turn, -kEntranceMaxThrust, kEntranceMaxThrust);
      }
    }
    else if (state == State::GO_TO_STOP)
    {
      const double longitudinal = longitudinalAt(*sample);
      const double remainingAlong = stopLongitudinal - longitudinal;
      const double lateral = lateralAt(*sample);
      const double distance = std::hypot(
          stopX - sample->x, stopY - sample->y);
      const double headingError = Wrap(entranceYaw - sample->yaw);
      const double wallClearance = wallClearanceAt(*sample);
      minimumBackWallClearance = std::min(
          minimumBackWallClearance, wallClearance);
      finalStopDistance = distance;
      finalHeadingError = headingError;
      finalLateral = lateral;

      const std::int64_t internalStart = internalEnterNanoseconds.load();
      if (internalZone && internalStart >= 0)
      {
        const double dwell = static_cast<double>(
            SteadyNanoseconds() - internalStart) / 1e9;
        dockedObserved = dwell >= kDockedSeconds;
      }

      if (stageElapsed >= kStopTimeoutSeconds)
      {
        stageTimedOut = true;
        fail("stop_timeout", *sample, now);
      }
      else if (std::abs(lateral) > corridorLimit)
      {
        corridorExceeded = true;
        fail("corridor_exceeded", *sample, now);
      }
      else if (longitudinal > stopLongitudinal + kMaximumStopOvershoot)
      {
        stopOvershot = true;
        fail("stop_overshot", *sample, now);
      }
      else if (wallClearance < kMinimumBackWallClearance)
      {
        backWallRisk = true;
        fail("back_wall_risk", *sample, now);
      }
      else
      {
        const bool withinTolerance =
            distance <= kStopAcceptanceRadius &&
            std::abs(remainingAlong) <= kStopAcceptanceLongitudinal &&
            std::abs(lateral) <= kStopAcceptanceLateral &&
            std::abs(headingError) <= kStopHeadingTolerance;
        if (withinTolerance)
        {
          stopZoneLatched = true;
          if (!stableSince)
            stableSince = now;
          if (std::chrono::duration<double>(now - *stableSince).count() >=
              kStopStableSeconds)
            stopReached = true;
        }
        else
          stableSince.reset();
        if (remainingAlong <= kStopZeroForwardDistance)
          stopZoneLatched = true;

        if (stopReached && dockedObserved)
          transitionTo(State::DONE, *sample, now);
        else if (!stopZoneLatched && stageElapsed >= kStopSettleSeconds)
        {
          const double desiredHeading = entranceYaw - std::clamp(
              kStopHeadingOffsetGain * lateral,
              -kStopHeadingOffsetLimit, kStopHeadingOffsetLimit);
          const double error = Wrap(desiredHeading - sample->yaw);
          double base = kStopBaseThrust;
          if (remainingAlong < 2.0)
            base *= std::clamp(
                (remainingAlong - kStopZeroForwardDistance) / 1.45,
                0.15, 1.0);
          if (std::abs(error) > 0.30)
            base = 0.0;
          else if (std::abs(error) > 0.18)
            base *= 0.35;
          const double turn = std::clamp(
              kStopTurnGain * error, -kStopMaxTurn, kStopMaxTurn);
          lastLeft = std::clamp(
              base - turn, -kStopMaxThrust, kStopMaxThrust);
          lastRight = std::clamp(
              base + turn, -kStopMaxThrust, kStopMaxThrust);
        }
      }
    }

    maximumAbsoluteThrust = std::max(
        maximumAbsoluteThrust,
        std::max(std::abs(lastLeft), std::abs(lastRight)));
    leftMessage.set_data(lastLeft);
    rightMessage.set_data(lastRight);
    const bool leftOk = leftPublisher.Publish(leftMessage);
    const bool rightOk = rightPublisher.Publish(rightMessage);
    if (!leftOk || !rightOk)
    {
      publishFailed = true;
      fail("publish_failure", *sample, now);
    }

    if (now >= nextLog)
    {
      std::cout << "sample total_elapsed=" << totalElapsed
                << " state=" << StateName(state)
                << " stage_elapsed=" << stageElapsed
                << " x=" << sample->x
                << " y=" << sample->y
                << " yaw=" << sample->yaw
                << " task_elapsed=" << taskElapsed.load()
                << " task_remaining=" << taskRemaining.load()
                << " internal_zone=" << internalZone.load()
                << " docked_observed=" << dockedObserved
                << " left=" << lastLeft
                << " right=" << lastRight << '\n';
      nextLog += std::chrono::seconds(1);
    }
    if (state == State::DONE || state == State::FAILED)
      break;
    next += std::chrono::duration_cast<
        std::chrono::steady_clock::duration>(period);
    std::this_thread::sleep_until(next);
  }

  if (interrupted && state != State::DONE && state != State::FAILED)
  {
    const auto sample = getSample();
    if (sample)
      fail("interrupted", *sample, std::chrono::steady_clock::now());
  }

  for (int i = 0; i < 10; ++i)
  {
    zeroPublished = leftPublisher.Publish(zeroMessage) && zeroPublished;
    zeroPublished = rightPublisher.Publish(zeroMessage) && zeroPublished;
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  const auto stopTime = std::chrono::steady_clock::now();
  std::this_thread::sleep_for(std::chrono::seconds(1));
  const auto final = getSample();
  if (!final)
    return 6;

  finalStopDistance = std::hypot(stopX - final->x, stopY - final->y);
  finalHeadingError = Wrap(entranceYaw - final->yaw);
  finalLateral = lateralAt(*final);
  const double finalBackWallClearance = wallClearanceAt(*final);
  minimumBackWallClearance = std::min(
      minimumBackWallClearance, finalBackWallClearance);
  const double totalRuntime =
      std::chrono::duration<double>(stopTime - controllerStart).count();
  const double taskElapsedAtEnd = taskElapsed.load();
  const bool withinOfficialWindow =
      state == State::DONE && !taskTimedOut &&
      taskElapsedAtEnd >= 0.0 && taskElapsedAtEnd <= 300.0;
  std::string finalTaskState;
  {
    std::lock_guard<std::mutex> lock(taskStateMutex);
    finalTaskState = taskState;
  }
  std::string finalCollisionPair;
  {
    std::lock_guard<std::mutex> lock(collisionMutex);
    finalCollisionPair = collisionPair;
  }

  std::cout << "result_state=" << StateName(state) << '\n'
            << "result_failure_reason=" << failureReason << '\n'
            << "result_total_runtime=" << totalRuntime << '\n'
            << "result_task_elapsed_at_start=" << taskElapsedAtStart << '\n'
            << "result_task_elapsed_at_end=" << taskElapsedAtEnd << '\n'
            << "result_task_state=" << finalTaskState << '\n'
            << "result_within_official_300s_window="
            << withinOfficialWindow << '\n'
            << "result_docked_observed=" << dockedObserved << '\n'
            << "result_stop_reached=" << stopReached << '\n'
            << "result_final_x=" << final->x << '\n'
            << "result_final_y=" << final->y << '\n'
            << "result_final_yaw=" << final->yaw << '\n'
            << "result_final_stop_distance=" << finalStopDistance << '\n'
            << "result_final_heading_error=" << finalHeadingError << '\n'
            << "result_final_lateral_error=" << finalLateral << '\n'
            << "result_final_back_wall_clearance="
            << finalBackWallClearance << '\n'
            << "result_minimum_back_wall_clearance="
            << minimumBackWallClearance << '\n'
            << "result_maximum_absolute_thrust="
            << maximumAbsoluteThrust << '\n'
            << "result_collision=" << collision.load() << '\n'
            << "result_collision_pair=" << finalCollisionPair << '\n'
            << "result_corridor_exceeded=" << corridorExceeded << '\n'
            << "result_pose_stale=" << poseStale << '\n'
            << "result_timeout=" << stageTimedOut << '\n'
            << "result_publish_failed=" << publishFailed << '\n'
            << "result_back_wall_risk=" << backWallRisk << '\n'
            << "result_stop_overshot=" << stopOvershot << '\n'
            << "result_zero_published=" << zeroPublished << '\n';
  for (std::size_t i = 0; i < transitions.size(); ++i)
  {
    const auto &entry = transitions[i];
    std::cout << "transition_" << i << "_from=" << StateName(entry.from) << '\n'
              << "transition_" << i << "_to=" << StateName(entry.to) << '\n'
              << "transition_" << i << "_duration=" << entry.duration << '\n'
              << "transition_" << i << "_x=" << entry.pose.x << '\n'
              << "transition_" << i << "_y=" << entry.pose.y << '\n'
              << "transition_" << i << "_yaw=" << entry.pose.yaw << '\n';
  }

  return state == State::DONE && withinOfficialWindow && dockedObserved &&
                 stopReached && zeroPublished && !collision &&
                 !corridorExceeded && !poseStale && !stageTimedOut &&
                 !publishFailed && !backWallRisk && !stopOvershot
             ? 0
             : 7;
}
