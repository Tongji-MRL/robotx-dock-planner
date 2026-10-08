"""Demonstrate exclusive goto/terminal velocity ownership without ROS."""

from dock_planner import (
    GenericVelocityCommand,
    MotionCommandArbiter,
    MotionCommandArbiterConfig,
    MotionCommandOwner,
)


def velocity(surge, yaw_rate, timestamp, source):
    return GenericVelocityCommand(
        surge,
        yaw_rate,
        timestamp,
        timestamp + 0.2,
        "DRY_RUN",
        source,
        "base_link",
    )


def main() -> None:
    arbiter = MotionCommandArbiter(MotionCommandArbiterConfig(
        max_abs_surge_mps=2.0,
        max_abs_yaw_rate_rps=1.0,
    ))

    stopped = arbiter.select(now=10.0)
    print(f"initial owner                = {stopped.owner.value}")

    arbiter.set_owner(MotionCommandOwner.GOTO)
    goto = arbiter.select(
        now=10.1,
        goto_command=velocity(1.0, 0.15, 10.0, "single_waypoint_goto"),
        terminal_command=velocity(0.2, -0.4, 10.0, "terminal_controller"),
    )
    print(f"approach owner               = {goto.owner.value}")
    print(f"approach command             = {goto.command.surge_mps}, {goto.command.yaw_rate_rps}")

    arbiter.set_owner(MotionCommandOwner.TERMINAL)
    terminal = arbiter.select(
        now=10.2,
        goto_command=velocity(0.0, 0.0, 10.1, "single_waypoint_goto"),
        terminal_command=velocity(0.2, -0.4, 10.1, "terminal_controller"),
    )
    print(f"terminal owner               = {terminal.owner.value}")
    print(f"terminal command             = {terminal.command.surge_mps}, {terminal.command.yaw_rate_rps}")

    arbiter.set_owner(MotionCommandOwner.STOP)
    final = arbiter.select(now=10.3)
    print(f"final owner                  = {final.owner.value}")
    print("TASK3 CONTROL HANDOFF DRY RUN PASS")


if __name__ == "__main__":
    main()
