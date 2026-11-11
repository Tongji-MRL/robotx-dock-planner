"""Move the Gazebo demonstration WAM-V along the generated search line."""

from __future__ import annotations

import json
import math
from pathlib import Path
import subprocess
import sys
import time


SERVICE = "/world/task3_geofence_search/set_pose"


def service_available() -> bool:
    result = subprocess.run(
        ["gz", "service", "-l"],
        check=False,
        capture_output=True,
        text=True,
        timeout=3,
    )
    return SERVICE in result.stdout.splitlines()


def set_pose(x: float, y: float, yaw: float) -> bool:
    request = (
        f'name: "search_usv", position: {{x: {x:.6f}, y: {y:.6f}, z: 0.22}}, '
        f'orientation: {{z: {math.sin(yaw / 2.0):.8f}, w: {math.cos(yaw / 2.0):.8f}}}'
    )
    result = subprocess.run(
        [
            "gz", "service", "-s", SERVICE,
            "--reqtype", "gz.msgs.Pose",
            "--reptype", "gz.msgs.Boolean",
            "--timeout", "1500",
            "--req", request,
        ],
        check=False,
        capture_output=True,
        text=True,
        timeout=3,
    )
    return result.returncode == 0 and "true" in result.stdout


def interpolate(stations, reverse=False, steps=12):
    route = list(reversed(stations)) if reverse else list(stations)
    last_yaw = float(route[0]["yaw"])
    for first, second in zip(route, route[1:]):
        yaw = math.atan2(second["y"] - first["y"], second["x"] - first["x"])
        last_yaw = yaw
        for index in range(steps):
            ratio = index / steps
            yield (
                first["x"] + (second["x"] - first["x"]) * ratio,
                first["y"] + (second["y"] - first["y"]) * ratio,
                yaw,
            )
    last = route[-1]
    yield last["x"], last["y"], last_yaw


def main() -> int:
    if len(sys.argv) != 2:
        raise SystemExit("usage: gazebo_geofence_animation.py METADATA.json")
    metadata = json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))
    stations = metadata["primary_stations"]
    deadline = time.monotonic() + 15.0
    while time.monotonic() < deadline and not service_available():
        time.sleep(0.25)
    if not service_available():
        print("Gazebo pose service did not become ready", file=sys.stderr)
        return 1
    while True:
        for reverse in (False, True):
            for x, y, yaw in interpolate(stations, reverse=reverse):
                if not set_pose(x, y, yaw):
                    return 0
                time.sleep(0.10)


if __name__ == "__main__":
    raise SystemExit(main())
