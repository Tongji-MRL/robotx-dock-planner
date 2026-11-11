"""Generate a Gazebo Garden scene for the USV-only geofence search path."""

from __future__ import annotations

import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

from dock_planner import GeofenceSearchConfig, NEDPoint, plan_geofence_search
from dock_planner.vrx_reference import vrx_dock_2022_bay_to_dockspec


ROOT = Path(__file__).resolve().parents[1]
WORLD_NAME = "task3_geofence_search"
DOCK_POSE = (18.0, 5.0, math.pi / 2.0)
WAMV_MESH = "model://WAM-V-Base/mesh/WAM-V-Base.dae"


def child(parent, tag, text):
    node = ET.SubElement(parent, tag)
    node.text = str(text)
    return node


def visual_model(world, name, x, y, z, yaw, shape, size, color):
    model = ET.SubElement(world, "model", name=name)
    child(model, "static", "true")
    child(model, "pose", f"{x:.6f} {y:.6f} {z:.6f} 0 0 {yaw:.6f}")
    link = ET.SubElement(model, "link", name="link")
    visual = ET.SubElement(link, "visual", name="visual")
    geometry = ET.SubElement(visual, "geometry")
    element = ET.SubElement(geometry, shape)
    child(element, "radius" if shape == "sphere" else "size", size)
    material = ET.SubElement(visual, "material")
    child(material, "ambient", color)
    child(material, "diffuse", color)
    child(visual, "cast_shadows", "false")


def line(world, name, start, end, color, *, width=0.10, z=0.30):
    dx, dy = end[0] - start[0], end[1] - start[1]
    visual_model(
        world,
        name,
        (start[0] + end[0]) / 2.0,
        (start[1] + end[1]) / 2.0,
        z,
        math.atan2(dy, dx),
        "box",
        f"{math.hypot(dx, dy):.6f} {width} 0.04",
        color,
    )


def wamv_model(world, pose):
    model = ET.SubElement(world, "model", name="search_usv")
    child(model, "static", "true")
    child(model, "pose", f"{pose.x:.6f} {pose.y:.6f} 0.22 0 0 {pose.yaw:.6f}")
    link = ET.SubElement(model, "link", name="base_link")
    visual = ET.SubElement(link, "visual", name="wamv")
    mesh = ET.SubElement(ET.SubElement(visual, "geometry"), "mesh")
    child(mesh, "uri", WAMV_MESH)
    material = ET.SubElement(visual, "material")
    child(material, "ambient", "0.95 0.55 0.08 1")
    child(material, "diffuse", "1 0.65 0.12 1")


def build_scene():
    corners = (
        NEDPoint(-6.0, -12.0, 0.0),
        NEDPoint(24.0, -12.0, 0.0),
        NEDPoint(24.0, 12.0, 0.0),
        NEDPoint(-6.0, 12.0, 0.0),
    )
    plan = plan_geofence_search(
        corners,
        GeofenceSearchConfig(safety_inset_m=3.0, station_spacing_m=4.0),
    )

    sdf = ET.Element("sdf", version="1.9")
    world = ET.SubElement(sdf, "world", name=WORLD_NAME)
    child(world, "gravity", "0 0 -9.8")
    for filename, name in (
        ("physics", "Physics"),
        ("user-commands", "UserCommands"),
        ("scene-broadcaster", "SceneBroadcaster"),
    ):
        ET.SubElement(
            world,
            "plugin",
            filename=f"gz-sim-{filename}-system",
            name=f"gz::sim::systems::{name}",
        )
    scene = ET.SubElement(world, "scene")
    child(scene, "ambient", "0.72 0.76 0.82 1")
    child(scene, "background", "0.12 0.18 0.28 1")
    gui = ET.SubElement(world, "gui", fullscreen="false")
    view = ET.SubElement(gui, "plugin", filename="MinimalScene", name="3D View")
    child(view, "engine", "ogre2")
    child(view, "scene", "scene")
    child(view, "camera_pose", "9 -26 31 0 0.72 1.5708")
    for plugin in ("GzSceneManager", "InteractiveViewControl", "EntityTree"):
        ET.SubElement(gui, "plugin", filename=plugin, name=plugin)

    sun = ET.SubElement(world, "light", type="directional", name="sun")
    child(sun, "pose", "9 0 35 0 0 0")
    child(sun, "diffuse", "0.9 0.9 0.9 1")
    child(sun, "direction", "-0.3 0.2 -1")

    visual_model(world, "water", 9.0, 0.0, -0.13, 0.0, "box", "42 34 0.08", "0.02 0.18 0.32 0.92")
    boundary = plan.ordered_corners + (plan.ordered_corners[0],)
    for index, (first, second) in enumerate(zip(boundary, boundary[1:])):
        line(world, f"geofence_{index}", (first.north, first.east), (second.north, second.east), "0.9 0.9 0.9 1", width=0.12)

    colors = ("0.05 0.9 1 1", "0.2 0.35 1 0.8")
    for line_index, route in enumerate(plan.lines):
        for index, (first, second) in enumerate(zip(route.stations, route.stations[1:])):
            line(world, f"search_{line_index}_{index}", (first.x, first.y), (second.x, second.y), colors[line_index], width=0.16 if line_index == 0 else 0.10, z=0.38)
        for index, station in enumerate(route.stations):
            visual_model(world, f"station_{line_index}_{index}", station.x, station.y, 0.48, 0.0, "sphere", "0.20", colors[line_index])

    dock = ET.SubElement(world, "include")
    child(dock, "uri", "model://dock_2022_base")
    child(dock, "name", "three_bay_dock")
    child(dock, "pose", f"{DOCK_POSE[0]} {DOCK_POSE[1]} 0 0 0 {DOCK_POSE[2]}")
    for bay_index in (1, 2, 3):
        bay = vrx_dock_2022_bay_to_dockspec(*DOCK_POSE, bay_index)
        visual_model(world, f"detected_bay_{bay_index}", bay.entrance_center_x, bay.entrance_center_y, 0.72, 0.0, "sphere", "0.34", "0.1 1 0.25 1")
        dx, dy = math.cos(bay.entrance_heading), math.sin(bay.entrance_heading)
        line(world, f"bay_heading_{bay_index}", (bay.entrance_center_x, bay.entrance_center_y), (bay.entrance_center_x + 2.0 * dx, bay.entrance_center_y + 2.0 * dy), "1 0.85 0.05 1", width=0.09, z=0.78)

    wamv_model(world, plan.primary.stations[0])
    ET.indent(sdf, space="  ")
    metadata = {
        "world": WORLD_NAME,
        "notice": "Visualization of search geometry; not a validated hydrodynamic controller.",
        "primary_stations": [
            {"x": item.x, "y": item.y, "yaw": item.yaw}
            for item in plan.primary.stations
        ],
        "fallback_stations": [
            {"x": item.x, "y": item.y, "yaw": item.yaw}
            for item in plan.fallback.stations
        ],
        "dock_pose": {"x": DOCK_POSE[0], "y": DOCK_POSE[1], "yaw": DOCK_POSE[2]},
    }
    return sdf, metadata


def main():
    sdf, metadata = build_scene()
    output = ROOT / "generated"
    output.mkdir(exist_ok=True)
    ET.ElementTree(sdf).write(
        output / "task3_geofence_search.sdf",
        encoding="utf-8",
        xml_declaration=True,
    )
    (output / "task3_geofence_search.json").write_text(
        json.dumps(metadata, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    print(f"Generated {output / 'task3_geofence_search.sdf'}")


if __name__ == "__main__":
    main()
