"""Create a Phase 3 test copy of the official world with extra run time."""

import argparse
import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SOURCE_CANDIDATES = (
    ROOT.parent / "src/vrx/vrx_gz/worlds/scan_dock_deliver_task.sdf",
    ROOT.parent / "install/vrx_gz/share/vrx_gz/worlds/scan_dock_deliver_task.sdf",
)
DEFAULT_OUTPUT = ROOT / "generated/scan_dock_deliver_phase3.sdf"
EXIT_DELAY_SECONDS = 120.0
RUNNING_STATE_SECONDS = 480.0


def source_world() -> Path:
    for candidate in SOURCE_CANDIDATES:
        if candidate.exists():
            return candidate
    raise FileNotFoundError("official scan_dock_deliver_task.sdf was not found")


def build_phase3_world(path: Path | None = None) -> ET.ElementTree:
    tree = ET.parse(path or source_world())
    root = tree.getroot()
    world = root.find("./world")
    if world is None or world.get("name") != "scan_dock_deliver_task":
        raise ValueError("unexpected official world name")
    plugins = world.findall("./plugin[@name='vrx::ScanDockScoringPlugin']")
    if len(plugins) != 1:
        raise ValueError("expected exactly one ScanDockScoringPlugin")
    plugin = plugins[0]
    exit_delay = plugin.find("exit_delay")
    if exit_delay is None:
        exit_delay = ET.SubElement(plugin, "exit_delay")
    exit_delay.text = str(EXIT_DELAY_SECONDS)
    running_state_duration = plugin.find("running_state_duration")
    if running_state_duration is None:
        raise ValueError("official world has no running_state_duration")
    running_state_duration.text = str(RUNNING_STATE_SECONDS)
    ET.indent(tree, space="  ")
    return tree


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    tree = build_phase3_world()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    tree.write(args.output, encoding="utf-8", xml_declaration=True)
    print(f"Generated {args.output}")
    print(
        "world=scan_dock_deliver_task; scoring lifecycle changed to "
        f"running_state_duration={RUNNING_STATE_SECONDS:.0f} s, "
        f"exit_delay={EXIT_DELAY_SECONDS:.0f} s"
    )


if __name__ == "__main__":
    main()
