"""Export sampled Mid-360 PointCloud2 messages to ROS-independent NPY files.

This is an offline inspection tool.  It reads a rosbag2 directory and never
creates ROS nodes, publishes topics, or sends vehicle commands.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Optional, Sequence

from dock_planner import pointcloud2_message_to_xyz, summarize_xyz_quality


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path, help="rosbag2 directory")
    parser.add_argument("output", type=Path, help="output directory")
    parser.add_argument("--topic", default="/livox/lidar")
    parser.add_argument("--stride", type=int, default=10, help="keep every Nth cloud")
    parser.add_argument("--max-frames", type=int, default=20)
    return parser


def export_frames(
    bag: Path,
    output: Path,
    *,
    topic: str = "/livox/lidar",
    stride: int = 10,
    max_frames: int = 20,
) -> dict:
    if stride < 1 or max_frames < 1:
        raise ValueError("stride and max_frames must be positive")

    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as exc:
        raise RuntimeError("ROS 2 Python packages must be sourced to read a rosbag") from exc

    bag = bag.resolve()
    output.mkdir(parents=True, exist_ok=True)
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    if topic not in topic_types:
        raise ValueError(f"bag does not contain topic {topic!r}")

    message_type = get_message(topic_types[topic])
    seen = 0
    records = []
    while reader.has_next() and len(records) < max_frames:
        current_topic, serialized, recorded_stamp_ns = reader.read_next()
        if current_topic != topic:
            continue
        source_index = seen
        seen += 1
        if source_index % stride:
            continue
        message = deserialize_message(serialized, message_type)
        raw_xyz = pointcloud2_message_to_xyz(
            message, remove_nonfinite=False, remove_zero_points=False,
        )
        clean_xyz = pointcloud2_message_to_xyz(message)
        quality = summarize_xyz_quality(raw_xyz)
        filename = f"frame_{source_index:06d}.npy"

        # Import here so the command gives a useful ROS error before doing work.
        import numpy as np
        np.save(output / filename, clean_xyz, allow_pickle=False)
        records.append({
            "file": filename,
            "source_index": source_index,
            "recorded_stamp_ns": int(recorded_stamp_ns),
            "header_stamp": {
                "sec": int(message.header.stamp.sec),
                "nanosec": int(message.header.stamp.nanosec),
            },
            "frame_id": message.header.frame_id,
            "raw_points": int(quality.total_points),
            "clean_points": int(len(clean_xyz)),
            "nonfinite_points": int(quality.nonfinite_points),
            "zero_points": int(quality.zero_points),
        })

    manifest = {
        "source_bag": str(bag),
        "topic": topic,
        "message_type": topic_types[topic],
        "stride": stride,
        "frames": records,
    }
    (output / "manifest.json").write_text(
        json.dumps(manifest, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    return manifest


def main(argv: Optional[Sequence[str]] = None) -> None:
    args = _parser().parse_args(argv)
    result = export_frames(
        args.bag,
        args.output,
        topic=args.topic,
        stride=args.stride,
        max_frames=args.max_frames,
    )
    print(f"exported_frames={len(result['frames'])}")
    print(f"manifest={args.output.resolve() / 'manifest.json'}")


if __name__ == "__main__":
    main()
