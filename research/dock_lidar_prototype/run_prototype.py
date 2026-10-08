"""Run the synthetic offline detector and save a representative PNG."""

from pathlib import Path

from .detector import DockLidarConfig, detect_entrance
from .synthetic import SyntheticDockConfig, generate_synthetic_dock_point_cloud
from .visualize import render_detection_png


def main() -> None:
    root = Path(__file__).resolve().parent
    cloud = generate_synthetic_dock_point_cloud(SyntheticDockConfig(
        heading_rad=0.42,
        noise_std_m=0.025,
        dropout_probability=0.08,
        outlier_count=12,
        seed=23,
    ))
    config = DockLidarConfig()
    result = render_detection_png(
        cloud,
        (0.0, 0.0),
        root / "artifacts" / "synthetic_detection.png",
        config,
    )
    candidate = detect_entrance(cloud, (0.0, 0.0), config)
    print(f"filtered_points={len(result.filtered_xy)}")
    print(f"wall_segments={len(result.wall_segments)} candidates={len(result.candidates)}")
    print(f"selected={candidate}")
    print(f"visualization={root / 'artifacts' / 'synthetic_detection.png'}")


if __name__ == "__main__":
    main()
