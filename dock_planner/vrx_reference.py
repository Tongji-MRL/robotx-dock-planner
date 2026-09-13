"""VRX_REFERENCE_GEOMETRY: dock_2022_base, not RobotX 2026 official geometry.

Source: vrx_gz/models/dock_2022_base/model.sdf and
vrx_gz/models/dock_block_4x4/model.sdf (VRX humble, dc30ed8d).
Each block collision is 2 x 2 m. Entrance is the finger-tip line x=-1;
the back wall inner face is x=5. No runtime dependency on VRX files.
"""

import math
from numbers import Integral, Real

from .types import DockSpec


VRX_REFERENCE_GEOMETRY = "VRX_REFERENCE_GEOMETRY"
_ENTRANCE_Y = (3.0, 9.0, 15.0)
_LOCAL_INWARD_HEADING = 0.0
_BAY_WIDTH = 4.0
_BAY_DEPTH = 6.0


def vrx_dock_2022_bay_to_dockspec(
    dock_world_x: float,
    dock_world_y: float,
    dock_world_yaw: float,
    bay_index: int,
) -> DockSpec:
    """Convert bay_index 1, 2 or 3 to a world-frame DockSpec.

    Coordinates are metres; yaw is radians, CCW from +X. Output heading is
    wrapped to [-pi, pi). Raises ValueError for invalid inputs. The generated
    dock_id identifies a bay within one dock, not a globally unique instance.
    """
    for name, value in (("dock_world_x", dock_world_x),
                        ("dock_world_y", dock_world_y),
                        ("dock_world_yaw", dock_world_yaw)):
        if (isinstance(value, bool) or not isinstance(value, Real)
                or not math.isfinite(value)):
            raise ValueError(f"{name} must be a finite real number")
    if isinstance(bay_index, bool) or not isinstance(bay_index, Integral) or bay_index not in (1, 2, 3):
        raise ValueError("bay_index must be 1, 2 or 3")

    heading = (dock_world_yaw + _LOCAL_INWARD_HEADING + math.pi) % (2 * math.pi) - math.pi
    c, s = math.cos(dock_world_yaw), math.sin(dock_world_yaw)
    local_y = _ENTRANCE_Y[bay_index - 1]
    return DockSpec(
        entrance_center_x=dock_world_x - c - local_y * s,
        entrance_center_y=dock_world_y - s + local_y * c,
        entrance_heading=heading,
        bay_width=_BAY_WIDTH,
        bay_depth=_BAY_DEPTH,
        dock_id=f"bay_{bay_index}",
    )
