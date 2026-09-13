"""V0.1 data contracts: metres, radians, one shared Cartesian frame."""

from dataclasses import dataclass, field
from enum import Enum
from typing import List, Optional


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class DockSpec:
    entrance_center_x: float
    entrance_center_y: float
    entrance_heading: float  # Points INTO the bay; not the dock model yaw.
    bay_width: float
    bay_depth: float
    confidence: Optional[float] = None
    dock_id: Optional[str] = None


@dataclass(frozen=True)
class PlannerConfig:
    approach_distance: float
    stop_distance_inside: float


class PlanStatus(str, Enum):
    SUCCESS = "SUCCESS"
    INVALID_INPUT = "INVALID_INPUT"
    UNSUPPORTED_INPUT = "UNSUPPORTED_INPUT"


@dataclass
class PlanResult:
    approach_pose: Optional[Pose2D]
    waypoints: List[Pose2D] = field(default_factory=list)
    status: PlanStatus = PlanStatus.INVALID_INPUT
    reason: str = ""
