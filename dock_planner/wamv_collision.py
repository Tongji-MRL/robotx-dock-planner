"""Read the personal VRX WAM-V URDF collision primitives for static checks."""

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
URDF_CANDIDATES = (
    ROOT.parent / "build/wamv_description/urdf/wamv_base.urdf",
    ROOT.parent / "src/vrx/vrx_urdf/wamv_description/urdf/wamv_base.urdf",
)


@dataclass(frozen=True)
class CollisionPrimitive:
    name: str
    kind: str
    xyz: tuple[float, float, float]
    rpy: tuple[float, float, float]
    dimensions: tuple[float, ...]


def urdf_path(path: Path | None = None) -> Path:
    if path is not None:
        return path
    for candidate in URDF_CANDIDATES:
        if candidate.exists():
            return candidate
    raise FileNotFoundError("personal VRX WAM-V base URDF was not found")


def load_collision_primitives(path: Path | None = None) -> list[CollisionPrimitive]:
    root = ET.parse(urdf_path(path)).getroot()
    out = []
    for collision in root.findall("./link/collision"):
        origin = collision.find("origin")
        xyz = tuple(float(v) for v in (origin.get("xyz", "0 0 0").split() if origin is not None else (0, 0, 0)))
        rpy = tuple(float(v) for v in (origin.get("rpy", "0 0 0").split() if origin is not None else (0, 0, 0)))
        geometry = collision.find("geometry")
        if geometry is None or len(geometry) != 1:
            raise ValueError(f"unsupported collision geometry: {collision.get('name')}")
        shape = geometry[0]
        if shape.tag == "box":
            dims = tuple(float(v) for v in shape.get("size").split())
        elif shape.tag == "cylinder":
            dims = (float(shape.get("radius")), float(shape.get("length")))
        else:
            raise ValueError(f"unsupported WAM-V collision shape: {shape.tag}")
        out.append(CollisionPrimitive(collision.get("name", "collision"), shape.tag, xyz, rpy, dims))
    if not out:
        raise ValueError("WAM-V URDF has no collision primitives")
    return out


def _rotation(rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return ((cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
            (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
            (-sp, cp * sr, cp * cr))


def footprint_bounds(primitives: list[CollisionPrimitive] | None = None):
    """Conservative exact XY projection bounds of all URDF primitives."""
    primitives = primitives or load_collision_primitives()
    xmin = ymin = float("inf")
    xmax = ymax = float("-inf")
    for p in primitives:
        R = _rotation(p.rpy)
        if p.kind == "box":
            half = [d / 2 for d in p.dimensions]
            ex = sum(abs(R[0][j]) * half[j] for j in range(3))
            ey = sum(abs(R[1][j]) * half[j] for j in range(3))
        else:
            radius, length = p.dimensions
            # Cylinder axis is local Z; this is the exact horizontal projection radius.
            ex = abs(R[0][2]) * length / 2 + radius * math.sqrt(max(0, 1 - R[0][2] ** 2))
            ey = abs(R[1][2]) * length / 2 + radius * math.sqrt(max(0, 1 - R[1][2] ** 2))
        xmin, xmax = min(xmin, p.xyz[0] - ex), max(xmax, p.xyz[0] + ex)
        ymin, ymax = min(ymin, p.xyz[1] - ey), max(ymax, p.xyz[1] + ey)
    return xmin, xmax, ymin, ymax
