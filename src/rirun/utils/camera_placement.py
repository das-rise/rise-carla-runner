"""
Placement checks for stationary (fixed world position) cameras.

On tiled Digital Twin Tool maps the CARLA server streams map tiles around
the active vehicles. An actor whose tile is not loaded is destroyed by the
server's LargeMapManager, and destroying a live sensor crashes the server
(SIGSEGV in ASensor::EndPlay) on the first tick. These helpers warn when a
stationary camera is placed far from the vehicle cloud.

Pure functions (no CARLA imports) so they can be tested headlessly.
"""

import math
from typing import Iterable, Optional, Tuple

# Warn when a stationary camera is farther than this (m) from the reference
# point (vehicle cloud centroid, or map center when nothing is spawned yet).
STATIONARY_CAMERA_MAX_DISTANCE_M = 2000.0


def point_centroid(
    points: Iterable[Tuple[float, float, float]],
) -> Optional[Tuple[float, float, float]]:
    """
    Centroid (mean) of an iterable of (x, y, z) points.

    Returns None when no points are given.
    """
    n = 0
    x = y = z = 0.0
    for p in points:
        x += p[0]
        y += p[1]
        z += p[2]
        n += 1
    if n == 0:
        return None
    return (x / n, y / n, z / n)


def euclidean_distance(
    a: Tuple[float, float, float], b: Tuple[float, float, float]
) -> float:
    """Euclidean distance between two (x, y, z) points."""
    return math.sqrt(sum((ai - bi) ** 2 for ai, bi in zip(a, b)))


def stationary_camera_warning(
    camera_position: Tuple[float, float, float],
    reference_point: Optional[Tuple[float, float, float]],
    reference_label: str,
    max_distance_m: float = STATIONARY_CAMERA_MAX_DISTANCE_M,
) -> Optional[str]:
    """
    Build a warning message for a stationary camera placed farther than
    max_distance_m from reference_point (vehicle cloud centroid or map
    center). Returns None when the placement is fine or there is no
    reference point to compare against.
    """
    if reference_point is None:
        return None
    dist = euclidean_distance(camera_position, reference_point)
    if dist <= max_distance_m:
        return None
    cx, cy, cz = reference_point
    return (
        f"Stationary camera at ({camera_position[0]:.0f}, {camera_position[1]:.0f}, "
        f"{camera_position[2]:.0f}) is {dist / 1000:.1f} km from the {reference_label} "
        f"({cx:.0f}, {cy:.0f}, {cz:.0f}). On tiled Digital Twin Tool maps this can "
        f"crash the CARLA server (SIGSEGV) on the first tick: the server's "
        f"LargeMapManager destroys actors whose map tile is not loaded, and "
        f"destroying a live sensor crashes it. Move the camera over the action, "
        f"e.g. --overhead_camera_position {cx:.0f} {cy:.0f} {camera_position[2]:.0f}."
    )
