"""
planner.py — Check an ordered list of waypoints against a CARLA road network.

Uses the same GlobalRoutePlanner the PCLA agents use at runtime (see
planner_grp.py / planner_dao.py, vendored from the CARLA leaderboard) to
trace a route between every pair of consecutive waypoints. This mirrors
`interpolate_trajectory()` in src/rirun/PCLA/leaderboard_codes/
route_manipulation.py: a route that passes this check is a route the agents
can drive.

Requires the `carla` + `networkx` dependencies (installed with xodr-plot on
Python 3.8–3.10).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable, List, Optional, Sequence, Tuple

import carla
import networkx as nx

from .planner_dao import GlobalRoutePlannerDAO
from .planner_grp import GlobalRoutePlanner

# Hop resolution used by PCLA's interpolate_trajectory() at runtime.
DEFAULT_HOP_RESOLUTION = 1.0

# Waypoints further than this from the nearest road are projected onto it by
# CARLA; flag them so the user knows the route starts elsewhere than clicked.
OFFROAD_THRESHOLD_M = 2.5

ProgressCallback = Optional[Callable[[str], None]]


@dataclass
class SegmentResult:
    """Result of tracing the route between two consecutive waypoints."""

    start_index: int  # 0-based index of the segment's first waypoint
    ok: bool
    reason: str = ""
    points: List[Tuple[float, float]] = field(
        default_factory=list
    )  # (x, y) in CARLA coords
    length_m: float = 0.0


@dataclass
class RouteTestResult:
    """Outcome of checking a full route (all segments + warnings)."""

    segments: List[SegmentResult]
    warnings: List[str] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return all(seg.ok for seg in self.segments)

    @property
    def total_length_m(self) -> float:
        return sum(seg.length_m for seg in self.segments)

    def summary_lines(self) -> List[str]:
        """Human-readable per-segment report (1-based waypoint numbering)."""
        lines = []
        for seg in self.segments:
            status = "OK" if seg.ok else "FAILED"
            detail = f"({seg.length_m:.1f} m)" if seg.ok else f"— {seg.reason}"
            lines.append(
                f"  [{seg.start_index + 1} -> {seg.start_index + 2}] {status}  {detail}"
            )
        for warning in self.warnings:
            lines.append(f"  warning: {warning}")
        lines.append(
            f"Route test: {'all segments routable' if self.ok else 'FAILED'} "
            f"({sum(1 for s in self.segments if s.ok)}/{len(self.segments)} segments, "
            f"{self.total_length_m:.1f} m)"
        )
        return lines


def check_route(
    wmap: carla.Map,
    locations: Sequence[carla.Location],
    hop_resolution: float = DEFAULT_HOP_RESOLUTION,
    offroad_threshold_m: float = OFFROAD_THRESHOLD_M,
    progress_cb: ProgressCallback = None,
) -> RouteTestResult:
    """Trace a route through the waypoints on the given CARLA map.

    Args:
        wmap: CARLA map of the loaded world.
        locations: Waypoint locations (CARLA coordinates), in route order.
        hop_resolution: Waypoint sampling distance for the planner (metres).
        offroad_threshold_m: Distance beyond which a waypoint is considered
            off-road (it gets projected onto the nearest road by CARLA).
        progress_cb: Optional callback receiving human-readable progress.

    Returns:
        RouteTestResult with one SegmentResult per consecutive pair plus
        per-waypoint off-road warnings.
    """
    if progress_cb is not None:
        progress_cb("Building route graph…")

    dao = GlobalRoutePlannerDAO(wmap, hop_resolution)
    grp = GlobalRoutePlanner(dao)
    grp.setup()

    # ── Off-road warnings ────────────────────────────────────────────────────
    warnings: List[str] = []
    for i, loc in enumerate(locations):
        projected = wmap.get_waypoint(loc)
        if projected is None:
            warnings.append(
                f"Waypoint {i + 1} has no road nearby (nothing to project onto)."
            )
            continue
        dist = projected.transform.location.distance(loc)
        if dist > offroad_threshold_m:
            warnings.append(
                f"Waypoint {i + 1} is {dist:.1f} m off the nearest road — "
                f"it will be projected onto the road."
            )

    # ── Per-segment trace ────────────────────────────────────────────────────
    segments: List[SegmentResult] = []
    num_segments = len(locations) - 1
    for i in range(num_segments):
        origin, destination = locations[i], locations[i + 1]
        if progress_cb is not None:
            progress_cb(f"Testing segment {i + 1}/{num_segments}…")
        try:
            trace = grp.trace_route(origin, destination)
        except nx.exception.NetworkXNoPath:
            segments.append(
                SegmentResult(
                    start_index=i,
                    ok=False,
                    reason="no path in the road network between the two waypoints",
                )
            )
            continue
        except Exception as exc:  # e.g. a waypoint that cannot be localized
            segments.append(
                SegmentResult(start_index=i, ok=False, reason=f"planner error: {exc}")
            )
            continue

        if not trace:
            segments.append(
                SegmentResult(
                    start_index=i, ok=False, reason="planner returned an empty route"
                )
            )
            continue

        points = [(wp.transform.location.x, wp.transform.location.y) for wp, _ in trace]
        length = sum(
            ((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2) ** 0.5
            for a, b in zip(points, points[1:])
        )
        segments.append(
            SegmentResult(start_index=i, ok=True, points=points, length_m=length)
        )

    return RouteTestResult(segments=segments, warnings=warnings)
