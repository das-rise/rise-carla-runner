"""
route.py — Route model, XML I/O, and coordinate transforms for xodr-plot.

A route is an ordered list of waypoints in *CARLA* coordinates:

    x, y : metres in the CARLA world frame
    yaw  : degrees, CARLA convention (0 = +x, 90 = +y)
    z    : metres (kept for XML compatibility; planners work in 2D)

The plot shows the map from a top-down CARLA perspective, so plot
coordinates are the CARLA coordinates rotated 90°:

    plot_x = y_carla     (CARLA +y is "right" on the plot)
    plot_y = x_carla     (CARLA +x is "up"    on the plot)

and a heading of `yaw` degrees points along the plot-space angle
`90 - yaw` measured from the plot +x axis.
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from typing import List, Tuple


@dataclass
class Waypoint:
    """A single route waypoint in CARLA coordinates."""

    x: float
    y: float
    yaw: float
    z: float = 0.0


@dataclass
class Route:
    """An ordered list of waypoints (a PCLA agent route)."""

    waypoints: List[Waypoint] = field(default_factory=list)
    route_id: str = "_"
    town: str = "_"

    # ── Coordinate transforms ────────────────────────────────────────────────

    @staticmethod
    def carla_to_plot(x: float, y: float) -> Tuple[float, float]:
        """CARLA (x, y) → plot (x, y)."""
        return y, x

    @staticmethod
    def plot_to_carla(plot_x: float, plot_y: float) -> Tuple[float, float]:
        """Plot (x, y) → CARLA (x, y)."""
        return plot_y, plot_x

    @staticmethod
    def yaw_to_plot_angle(yaw_deg: float) -> float:
        """Heading (CARLA yaw, degrees) → arrow direction angle in plot space."""
        return 90.0 - yaw_deg

    @staticmethod
    def waypoint_to_plot(wp: Waypoint) -> Tuple[float, float]:
        return wp.y, wp.x

    # ── Editing helpers ──────────────────────────────────────────────────────

    def add(self, wp: Waypoint) -> int:
        self.waypoints.append(wp)
        return len(self.waypoints) - 1

    def remove(self, index: int) -> Waypoint:
        return self.waypoints.pop(index)

    def clear(self) -> None:
        self.waypoints.clear()

    def interpolated_waypoint(self, i: int, j: int) -> Waypoint:
        """Waypoint at the midpoint of waypoints i and j with an averaged yaw."""
        a, b = self.waypoints[i], self.waypoints[j]
        mx, my = 0.5 * (a.x + b.x), 0.5 * (a.y + b.y)
        # Average heading via the unit-vector sum so 170°/-170° → 180°, not 0°.
        vx = math.sin(math.radians(a.yaw)) + math.sin(math.radians(b.yaw))
        vy = math.cos(math.radians(a.yaw)) + math.cos(math.radians(b.yaw))
        yaw = math.degrees(math.atan2(vx, vy)) % 360.0
        return Waypoint(x=mx, y=my, yaw=yaw, z=0.5 * (a.z + b.z))

    # ── XML I/O ──────────────────────────────────────────────────────────────

    def to_xml(self) -> str:
        """Serialize to the PCLA agent-route XML format.

        The format is byte-identical to the one printed by the original
        interactive tool: pitch/roll/z are always written as "0.0" and
        x/y/yaw with three decimals.
        """
        lines = [
            '<?xml version="1.0" ?>',
            '<route id="{}" town="{}">'.format(self.route_id, self.town),
        ]
        for wp in self.waypoints:
            lines.append(
                '\t<waypoint pitch="0.0" roll="0.0" x="{}" y="{}" yaw="{}" z="0.0"/>'.format(
                    f"{wp.x:.3f}", f"{wp.y:.3f}", f"{wp.yaw:.3f}"
                )
            )
        lines.append("</route>")
        return "\n".join(lines)

    @classmethod
    def from_xml(cls, text: str) -> "Route":
        """Parse the PCLA agent-route XML format."""
        root = ET.fromstring(text)
        route_id = root.get("id", "_")
        town = root.get("town", "_")
        waypoints = []
        for elem in root.iter("waypoint"):
            waypoints.append(
                Waypoint(
                    x=float(elem.get("x", 0.0)),
                    y=float(elem.get("y", 0.0)),
                    yaw=float(elem.get("yaw", 0.0)),
                    z=float(elem.get("z", 0.0)),
                )
            )
        return cls(waypoints=waypoints, route_id=route_id, town=town)

    @classmethod
    def from_file(cls, path: str) -> "Route":
        with open(path, "r") as fh:
            return cls.from_xml(fh.read())
