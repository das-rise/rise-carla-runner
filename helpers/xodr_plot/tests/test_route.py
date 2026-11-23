"""Unit tests for xodr_plot.route (pure stdlib — no carla needed)."""

import math
from pathlib import Path

from xodr_plot.route import Route, Waypoint

REPO_ROOT = Path(__file__).resolve().parents[3]


# ── Transforms ────────────────────────────────────────────────────────────────


def test_carla_plot_roundtrip():
    x, y = Route.plot_to_carla(*Route.carla_to_plot(123.4, -56.7))
    assert (x, y) == (123.4, -56.7)


def test_carla_to_plot_is_90deg_rotation():
    # CARLA +x is "up" (plot +y), CARLA +y is "right" (plot +x)
    assert Route.carla_to_plot(10.0, 0.0) == (0.0, 10.0)
    assert Route.carla_to_plot(0.0, 10.0) == (10.0, 0.0)


def test_yaw_to_plot_angle():
    assert Route.yaw_to_plot_angle(0) == 90.0  # CARLA +x → plot up
    assert Route.yaw_to_plot_angle(90) == 0.0  # CARLA +y → plot right
    assert Route.yaw_to_plot_angle(180) == -90.0
    assert Route.yaw_to_plot_angle(-90) == 180.0


# ── XML I/O ───────────────────────────────────────────────────────────────────


def test_xml_format_matches_original_tool():
    route = Route(
        waypoints=[
            Waypoint(x=154.0238037109375, y=46.19753646850586, yaw=90.06959533691406)
        ]
    )
    expected = (
        '<?xml version="1.0" ?>\n'
        '<route id="_" town="_">\n'
        '\t<waypoint pitch="0.0" roll="0.0" x="154.024" y="46.198" yaw="90.070" z="0.0"/>\n'
        "</route>"
    )
    assert route.to_xml() == expected


def test_xml_roundtrip():
    route = Route(
        waypoints=[
            Waypoint(x=1.5, y=-2.25, yaw=45.0),
            Waypoint(x=10.0, y=20.0, yaw=-135.5),
        ]
    )
    restored = Route.from_xml(route.to_xml())
    assert len(restored.waypoints) == 2
    for original, wp in zip(route.waypoints, restored.waypoints):
        assert abs(wp.x - original.x) < 5e-4
        assert abs(wp.y - original.y) < 5e-4
        assert abs(wp.yaw - original.yaw) < 5e-4
        assert wp.z == 0.0


def test_load_existing_repo_route():
    route = Route.from_file(REPO_ROOT / "scenes/town01/agent_routes/agent_turn.xml")
    assert len(route.waypoints) == 2
    assert abs(route.waypoints[0].x - 154.0238037109375) < 1e-6
    assert abs(route.waypoints[1].yaw - (-12.572527885437012)) < 1e-6


def test_from_xml_reads_z_and_attributes():
    text = (
        '<?xml version="1.0" ?>\n'
        '<route id="42" town="Town01">\n'
        '\t<waypoint pitch="0.0" roll="0.0" x="1.0" y="2.0" yaw="3.0" z="4.5"/>\n'
        "</route>"
    )
    route = Route.from_xml(text)
    assert route.route_id == "42"
    assert route.town == "Town01"
    assert route.waypoints[0].z == 4.5
    # to_xml always writes z="0.0" (original tool behaviour)
    assert 'z="0.0"' in route.to_xml()


# ── Editing helpers ───────────────────────────────────────────────────────────


def test_add_remove_clear():
    route = Route()
    idx = route.add(Waypoint(x=1, y=2, yaw=0))
    assert idx == 0 and len(route.waypoints) == 1
    removed = route.remove(0)
    assert removed.yaw == 0 and route.waypoints == []
    route.add(Waypoint(x=0, y=0, yaw=0))
    route.clear()
    assert route.waypoints == []


def test_interpolated_waypoint_midpoint():
    route = Route(waypoints=[Waypoint(x=0, y=0, yaw=0), Waypoint(x=10, y=0, yaw=90)])
    wp = route.interpolated_waypoint(0, 1)
    assert (wp.x, wp.y) == (5.0, 0.0)
    # 0° and 90° average to 45°
    assert math.isclose(wp.yaw, 45.0, abs_tol=1e-9)


def test_interpolated_waypoint_yaw_wraparound():
    # 170° and -170° average to 180° (not 0°)
    route = Route(
        waypoints=[Waypoint(x=0, y=0, yaw=170), Waypoint(x=10, y=0, yaw=-170)]
    )
    wp = route.interpolated_waypoint(0, 1)
    assert math.isclose(wp.yaw % 360.0, 180.0, abs_tol=1e-6)
