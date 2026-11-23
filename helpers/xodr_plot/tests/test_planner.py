"""Unit tests for xodr_plot.planner.check_route with a stubbed GRP."""

import math

import pytest

pytest.importorskip("carla")
pytest.importorskip("networkx")

import networkx as nx  # noqa: E402
from xodr_plot import planner  # noqa: E402

# ── Fakes ─────────────────────────────────────────────────────────────────────


class _FakeLocation:
    def __init__(self, x, y, z=0.0):
        self.x, self.y, self.z = x, y, z

    def distance(self, other):
        return float(
            math.sqrt(
                (self.x - other.x) ** 2
                + (self.y - other.y) ** 2
                + (self.z - other.z) ** 2
            )
        )


class _FakeWaypoint:
    """carla.Waypoint stand-in: .transform.location only."""

    def __init__(self, loc):
        self.transform = type("T", (), {"location": loc})()


class _FakeMap:
    """carla.Map stand-in. get_waypoint projects onto the x-axis (y=0)."""

    def __init__(self, offroad_points=()):
        self._offroad = set(offroad_points)  # (x, y) keys

    def get_waypoint(self, loc):
        if (loc.x, loc.y) in self._offroad:
            return _FakeWaypoint(_FakeLocation(loc.x, 20.0))  # 20 m off the road
        return _FakeWaypoint(_FakeLocation(loc.x, 0.0))  # on the road


class _FakeGRPC:
    """GlobalRoutePlanner stand-in: per-origin behaviour is scripted."""

    def __init__(self, dao):
        self.behaviour = {}  # origin_x -> "ok" | "nopath" | "empty" | Exception

    def setup(self):
        pass

    def trace_route(self, origin, destination):
        behaviour = self.behaviour.get(origin.x, "ok")
        if isinstance(behaviour, Exception):
            raise behaviour
        if behaviour == "nopath":
            raise nx.exception.NetworkXNoPath("no path")
        if behaviour == "empty":
            return []
        return [
            (_FakeWaypoint(_FakeLocation(origin.x, 0.0)), None),
            (_FakeWaypoint(_FakeLocation(origin.x + 1.0, 0.0)), None),
            (_FakeWaypoint(_FakeLocation(destination.x, 0.0)), None),
        ]


class _FakeDAO:
    def __init__(self, wmap, resolution):
        pass


def _make_grpc(monkeypatch, behaviour):
    fake = _FakeGRPC(None)
    fake.behaviour = behaviour
    monkeypatch.setattr(planner, "GlobalRoutePlannerDAO", _FakeDAO)
    monkeypatch.setattr(planner, "GlobalRoutePlanner", lambda dao: fake)


def _locs(*xs):
    import carla

    return [carla.Location(x=x, y=0.0, z=0.0) for x in xs]


# ── Tests ─────────────────────────────────────────────────────────────────────


def test_all_segments_ok(monkeypatch):
    _make_grpc(monkeypatch, {})
    result = planner.check_route(_FakeMap(), _locs(0.0, 10.0, 20.0))
    assert result.ok
    assert len(result.segments) == 2
    assert all(seg.ok for seg in result.segments)
    assert result.total_length_m > 0
    # dense points returned in CARLA coords for plotting
    assert result.segments[0].points[0] == (0.0, 0.0)
    assert result.segments[0].points[-1] == (10.0, 0.0)


def test_no_path_segment_fails_others_pass(monkeypatch):
    _make_grpc(monkeypatch, {10.0: "nopath"})
    result = planner.check_route(_FakeMap(), _locs(0.0, 10.0, 20.0))
    assert not result.ok
    assert result.segments[0].ok
    assert not result.segments[1].ok
    assert "no path in the road network" in result.segments[1].reason
    assert result.segments[1].points == []
    assert result.segments[1].length_m == 0.0


def test_empty_trace_counts_as_failure(monkeypatch):
    _make_grpc(monkeypatch, {0.0: "empty"})
    result = planner.check_route(_FakeMap(), _locs(0.0, 10.0))
    assert not result.ok
    assert "empty" in result.segments[0].reason


def test_unhandled_planner_exception_is_reported(monkeypatch):
    _make_grpc(monkeypatch, {0.0: ValueError("boom")})
    result = planner.check_route(_FakeMap(), _locs(0.0, 10.0))
    assert not result.ok
    assert "planner error: boom" in result.segments[0].reason


def test_offroad_warning(monkeypatch):
    _make_grpc(monkeypatch, {})
    wmap = _FakeMap(offroad_points=[(5.0, 0.0)])  # waypoint 2 is 20 m off-road
    result = planner.check_route(wmap, _locs(0.0, 5.0))
    assert result.ok
    assert len(result.warnings) == 1
    assert "Waypoint 2" in result.warnings[0]
    assert "20.0 m off the nearest road" in result.warnings[0]


def test_summary_lines(monkeypatch):
    _make_grpc(monkeypatch, {10.0: "nopath"})
    result = planner.check_route(_FakeMap(), _locs(0.0, 10.0, 20.0))
    lines = result.summary_lines()
    assert lines[0].startswith("  [1 -> 2] OK")
    assert lines[1].startswith("  [2 -> 3] FAILED")
    assert "Route test: FAILED" in lines[-1]


def test_progress_callbacks(monkeypatch):
    _make_grpc(monkeypatch, {})
    messages = []
    planner.check_route(_FakeMap(), _locs(0.0, 10.0), progress_cb=messages.append)
    assert any("Building route graph" in m for m in messages)
    assert any("Testing segment 1/1" in m for m in messages)
