"""Integration tests for the new Vehicle code paths (no CARLA server needed).

Uses a fake world/blueprint library to verify:
- exact blueprint-id matching (filter() is regex-based, must not over-match)
- fallback to the default blueprint when the id is missing
- color application at spawn time (and no change when unsupported/unset)

Run with pytest, or standalone:  python tests/test_vehicle_spawn.py
"""

import logging
import os
import re
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

import carla  # noqa: E402

from rirun.kinetics.trajectory import Trajectory  # noqa: E402
from rirun.kinetics.vehicle_tools import Vehicle  # noqa: E402

logging.basicConfig(level=logging.DEBUG)


class _FakeActor:
    def __init__(self) -> None:
        self.color = None
        self.destroyed = False

    def set_color(self, color) -> None:
        self.color = color

    def set_simulate_physics(self, enabled: bool) -> None:
        pass

    def destroy(self) -> bool:
        self.destroyed = True
        return True


class _FakeActorList:
    def filter(self, pattern: str) -> list:
        return []


class _FakeWorld:
    def __init__(self, blueprint_ids) -> None:
        self._blueprints = [
            _FakeBlueprint(bid, color_attr=bid in COLORABLE) for bid in blueprint_ids
        ]
        self.spawned = []

    def get_blueprint_library(self) -> "_FakeBlueprintLibrary":
        return _FakeBlueprintLibrary(self._blueprints)

    def get_actors(self) -> _FakeActorList:
        return _FakeActorList()

    def try_spawn_actor(self, blueprint, transform):
        actor = _FakeActor()
        self.spawned.append((blueprint, actor))
        return actor


class _FakeBlueprint:
    # Mirrors the carla Python API: ActorBlueprint.id is the type id string
    # (e.g. "vehicle.tesla.model3"), not an integer.
    def __init__(self, id: str, color_attr: bool = True) -> None:
        self.id = id
        self._has_color = color_attr
        self.role_name = None

    def has_attribute(self, name: str) -> bool:
        return name == "role_name" or (name == "color" and self._has_color)

    def set_attribute(self, name: str, value) -> None:
        if name == "role_name":
            self.role_name = value


class _FakeBlueprintLibrary:
    def __init__(self, blueprints) -> None:
        self._blueprints = blueprints

    def filter(self, pattern: str) -> list:
        # same semantics as carla: regex search over the blueprint id
        return [bp for bp in self._blueprints if re.search(pattern, bp.id)]


#: Blueprints whose fake asset supports set_color.
COLORABLE = {"vehicle.tesla.model3", "model3", "vehicle.audi.etron"}


def _make_trajectory() -> Trajectory:
    # (x, y, t, heading, speed) x3 -> has headings and speeds
    return Trajectory(
        [
            (0.0, 0.0, 0.0, 0.0, 10.0),
            (1.0, 0.0, 1.0, 0.0, 10.0),
            (2.0, 0.0, 2.0, 0.0, 10.0),
        ]
    )


def _make_vehicle(world: _FakeWorld, blueprint: str, color=None) -> Vehicle:
    return Vehicle(
        world,
        _make_trajectory(),
        "test_vehicle",
        movement="teleport",
        blueprint=blueprint,
        color=color,
    )


def test_exact_blueprint_match_beats_regex_overmatch() -> None:
    world = _FakeWorld(["model3", "vehicle.tesla.model3", "vehicle.teslaXmodel3y"])
    v = _make_vehicle(world, "vehicle.tesla.model3")
    assert v._blueprint.id == "vehicle.tesla.model3"


def test_fallback_to_default_when_missing() -> None:
    world = _FakeWorld(["model3", "vehicle.tesla.model3"])
    v = _make_vehicle(world, "vehicle.does.not.exist")
    assert v._blueprint.id == "model3"


def test_color_applied_at_spawn() -> None:
    world = _FakeWorld(["vehicle.tesla.model3"])
    v = _make_vehicle(world, "vehicle.tesla.model3", color=carla.Color(10, 20, 30))
    v.spawn()
    assert v.is_spawned()
    assert len(world.spawned) == 1
    assert world.spawned[0][1].color == carla.Color(10, 20, 30)


def test_color_skipped_when_unsupported() -> None:
    world = _FakeWorld(["vehicle.unsupported.car"])
    v = _make_vehicle(world, "vehicle.unsupported.car", color=carla.Color(10, 20, 30))
    v.spawn()
    assert v.is_spawned()
    assert world.spawned[0][1].color is None  # set_color never called


def test_no_color_by_default() -> None:
    world = _FakeWorld(["model3"])
    v = _make_vehicle(world, "model3")
    v.spawn()
    assert v.is_spawned()
    assert world.spawned[0][1].color is None


def test_role_name_still_set() -> None:
    world = _FakeWorld(["vehicle.audi.etron"])
    v = Vehicle(
        world,
        _make_trajectory(),
        "ego_agent",
        movement="teleport",
        blueprint="vehicle.audi.etron",
        role_name="hero",
        color=carla.Color(255, 0, 0),
    )
    assert v._blueprint.role_name == "hero"
    v.spawn()
    assert world.spawned[0][1].color == carla.Color(255, 0, 0)


if __name__ == "__main__":
    import traceback

    failures = 0
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            try:
                fn()
                print(f"PASS {name}")
            except Exception:
                failures += 1
                print(f"FAIL {name}")
                traceback.print_exc()
    print(f"\n{'all tests passed' if failures == 0 else f'{failures} test(s) failed'}")
    raise SystemExit(1 if failures else 0)
