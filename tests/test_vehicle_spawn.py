"""Integration tests for the new Vehicle code paths (no CARLA server needed).

Uses a fake world/blueprint library to verify:
- exact blueprint-id matching (filter() is regex-based, must not over-match)
- fallback to the default blueprint when the id is missing
- color application at spawn time (and no change when unsupported/unset)
- spawn z: road-surface based (waypoint), blueprint-aware half height,
  obstacle ladder anchored on the road, exact bounding-box ground fit

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


class _FakeBoxExtent:
    def __init__(self, z: float) -> None:
        self.z = z


class _FakeBoundingBox:
    def __init__(self, z: float) -> None:
        # carla.BoundingBox.extent is the HALF size; extent.z is used for
        # the exact ground fit in Vehicle.spawn.
        self.extent = _FakeBoxExtent(z)


class _FakeActor:
    def __init__(self, bb_half_height: float = 0.9) -> None:
        self.color = None
        self.destroyed = False
        self.bounding_box = _FakeBoundingBox(bb_half_height)
        self.final_transform = None

    def set_color(self, color) -> None:
        self.color = color

    def set_simulate_physics(self, enabled: bool) -> None:
        pass

    def set_transform(self, transform) -> None:
        self.final_transform = transform

    def destroy(self) -> bool:
        self.destroyed = True
        return True


class _FakeActorList:
    def filter(self, pattern: str) -> list:
        return []


class _FakeWaypoint:
    def __init__(self, z: float) -> None:
        self.transform = carla.Transform(
            carla.Location(0.0, 0.0, z), carla.Rotation(0.0, 0.0, 0.0)
        )


class _FakeMap:
    def __init__(self, road_z) -> None:
        self._road_z = road_z

    def get_waypoint(self, location, project_to_road=True):
        # None = no road at this point (off-road)
        return _FakeWaypoint(self._road_z) if self._road_z is not None else None


class _FakeWorld:
    def __init__(
        self,
        blueprint_ids,
        road_z: float = 0.0,
        fail_below_z: float = None,
        bb_half_height: float = 0.9,
    ) -> None:
        self._blueprints = [
            _FakeBlueprint(bid, color_attr=bid in COLORABLE) for bid in blueprint_ids
        ]
        self._map = _FakeMap(road_z)
        # try_spawn_actor fails (overlap) for all attempts below this z
        self._fail_below_z = fail_below_z
        self._bb_half_height = bb_half_height
        self.spawned = []
        self.attempts = []  # z of every try_spawn_actor attempt

    def get_blueprint_library(self) -> "_FakeBlueprintLibrary":
        return _FakeBlueprintLibrary(self._blueprints)

    def get_actors(self) -> _FakeActorList:
        return _FakeActorList()

    def get_map(self) -> _FakeMap:
        return self._map

    def try_spawn_actor(self, blueprint, transform):
        self.attempts.append(transform.location.z)
        if self._fail_below_z is not None and transform.location.z < self._fail_below_z:
            return None
        actor = _FakeActor(bb_half_height=self._bb_half_height)
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


def _make_vehicle(
    world: _FakeWorld, blueprint: str, color=None, height_m=None
) -> Vehicle:
    return Vehicle(
        world,
        _make_trajectory(),
        "test_vehicle",
        movement="teleport",
        blueprint=blueprint,
        color=color,
        height_m=height_m,
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


# ---------------------------------------------------------------------------
# Spawn z: road-surface based, blueprint-aware, clip-free
# ---------------------------------------------------------------------------


def test_spawn_z_is_road_z_plus_half_height() -> None:
    world = _FakeWorld(["vehicle.tesla.model3"], road_z=5.0)
    v = _make_vehicle(world, "vehicle.tesla.model3", height_m=1.8)
    v.spawn()
    # 5.0 (road) + 0.9 (half of 1.8) + 0.05 (ground margin)
    assert abs(world.attempts[0] - 5.95) < 1e-5


def test_spawn_default_half_height_for_unknown_blueprint() -> None:
    world = _FakeWorld(["model3"], road_z=2.0)
    v = _make_vehicle(world, "model3")  # no height_m -> 0.9 m default half height
    v.spawn()
    assert abs(world.attempts[0] - (2.0 + 0.9 + 0.05)) < 1e-5


def test_spawn_ladder_anchors_on_road_z() -> None:
    # overlap at the ground level: first attempt fails, second (lift +0.4) works
    world = _FakeWorld(["vehicle.tesla.model3"], road_z=3.0, fail_below_z=3.96)
    v = _make_vehicle(world, "vehicle.tesla.model3", height_m=1.8)
    v.spawn()
    assert v.is_spawned()
    assert len(world.attempts) == 2
    assert abs(world.attempts[0] - 3.95) < 1e-5  # road + half + margin
    assert abs(world.attempts[1] - 4.35) < 1e-5  # + 0.4 lift step


def test_spawn_ladder_gives_up_instead_of_floating_high() -> None:
    # blocked at every step (obstacle all the way up) -> no spawn, no 4 m float
    world = _FakeWorld(["vehicle.tesla.model3"], road_z=3.0, fail_below_z=99.0)
    v = _make_vehicle(world, "vehicle.tesla.model3", height_m=1.8)
    v.spawn()
    assert not v.is_spawned()
    assert len(world.attempts) == 4  # [0.0, 0.4, 0.8, 1.6] only


def test_spawn_ground_fit_from_bounding_box() -> None:
    # real collision box (half 1.0) differs from the table estimate (0.9):
    # the exact fit snaps the bottom onto the road surface
    world = _FakeWorld(["vehicle.tesla.model3"], road_z=2.0, bb_half_height=1.0)
    v = _make_vehicle(world, "vehicle.tesla.model3", height_m=1.8)
    v.spawn()
    actor = world.spawned[0][1]
    assert actor.final_transform is not None
    assert abs(actor.final_transform.location.z - 3.0) < 1e-5  # 2.0 + 1.0


def test_spawn_ground_fit_rejected_when_inconsistent() -> None:
    # bounding box wildly disagrees with the waypoint (far/wrong road):
    # do not yank the vehicle, keep the successful spawn z
    world = _FakeWorld(["vehicle.tesla.model3"], road_z=2.0, bb_half_height=5.0)
    v = _make_vehicle(world, "vehicle.tesla.model3", height_m=1.8)
    v.spawn()
    actor = world.spawned[0][1]
    assert actor.final_transform is None
    assert abs(world.attempts[0] - 2.95) < 1e-5


def test_spawn_no_waypoint_falls_back_to_location_z() -> None:
    # off-road point: no waypoint -> ground z = location z (trajectory z = 0)
    world = _FakeWorld(["vehicle.tesla.model3"], road_z=None)
    v = _make_vehicle(world, "vehicle.tesla.model3", height_m=1.8)
    v.spawn()
    assert abs(world.attempts[0] - (0.0 + 0.9 + 0.05)) < 1e-5


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
