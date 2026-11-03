import logging
from typing import NamedTuple, Optional, Union

import carla

from rirun.kinetics.actor import Actor
from rirun.kinetics.movement import (
    MovementPolicy,
    PIDMovement,
    PIDMovementTimestampAdvanced,
    TeleportMovement,
)
from rirun.kinetics.stats import Average_Distance_True
from rirun.kinetics.trajectory import Trajectory
from rirun.kinetics.trajectory_utils import get_first_xml_waypoint
from rirun.kinetics.vehicle_models import VehicleModel

#: Half overall height per blueprint id, from the VehicleModel table.
_BLUEPRINT_HALF_HEIGHTS = {m.blueprint: m.height / 2.0 for m in VehicleModel}

#: Fallback half height (m) for blueprints not in the VehicleModel table
#: (e.g. the DTT custom "model3"): a generic mid-size car.
DEFAULT_HALF_HEIGHT_M = 0.9


def half_height_for(blueprint_id: str) -> float:
    """Half overall height (m) for a blueprint id.

    Uses the VehicleModel table when the id is listed there, otherwise the
    generic mid-size-car default.
    """
    return _BLUEPRINT_HALF_HEIGHTS.get(blueprint_id, DEFAULT_HALF_HEIGHT_M)


class Vehicle(Actor):
    """
    A Vehicle has a Trajectory and takes steps to follow it.
    """

    def __init__(
        self,
        world: carla.World,
        trajectory: Union[Trajectory, str],
        name: str,
        movement: Union[str, MovementPolicy] = "pid",
        blueprint: str = "model3",
        deviation_statistics: Optional[str] = None,
        role_name: str = "npc",
        color: Optional[carla.Color] = None,
        height_m: Optional[float] = None,
    ) -> None:
        """
        Initialize an actor with a specified trajectory, movement policy, and vehicle blueprint.

        Args:
            world (carla.World):
                The CARLA simulation world in which the actor will be spawned.
            trajectory (Union[Trajectory, str]):
                The path the actor should follow.
                - If a `Trajectory` object, it must include headings and speeds, and
                is converted into a CARLA trajectory generator.
                - If a `str`, it is interpreted as the string of a path to a XML route.
            name (str):
                A unique name for this actor instance.
            movement (Union[str, MovementPolicy], optional):
                The movement policy for controlling the actor. Can be:
                - A `MovementPolicy` object.
                - A string specifying the policy: `"pid"` (default) for PID-based
                control, or any other string to fall back to teleportation-based movement.
            blueprint (str, optional):
                The vehicle blueprint to spawn, e.g., `"model3"`. Defaults to `"model3"`.
            deviation_statistics (Optional[str], optional):
                Which trajectory deviation statistics to compute during the run. Can be:
                - `"average_distance_true"`: computes the average distance between the reference and true position for all timestamps in a trajectory.
                Defaults to `None`.
            role_name (str, optional):
                The role name to set on the blueprint, e.g., `"npc"` or `"hero"`.
                Defaults to `"npc"`.
            color (Optional[carla.Color], optional):
                Color to apply to the vehicle after spawning (if the blueprint
                supports it). Defaults to `None` (no color change).
            height_m (Optional[float], optional):
                Overall height of the vehicle in meters, used to place the
                vehicle on the road when spawning (see `spawn`). Defaults to
                `None`, in which case a generic mid-size-car height (1.8 m)
                is assumed.
        """

        self.name = name
        self._world = world
        self._color = color
        # Half of the overall height: CARLA vehicle actors are placed at the
        # center of their bounding box, so the wheels sit ~half a height
        # below the actor location.
        self._half_height = (
            height_m / 2.0 if height_m is not None else DEFAULT_HALF_HEIGHT_M
        )
        blueprint_library = world.get_blueprint_library()
        # `filter` treats the argument as a regex and returns candidates;
        # require an exact type_id match and fall back to the default
        # blueprint if the requested one is not available in this world.
        # Note: on the Python side, ActorBlueprint.id is the type id string
        # (e.g. "vehicle.tesla.model3"), not an integer.
        self._blueprint = next(
            (bp for bp in blueprint_library.filter(blueprint) if bp.id == blueprint),
            None,
        )
        if self._blueprint is None:
            logging.warning(
                f"Vehicle blueprint '{blueprint}' not found in the world; "
                "falling back to default blueprint 'model3'."
            )
            self._blueprint = blueprint_library.filter("model3")[0]
        if self._blueprint.has_attribute("role_name"):
            self._blueprint.set_attribute("role_name", role_name)

        if isinstance(trajectory, Trajectory):
            assert trajectory._has_headings and trajectory._has_speeds, (
                f"Trajectory needs headings and speeds; has headings [{trajectory._has_headings}], speeds [{trajectory._has_speeds}]"
            )
            self._trajectory_generator = trajectory.get_carla_trajectory()
            self._current_trajectory_point = next(self._trajectory_generator)
            self.first_trajectory_point = next(self._trajectory_generator)
        else:
            self._xml_path = trajectory
            self.first_trajectory_point = get_first_xml_waypoint(self._xml_path)
            self._current_trajectory_point = self.first_trajectory_point

        # Movement strategy
        if isinstance(movement, MovementPolicy):
            self._mover = movement
        elif isinstance(movement, str):
            if movement.lower() == "pid":
                self._mover = PIDMovement()
            elif movement.lower() == "pid_ts":
                self._mover = PIDMovementTimestampAdvanced()
            elif movement.lower() == "teleport":
                self._mover = TeleportMovement()
            else:
                raise ValueError(
                    f"Invalid movement policy string: {movement}. Must be 'pid', 'pid_ts', or 'teleport'."
                )
        else:
            raise ValueError(
                f"Invalid movement policy type {type(movement)}. Must be MovementPolicy or str."
            )

        # Deviation statistic
        if deviation_statistics is None:
            self._deviation_statistics = None
        elif deviation_statistics == "average_distance_true":
            self._deviation_statistics = Average_Distance_True()
        else:
            raise ValueError(f"Invalid statistic name: {deviation_statistics}")

        self._spawned = False
        self._destroyed = False

    def step(self, simulation_time: float) -> None:
        """
        Execute one step of the simulation.
        Args:
            simulation_time (float): The current world time
        """
        if self._destroyed:
            return
        self._mover.step(self, simulation_time, self._deviation_statistics)

    # Small margin above the road surface so the spawn overlap test against
    # the ground mesh does not fire on centimeter-level penetration.
    _GROUND_MARGIN = 0.05
    # Lift steps above the road z when the exact spot is blocked by an
    # obstacle; beyond the last step the spot is considered unusable instead
    # of floating the vehicle high above the street.
    _Z_LADDERS = [0.0, 0.4, 0.8, 1.6]
    # The exact ground fit (bounding box) is only applied when it agrees with
    # the z that successfully spawned; a larger deviation indicates a wrong
    # or far-away waypoint and must not yank the vehicle.
    _GROUND_FIT_TOLERANCE = 0.75

    def _ground_z(self, location: carla.Location) -> float:
        """
        Road surface elevation at the (x, y) of `location`, from the map's
        lane tree (client-side query, no RPC). Falls back to the location's
        own z when no road is found (off-road spawn point).
        """
        wp = self._world.get_map().get_waypoint(
            carla.Location(location.x, location.y, location.z), project_to_road=True
        )
        return wp.transform.location.z if wp is not None else location.z

    def spawn(self, transform: carla.Transform = None) -> None:
        """
        Spawn the vehicle at the location and rotation stored in `transform`.

        The spawn height is derived from the road surface elevation at the
        spawn point plus the vehicle's half height (wheels on the street,
        no clipping, no floating). If the exact spot is blocked by an
        obstacle, the vehicle is lifted in small steps (max 1.6 m above the
        road); after a successful spawn, its z is refined to the exact
        ground using the spawned actor's own bounding box.

        Args:
            transform (carla.Transform): The location and rotation at which the vehicle should spawn.
                If `None`, the first trajectory point is used. The z component
                only serves as a hint for the road query / off-road fallback;
                the actual spawn height comes from the road surface.
        """
        if transform is None:
            loc = self.first_trajectory_point.transform.location
            rot = self.first_trajectory_point.transform.rotation
            transform = carla.Transform(carla.Location(loc.x, loc.y, loc.z), rot)

        ground_z = self._ground_z(transform.location)
        spawn_z = ground_z + self._half_height + self._GROUND_MARGIN

        # Safety pre-check: if another vehicle is already occupying the spawn area,
        # return without spawning so the movement policy retries on the next tick.
        _safe_radius = 8.0
        nearby = [
            a
            for a in self._world.get_actors().filter("*vehicle*")
            if a.get_location().distance(transform.location) < _safe_radius
        ]
        if nearby:
            logging.debug(
                f"{self.name}: spawn location occupied by vehicle {nearby[0].id} "
                f"({nearby[0].type_id}), delaying spawn."
            )
            return  # _spawned stays False; movement policy will retry next tick

        for z_delta in self._Z_LADDERS:
            attempt = carla.Transform(
                carla.Location(
                    transform.location.x,
                    transform.location.y,
                    spawn_z + z_delta,
                ),
                transform.rotation,
            )
            actor = self._world.try_spawn_actor(self._blueprint, attempt)
            if actor is not None:
                self._actor = actor
                if self._color is not None and self._blueprint.has_attribute("color"):
                    try:
                        actor.set_color(self._color)
                    except Exception as e:
                        logging.debug(f"{self.name}: could not set color: {e}")
                # Exact ground fit from the spawned actor's real collision
                # box (works for any blueprint, not just the VehicleModel
                # table): place the box bottom on the road surface.
                final_transform = attempt
                target_z = ground_z + actor.bounding_box.extent.z
                if abs(target_z - attempt.location.z) <= self._GROUND_FIT_TOLERANCE:
                    final_transform = carla.Transform(
                        carla.Location(
                            attempt.location.x, attempt.location.y, target_z
                        ),
                        attempt.rotation,
                    )
                    actor.set_transform(final_transform)
                self._mover.on_spawn(self)
                logging.info(
                    f"Spawned vehicle {self.name} [{self._blueprint.id}] "
                    f"at {final_transform} [{self._mover.__class__.__name__}]"
                )
                self._spawned = True
                return

        logging.error(
            f"Could not spawn vehicle {self.name} at {transform} (blocked up to "
            f"{self._Z_LADDERS[-1]} m above road z {ground_z:.2f}). Skipping."
        )
        self._destroyed = True  # mark as failed so it is excluded from active_vehicles
        return

    def destroy(self) -> None:
        """
        Destroy the vehicle, throw AssertionError on failure
        """
        if self._actor is None:
            return  # never successfully spawned
        assert self._actor.destroy(), f"Could not destroy Vehicle {self.name}."
        self._destroyed = True
        if self._deviation_statistics is not None:
            try:
                logging.info(
                    f"Deviation statistics for {self.name} with statistic {self._deviation_statistics.__class__.__name__}: {self._deviation_statistics.evaluate()}"
                )
            except Exception as e:
                logging.error(
                    f"Error evaluating deviation statistics for {self.name}: {e}"
                )

    def kick(self, vec=carla.Vector3D(10, 10, 0)) -> None:
        """
        Instantly set velocity of vehicle to a value.

        Args:
            vec (carla.Vector3D): The speed vector with unit km/h
        """
        self._actor.set_target_velocity(vec)

    def is_spawned(self) -> bool:
        """
        Getter for the _spawned attribute.

        Returns:
            bool: True if the vehicle has been spawned, False otherwise.
        """
        return self._spawned

    def get_actor(self) -> carla.Actor:
        """
        Getter for the _actor.

        Returns:
            carla.Actor: The carla.Actor of this Vehicle
        """
        return self._actor

    def advance_trajectory(self) -> None:
        self._current_trajectory_point = next(self._trajectory_generator)

    def get_current_trajectory_point(self) -> NamedTuple:
        return self._current_trajectory_point

    def get_xml_route(self) -> str:
        return self._xml_path

    def has_valid_z(self, z_min=-1) -> bool:
        """
        Check whether the z-coordinate of the vehicle's current trajectory point is valid .

        Returns:
            bool: True if the z-coordinate is valid, False otherwise.

        Args:
            z_min (float): Minimum valid z-coordinate value in meters. Defaults to -1.
        """

        if self._destroyed:
            return True  # If the vehicle is destroyed, we consider the z-coordinate to be valid by default.
        if not getattr(self._mover, "validate_z", True):
            return True  # Physics-enabled movement manages z itself; skip the check.
        if self._spawned:
            current_position = self.get_actor().get_transform().location
            return current_position.z >= z_min
        else:
            return True  # If the vehicle is not spawned, we consider the z-coordinate to be valid by default.


def spawn_behavior_npcs(
    world: carla.World,
    client,
    num: int,
    npc_behavior: str = "cautious",
    min_spacing: float = 15.0,
) -> list:
    """
    Spawn up to `num` roaming NPC vehicles driven by BehaviorMovement.

    Vehicles are placed at well-spaced map spawn points (at least `min_spacing`
    metres apart).  Each is created as a regular Vehicle with a BehaviorMovement
    policy so it integrates with the normal run.py step loop.

    Args:
        world: carla.World
        client: carla.Client  (passed through to BehaviorMovement)
        num: maximum number of NPCs to spawn
        npc_behavior: BehaviorAgent behavior string ("cautious"/"normal"/"aggressive")
        min_spacing: minimum distance (m) between chosen spawn points

    Returns:
        List of Vehicle instances (already stepped once to trigger spawning).
    """
    import random

    from rirun.kinetics.movement import BehaviorMovement

    spawn_points = world.get_map().get_spawn_points()
    random.shuffle(spawn_points)

    chosen = []
    for sp in spawn_points:
        if len(chosen) >= num:
            break
        if all(sp.location.distance(c.location) >= min_spacing for c in chosen):
            chosen.append(sp)

    if len(chosen) < num:
        logging.warning(
            f"spawn_behavior_npcs: could only find {len(chosen)} spawn points "
            f"with {min_spacing} m spacing for {num} requested NPCs."
        )

    npcs = []
    blueprint_library = world.get_blueprint_library()
    car_bps = blueprint_library.filter("vehicle.*")
    # exclude bikes / motorcycles for stability
    car_bps = [
        bp for bp in car_bps if int(bp.get_attribute("number_of_wheels").as_int()) == 4
    ]

    for i, sp in enumerate(chosen):
        bp = random.choice(car_bps)
        if bp.has_attribute("role_name"):
            bp.set_attribute("role_name", "npc")
        mover = BehaviorMovement(client, behavior=npc_behavior)
        # We need a dummy Trajectory-like first_trajectory_point; use spawn point
        from rirun.kinetics.trajectory import CarlaTrajectoryPoint

        dummy_traj_point = CarlaTrajectoryPoint(transform=sp, time=0.0, speed=0.0)

        v = Vehicle.__new__(Vehicle)
        v.name = f"behavior_npc_{i}"
        v._world = world
        v._blueprint = bp
        v.first_trajectory_point = dummy_traj_point
        v._mover = mover
        v._spawned = False
        v._destroyed = False
        v._deviation_statistics = None
        v._color = None
        v._half_height = half_height_for(bp.id)
        npcs.append(v)

    return npcs
