import carla
from rirun.kinetics.movement import (
    MovementPolicy,
    PIDMovement,
    PIDMovementTimestampAdvanced,
    TeleportMovement,
)
from rirun.kinetics.trajectory import Trajectory
import logging
from typing import NamedTuple, Union, Optional
from rirun.kinetics.stats import Average_Distance_True
from rirun.kinetics.trajectory_utils import get_first_xml_waypoint
from rirun.kinetics.actor import Actor


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
                - `"average_distance_interpolated"`: computes the average distance between the reference and interpolated position for all timestamps in a trajectory.
                - `"average_distance_true"`: computes the average distance between the reference and true position for all timestamps in a trajectory.
                Defaults to `None`.
        """

        self.name = name
        self._world = world
        blueprint_library = world.get_blueprint_library()
        self._blueprint = blueprint_library.filter(blueprint)[0]
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
        elif deviation_statistics == "average_distance_interpolated":
            self._deviation_statistics
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

    def spawn(self, transform: carla.Transform = None) -> None:
        """
        Spawn the vehicle at the location and rotation stored in `transform`
        Args:
            transform (carla.Transform): The location and rotation at which the vehicle should spawn.
        """
        if transform is None:
            loc = self.first_trajectory_point.transform.location
            rot = self.first_trajectory_point.transform.rotation
            transform = carla.Transform(carla.Location(loc.x, loc.y, 0.3), rot)

        # Safety pre-check: if another vehicle is already occupying the spawn area,
        # return without spawning so the movement policy retries on the next tick.
        _safe_radius = 8.0
        nearby = [
            a for a in self._world.get_actors().filter("*vehicle*")
            if a.get_location().distance(transform.location) < _safe_radius
        ]
        if nearby:
            logging.debug(
                f"{self.name}: spawn location occupied by vehicle {nearby[0].id} "
                f"({nearby[0].type_id}), delaying spawn."
            )
            return  # _spawned stays False; movement policy will retry next tick

        for z_delta in [0.0, 0.5, 1.0, 2.0, 4.0]:
            attempt = carla.Transform(
                carla.Location(transform.location.x, transform.location.y, transform.location.z + z_delta),
                transform.rotation,
            )
            actor = self._world.try_spawn_actor(self._blueprint, attempt)
            if actor is not None:
                self._actor = actor
                self._mover.on_spawn(self)
                logging.info(
                    f"Spawned vehicle {self.name} at {attempt} [{self._mover.__class__.__name__}]"
                )
                self._spawned = True
                return

        logging.error(
            f"Could not spawn vehicle {self.name} at {transform} (collision at all z offsets). Skipping."
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
        if not getattr(self._mover, 'validate_z', True):
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
    from rirun.kinetics.movement import BehaviorMovement
    import random

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
    car_bps = [bp for bp in car_bps if int(bp.get_attribute("number_of_wheels").as_int()) == 4]

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
        npcs.append(v)

    return npcs
