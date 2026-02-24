import carla
from rirun.kinetics.movement import MovementPolicy, PIDMovement, TeleportMovement
from rirun.kinetics.trajectory import Trajectory
import logging
from typing import NamedTuple, Union, Optional
from rirun.kinetics.stats import Statistic
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
        deviation_statistics: Optional[Statistic] = None,
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
            deviation_statistics (Optional[Statistic], optional):
                An object to collect deviation metrics during the actor's run.
                Defaults to `None`.
        """

        self.name = name
        self._world = world
        blueprint_library = world.get_blueprint_library()
        self._blueprint = blueprint_library.filter(blueprint)[0]

        if isinstance(trajectory, Trajectory):
            assert (
                trajectory._has_headings and trajectory._has_speeds
            ), f"Trajectory needs headings and speeds; has headings [{trajectory._has_headings}], speeds [{trajectory._has_speeds}]"
            self._trajectory_generator = trajectory.get_carla_trajectory()
            self._current_trajectory_point = next(self._trajectory_generator)
            self.first_trajectory_point = next(self._trajectory_generator)
        else:
            self._xml_path = trajectory
            self.first_trajectory_point = get_first_xml_waypoint(self._xml_path)

        # Movement strategy
        if isinstance(movement, MovementPolicy):
            self._mover = movement
        elif isinstance(movement, str):
            self._mover = (
                PIDMovement() if movement.lower() == "pid" else TeleportMovement()
            )
        else:
            raise ValueError(
                f"Invalid movement policy type {type(movement)}. Must be MovementPolicy or str."
            )

        # Deviation statistic
        self._deviation_statistics = deviation_statistics

        self._spawned = False
        self._destroyed = False

    def step(self, simulation_time: float) -> None:
        """
        Execute one step of the simulation.
        Args:
            simulation_time (float): The current world time
        """
        self._mover.step(self, simulation_time, self._deviation_statistics)

    def spawn(self, transform: carla.Transform = None) -> None:
        """
        Spawn the vehicle at the location and rotation stored in `transform`
        Args:
            transform (carla.Transform): The location and rotation at which the vehicle should spawn.
        """
        if transform is None:
            transform = self.first_trajectory_point.transform
            transform.location.z = 0.1

        self._actor = self._world.spawn_actor(self._blueprint, transform)
        self._mover.on_spawn(self)
        logging.info(
            f"Spawned vehicle {self.name} at {transform} [{self._mover.__class__.__name__}]"
        )
        self._spawned = True

    def destroy(self) -> None:
        """
        Destroy the vehicle, throw AssertionError on failure
        """
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
        if self._spawned:
            current_position = self.get_actor().get_transform().location
            return current_position.z >= z_min
        else:
            return True  # If the vehicle is not spawned, we consider the z-coordinate to be valid by default.
