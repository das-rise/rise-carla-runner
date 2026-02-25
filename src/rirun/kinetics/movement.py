from rirun.kinetics.stats import Statistic
from rirun.kinetics.actor import Actor
from rirun.PCLA.PCLA_agents import PCLA_Agent
import math, sys
from typing import Optional
import logging
import carla
from pathlib import Path
from rirun.carla_agents.navigation.controller import VehiclePIDController
from rirun.PCLA.PCLA import PCLA

###


class MovementPolicy:
    """Strategy interface for moving vehicles along a trajectory."""

    def on_spawn(self, actor: Actor) -> None:
        """Called once the underlying CARLA actor is spawned."""
        return

    def step(
        self,
        actor: Actor,
        simulation_time: float,
        deviation_statistics: Optional[Statistic] = None,
    ) -> None:
        """Advance the actor along target_points (a Carla_Trajectory_Point)."""
        raise NotImplementedError


class PIDMovement(MovementPolicy):
    """Move using CARLA's VehiclePIDController aiming at the target waypoint."""

    MIN_DIST_TO_WAYPOINT = 2

    def __init__(
        self,
        lateral_dict: dict = None,
        longitudinal_dict: dict = None,
        max_throttle: float = 0.75,
        max_brake: float = 0.3,
        max_steering: float = 0.8,
        dt: float = 1.0 / 20.0,
    ) -> None:
        lateral_default = {"K_P": 1.95, "K_I": 0.05, "K_D": 0.2, "dt": dt}
        longitudinal_default = {"K_P": 1.0, "K_I": 0.05, "K_D": 0, "dt": dt}
        self.lateral = lateral_dict or lateral_default
        self.longitudinal = longitudinal_dict or longitudinal_default
        self.max_throttle = max_throttle
        self.max_brake = max_brake
        self.max_steering = max_steering
        self._controller = None

    def on_spawn(self, vehicle: Actor) -> None:
        self._controller = VehiclePIDController(
            vehicle.get_actor(),
            self.lateral,
            self.longitudinal,
            offset=0,
            max_throttle=self.max_throttle,
            max_brake=self.max_brake,
            max_steering=self.max_steering,
        )

    def step(
        self,
        actor: Actor,
        simulation_time: float,
        deviation_statistics: Optional[Statistic] = None,
    ) -> None:
        """
        Args:
            vehicle (Vehicle): The Vehicle to move.
            target_point: The target trajectory point to move towards.
            simulation_time: The current time in the simulation.

        Returns:
            bool: True if the vehicle has reached the target point and it can be cleared,
                  False otherwise.
        """

        raise NotImplementedError

        """
        What needs to be done here: Take care of the spawn logic, give vehicles an initial kick 
        to come to the target speed, and take care of destroying the vehicle
        """

        while True:
            dist_to_waypoint = (
                actor.get_actor()
                .get_location()
                .distance(actor.get_current_trajectory_point().transform.location)
            )
            if dist_to_waypoint < self.MIN_DIST_TO_WAYPOINT:
                try:
                    actor.advance_trajectory()
                except StopIteration:
                    logging.info(f"{actor.name} Reached end of trajectory")
                    return
            else:
                break

        control = self._controller.run_step(
            actor.get_current_trajectory_point().speed,
            actor.get_current_trajectory_point(),
        )
        actor.get_actor().apply_control(control)


class TeleportMovement(MovementPolicy):
    """Move by setting the actor's transform directly at each timestamp.
    Optionally sets target velocity to match the speed in m/s."""

    def __init__(self, temporal_trigger: float = 0.01):
        """
        Initialize the TeleportMovement policy.

        Args:
            temporal_trigger (float): The simulation time threshold used to determine
                how close the current simulation time needs to be to the first timestamp
                on the trajectory to start traversal.
        """

        self._traversed_trajectory = False
        self._temporal_trigger = temporal_trigger

    def step(
        self,
        actor: Actor,
        simulation_time=Optional[float],
        deviation_statistics: Optional[Statistic] = None,
    ) -> None:
        """Move the vehicle to the next trajectory point by teleporting it.

        This method updates the vehicle's transform and optionally its velocity.
        It uses the trajectory generator to get the next point and advances
        through the trajectory.

        Args:
            vehicle (Vehicle): The Vehicle to move.
            simulation_time: The current time in the simulation.

        Returns:
            None: This method modifies the vehicle's state directly.
        """

        curr_traj_time = actor.get_current_trajectory_point().time
        curr_traj_trafo = actor.get_current_trajectory_point().transform
        curr_traj_speed = actor.get_current_trajectory_point().speed

        if not actor.is_spawned():
            # check if it is time to spawn the actor
            if simulation_time >= curr_traj_time - self._temporal_trigger:
                logging.info(
                    f"{actor.name}: trying spawn at sim time {simulation_time}, traj time {curr_traj_time}, temporal trigger {self._temporal_trigger}"
                )
                actor.spawn()
                return

        elif self._traversed_trajectory:
            # return immediately if whole trajectory has been traversed
            return

        else:

            # Place vehicle exactly at the time-aligned trajectory point.
            actor.get_actor().set_transform(curr_traj_trafo)

            # Convert km/h to m/s and set a velocity vector aligned with yaw.
            # TODO: check whether this simly needs to convert to kmh!
            speed_ms = float(curr_traj_speed) / 3.6
            yaw_rad = math.radians(curr_traj_trafo.rotation.yaw)
            vx = math.cos(yaw_rad) * speed_ms
            vy = math.sin(yaw_rad) * speed_ms
            speed_vector = carla.Vector3D(vx, vy, 0)
            actor.get_actor().set_target_velocity(speed_vector)

            if deviation_statistics is not None:
                deviation_statistics.add(
                    (
                        curr_traj_time,
                        curr_traj_trafo.location,
                        speed_vector,
                        simulation_time,
                        curr_traj_trafo.location,
                    )
                )

            try:
                actor.advance_trajectory()
            except StopIteration:
                logging.info(f"{actor.name} Reached end of trajectory at simulation time {simulation_time}")
                actor.destroy()
                self._traversed_trajectory = True
                return


class PCLA_Movement(MovementPolicy):
    _START_TIME = 0

    def __init__(self, pcla_agent: PCLA_Agent, client: carla.Client) -> None:
        """
        Initialize the PCLA_Movement policy.

        Args:
            pcla_agent (PCLA_Agent): The PCLA agent responsible for decision making
            xml_route (str): Path to route agent must follow, path must be .xml file
            client (carla.Client): CARLA client instance
        """
        self._pcla_agent = pcla_agent
        self._client = client
        self._pcla = None
        self._throttle_exponent = 1.0

    def step(
        self,
        actor: Actor,
        simulation_time: Optional[float],
        deviation_statistics: Optional[Statistic] = None,
    ) -> None:
        """
        Execute one step of the movement policy.

        Args:
            actor (Actor): The actor to control
            simulation_time (Optional[float]): Current simulation time
            deviation_statistics (Optional[Statistic]): Statistics about deviation from route, currently ignored
        """
        if simulation_time < self._START_TIME:
            return

        elif not actor.is_spawned():
            actor.spawn()
            return

        elif self._pcla is None:
            self._pcla = PCLA(
                self._pcla_agent.value,
                actor.get_actor(),
                actor.get_xml_route(),
                self._client,
            )

        ego_action = self._pcla.get_action()
        ego_action.throttle = math.pow(ego_action.throttle, self._throttle_exponent)

        # logging.info(f"throttle: {ego_action.throttle}, brake: {ego_action.brake}")

        actor.get_actor().apply_control(ego_action)

        # TODO: find out that agent has reached end of trajectory and kill agent?

    def set_throttle_exponent(self, exponent: float) -> None:
        """
        Set the throttle exponent used to adjust throttle sensitivity.
        throttle -> throttle ^ exponent
        As throttle in [0;1], exponent < 1 increases sensitivity, exponent > 1 decreases sensitivity.

        Args:
            exponent (float): The exponent to set for throttle adjustment.
        """
        self._throttle_exponent = exponent
