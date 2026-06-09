import logging
import math
import random as _random_module
import xml.etree.ElementTree as ET
from typing import Optional

import carla

from rirun.carla_agents.navigation.behavior_agent import BehaviorAgent
from rirun.carla_agents.navigation.controller import VehiclePIDController
from rirun.kinetics.actor import Actor
from rirun.kinetics.stats import Average_Distance_True, Statistic
from rirun.kinetics.trajectory import CarlaTrajectoryPoint
from rirun.PCLA.PCLA import PCLA
from rirun.PCLA.PCLA_agents import PCLA_Agent
from rirun.utils.carla_tools import OpenDriveSpeedProvider

###


class MovementPolicy:
    """Strategy interface for moving vehicles along a trajectory."""

    #: Set to False in physics-enabled policies (e.g. BehaviorMovement) to skip
    #: the z-coordinate sanity check that detects teleport vehicles falling off the map.
    validate_z: bool = True

    def on_spawn(self, actor: Actor) -> None:
        """Called once the underlying CARLA actor is spawned."""
        return

    def _validate_statistics(self, deviation_statistics: Optional[Statistic]) -> None:
        if deviation_statistics is not None and not isinstance(
            deviation_statistics, Average_Distance_True
        ):
            raise Exception(
                "Only Average_Distance_True statistic is currently supported. "
                f"Provided statistic: {deviation_statistics.__class__.__name__}"
            )

    def _pass_point_to_stats(
        self,
        point: CarlaTrajectoryPoint,
        deviation_statistics: Optional[Statistic],
    ) -> None:
        if deviation_statistics is not None:
            deviation_statistics.add(point)

    def _advance_or_finish(
        self,
        actor: Actor,
        simulation_time: float,
        deviation_statistics: Optional[Statistic],
    ) -> Optional[CarlaTrajectoryPoint]:
        """Advance the trajectory by one step.

        Returns the new current trajectory point, or None if the trajectory
        has been fully traversed (actor is destroyed and flag is set).
        """
        try:
            actor.advance_trajectory()
            point = actor.get_current_trajectory_point()
            self._pass_point_to_stats(point, deviation_statistics)
            return point
        except StopIteration:
            logging.info(
                f"{actor.name}: reached end of trajectory at sim time {simulation_time}"
            )
            actor.destroy()
            self._traversed_trajectory = True
            return None

    def _on_spawned(self, actor: Actor, curr_point: CarlaTrajectoryPoint) -> None:
        """Hook called immediately after the actor is spawned during step().

        Override to perform post-spawn actions such as applying an initial
        velocity kick. Default implementation is a no-op.
        """
        return

    def step(
        self,
        actor: Actor,
        simulation_time: float,
        deviation_statistics: Optional[Statistic] = None,
    ) -> None:
        """Advance the actor along target_points (a Carla_Trajectory_Point)."""
        self._validate_statistics(deviation_statistics)

        curr_point = actor.get_current_trajectory_point()

        if not actor.is_spawned():
            if simulation_time >= curr_point.time - self._temporal_trigger:
                logging.info(
                    f"{actor.name}: trying spawn at sim time {simulation_time}, "
                    f"traj time {curr_point.time}, temporal trigger {self._temporal_trigger}"
                )
                actor.spawn()
                self._on_spawned(actor, curr_point)
                self._pass_point_to_stats(
                    actor.get_current_trajectory_point(), deviation_statistics
                )
            return

        if self._traversed_trajectory:
            return

        self._move(actor, simulation_time, deviation_statistics)

    def _move(
        self,
        actor: Actor,
        simulation_time: float,
        deviation_statistics: Optional[Statistic],
    ) -> None:
        raise NotImplementedError


class PIDMovement(MovementPolicy):
    """Move using CARLA's VehiclePIDController aiming at the target waypoint."""

    MIN_DIST_TO_WAYPOINT = 5

    def __init__(
        self,
        lateral_dict: dict = None,
        longitudinal_dict: dict = None,
        max_throttle: float = 0.75,
        max_brake: float = 0.3,
        max_steering: float = 0.8,
        dt: float = 1.0 / 20.0,
        temporal_trigger: float = 0.01,
    ) -> None:
        # Setup PID controller parameters, use defaults if not provided
        lateral_default = {"K_P": 1.95, "K_I": 0.05, "K_D": 0.2, "dt": dt}
        longitudinal_default = {"K_P": 1.0, "K_I": 0.05, "K_D": 0, "dt": dt}
        self.lateral = lateral_dict or lateral_default
        self.longitudinal = longitudinal_dict or longitudinal_default
        self.max_throttle = max_throttle
        self.max_brake = max_brake
        self.max_steering = max_steering
        self._controller = None

        self._traversed_trajectory = False
        self._temporal_trigger = temporal_trigger

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

    def _on_spawned(self, actor: Actor, curr_point: CarlaTrajectoryPoint) -> None:
        # bring actor up to starting speed
        speed_vector = curr_point.transform.get_forward_vector() * (
            float(curr_point.speed) / 3.6
        )
        actor.kick(speed_vector)

    def _move(
        self,
        actor: Actor,
        simulation_time: float,
        deviation_statistics: Optional[Statistic],
    ) -> None:
        curr_point = actor.get_current_trajectory_point()

        while True:
            dist_to_waypoint = (
                actor.get_actor().get_location().distance(curr_point.transform.location)
            )
            if dist_to_waypoint < self.MIN_DIST_TO_WAYPOINT:
                curr_point = self._advance_or_finish(
                    actor, simulation_time, deviation_statistics
                )
                if curr_point is None:
                    return
            else:
                break

        # perform a measurement by passing the vehicles current location
        if deviation_statistics is not None:
            deviation_statistics.add(
                (actor.get_actor().get_transform(), simulation_time)
            )

        control = self._controller.run_step(
            actor.get_current_trajectory_point().speed,
            actor.get_current_trajectory_point(),
        )

        actor.get_actor().apply_control(control)


class PIDMovementTimestampAdvanced(PIDMovement):
    """PID movement that advances trajectory based on timestamps
    instead of distance to waypoint."""

    def _move(
        self,
        actor: Actor,
        simulation_time: float,
        deviation_statistics: Optional[Statistic],
    ) -> None:
        curr_point = actor.get_current_trajectory_point()

        # advance trajectory based on timestamps
        while simulation_time >= curr_point.time + self._temporal_trigger:
            curr_point = self._advance_or_finish(
                actor, simulation_time, deviation_statistics
            )
            if curr_point is None:
                return

        # perform a measurement by passing the vehicles current location
        if deviation_statistics is not None:
            deviation_statistics.add(
                (actor.get_actor().get_transform(), simulation_time)
            )

        control = self._controller.run_step(
            actor.get_current_trajectory_point().speed,
            actor.get_current_trajectory_point(),
        )

        actor.get_actor().apply_control(control)


class TeleportMovement(MovementPolicy):
    """Move by setting the actor's transform directly at each timestamp.
    Optionally sets target velocity to match the speed in m/s."""

    def on_spawn(self, actor: Actor) -> None:
        actor.get_actor().set_simulate_physics(True)

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

    def _move(
        self,
        actor: Actor,
        simulation_time: float,
        deviation_statistics: Optional[Statistic],
    ) -> None:
        """Move the vehicle to the next trajectory point by teleporting it.

        This method updates the vehicle's transform and optionally its velocity.
        It uses the trajectory generator to get the next point and advances
        through the trajectory.

        Args:
            actor (Actor): The Vehicle to move.
            simulation_time: The current time in the simulation.
        """
        curr_point = actor.get_current_trajectory_point()

        # Place vehicle exactly at the time-aligned trajectory point.
        actor.get_actor().set_transform(curr_point.transform)

        # Convert km/h to m/s and set a velocity vector aligned with yaw.
        speed_ms = float(curr_point.speed) / 3.6
        yaw_rad = math.radians(curr_point.transform.rotation.yaw)
        vx = math.cos(yaw_rad) * speed_ms
        vy = math.sin(yaw_rad) * speed_ms
        actor.get_actor().set_target_velocity(carla.Vector3D(vx, vy, 0))

        if deviation_statistics is not None:
            deviation_statistics.add(
                (actor.get_actor().get_transform(), simulation_time)
            )

        self._advance_or_finish(actor, simulation_time, deviation_statistics)


class PCLA_Movement(MovementPolicy):
    def __init__(
        self,
        pcla_agent: PCLA_Agent,
        client: carla.Client,
        spawn_timepoint: float = -1.0,
    ) -> None:
        """
        Initialize the PCLA_Movement policy.

        Args:
            pcla_agent (PCLA_Agent): The PCLA agent responsible for decision making
            client (carla.Client): CARLA client instance
            spawn_timepoint (float): The simulation time at which the vehicle should be spawned and start following the trajectory.
                If set to a negative value, the vehicle will be spawned immediately at the first trajectory point. Defaults to -1.0.
        """
        self._pcla_agent = pcla_agent
        self._client = client
        self._pcla = None
        self._throttle_exponent = 1.0
        self._route_completed_logged = False
        self._spawn_timepoint = spawn_timepoint

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
        if simulation_time < self._spawn_timepoint:
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

        # Always call get_action() to drain sensor buffers and avoid
        # unbounded memory growth from queued camera/lidar frames.
        ego_action = self._pcla.get_action()

        if self._route_completed_logged:
            actor.get_actor().apply_control(
                carla.VehicleControl(steer=0.0, throttle=0.0, brake=1.0)
            )
            return

        ego_action.throttle = math.pow(ego_action.throttle, self._throttle_exponent)

        # Check after get_action (which updates the route planner) if route is done
        if self._pcla.route_completed:
            ego_action = carla.VehicleControl(steer=0.0, throttle=0.0, brake=1.0)
            logging.info(
                f"{actor.name}: reached end of trajectory (`self._pcla.route_completed`). Stopping."
            )
            self._route_completed_logged = True

        actor.get_actor().apply_control(ego_action)

    def set_throttle_exponent(self, exponent: float) -> None:
        """
        Set the throttle exponent used to adjust throttle sensitivity.
        throttle -> throttle ^ exponent
        As throttle in [0;1], exponent < 1 increases sensitivity, exponent > 1 decreases sensitivity.

        Args:
            exponent (float): The exponent to set for throttle adjustment.
        """
        self._throttle_exponent = exponent


class BehaviorMovement(MovementPolicy):
    """
    Move a vehicle using CARLA's BehaviorAgent (fully autonomous, routes on the CARLA road network).

    Agent creation and route planning are intentionally deferred to the first step() call
    after spawning so that at least one world.tick() has occurred in synchronous mode.
    This ensures the vehicle has settled onto the road surface before get_waypoint() is
    called, preventing GlobalRoutePlanner from planning routes through off-road terrain.

    An optional XML route file (same format used by PCLA) is supported: the first waypoint
    is the spawn transform and the last waypoint is the initial destination.  If no route is
    supplied, the agent roams to a random reachable spawn point.
    """

    validate_z: bool = False  # physics-enabled; CARLA settles the vehicle itself

    def __init__(
        self,
        client: carla.Client,
        behavior: str = "cautious",
        xml_route: Optional[str] = None,
        lateral_yield: float = 15.0,
        temporal_trigger: float = 0.01,
        on_route_done: str = "stop",
        route_done_distance: float = 5.0,
        enforce_driving_lane: bool = True,
        rng: Optional[_random_module.Random] = None,
        speed_provider: Optional[OpenDriveSpeedProvider] = None,
    ) -> None:
        self._client = client
        self._behavior = behavior
        self._xml_route = xml_route
        self._lateral_yield = lateral_yield
        self._temporal_trigger = temporal_trigger
        self._on_route_done = on_route_done  # "stop" | "destroy" | "roam"
        self._route_done_distance = route_done_distance
        self._enforce_driving_lane = enforce_driving_lane
        self._rng = rng or _random_module.Random()
        self._agent: Optional[BehaviorAgent] = None
        self._destination: Optional[carla.Location] = None
        self._route_done = False
        self._agent_initialized = False  # True after first post-spawn step
        self._offroad_stop_logged = False
        self.speed_provider = speed_provider

    def on_spawn(self, actor: Actor) -> None:
        # Intentionally empty: agent is created in the first step() after spawn
        # so that world.tick() has run and the vehicle is on the road surface.
        pass

    def _init_agent(self, actor: Actor) -> None:
        """Create the BehaviorAgent and set destination. Called on first step after spawn."""
        world = actor.get_actor().get_world()
        self._agent = BehaviorAgent(
            actor.get_actor(),
            behavior=self._behavior,
            opt_dict={"lateral_yield_distance": self._lateral_yield},
            speed_provider=self.speed_provider,
        )
        self._agent._npc_mode = True

        if self._xml_route is not None:
            tree = ET.parse(self._xml_route)
            waypoints = tree.getroot().findall("waypoint")
            last = waypoints[-1]
            dest = carla.Location(
                x=float(last.get("x")),
                y=float(last.get("y")),
                z=float(last.get("z", 0.0)),
            )
        else:
            spawn_points = world.get_map().get_spawn_points()
            dest = self._rng.choice(spawn_points).location

        self._destination = dest
        self._agent.set_destination(dest)
        self._agent_initialized = True
        logging.info(
            f"{actor.name}: BehaviorMovement destination set to ({dest.x:.1f}, {dest.y:.1f})"
        )

    def _distance_to_destination(self, actor: Actor) -> Optional[float]:
        if self._destination is None:
            return None
        location = actor.get_actor().get_location()
        return math.hypot(
            location.x - self._destination.x, location.y - self._destination.y
        )

    def _apply_stop(self, actor: Actor) -> None:
        actor.get_actor().apply_control(
            carla.VehicleControl(steer=0.0, throttle=0.0, brake=1.0)
        )

    def _is_on_driving_lane(self, actor: Actor) -> bool:
        if not self._enforce_driving_lane:
            return True
        world = actor.get_actor().get_world()
        location = actor.get_actor().get_location()
        waypoint = world.get_map().get_waypoint(
            location,
            project_to_road=False,
            lane_type=carla.LaneType.Driving,
        )
        return waypoint is not None

    def _stop_if_offroad(self, actor: Actor) -> bool:
        if self._is_on_driving_lane(actor):
            return False

        if not self._offroad_stop_logged:
            location = actor.get_actor().get_location()
            logging.warning(
                f"{actor.name}: BehaviorMovement stopped off-road at "
                f"({location.x:.1f}, {location.y:.1f}, {location.z:.1f})"
            )
            self._offroad_stop_logged = True

        self._route_done = True
        self._apply_stop(actor)
        return True

    def _handle_route_done(self, actor: Actor) -> None:
        logging.info(
            f"{actor.name}: BehaviorMovement route completed ({self._on_route_done})."
        )
        if self._on_route_done == "destroy":
            actor.destroy()
        elif self._on_route_done == "roam":
            world = actor.get_actor().get_world()
            spawn_points = world.get_map().get_spawn_points()
            dest = self._rng.choice(spawn_points).location
            self._destination = dest
            self._agent.set_destination(dest)
            logging.info(
                f"{actor.name}: BehaviorMovement new roam destination ({dest.x:.1f}, {dest.y:.1f})"
            )
        else:  # "stop"
            self._route_done = True
            self._apply_stop(actor)

    def step(
        self,
        actor: Actor,
        simulation_time: Optional[float],
        deviation_statistics: Optional[Statistic] = None,
    ) -> None:
        if not actor.is_spawned():
            curr_traj_time = actor.get_current_trajectory_point().time
            if simulation_time >= curr_traj_time - self._temporal_trigger:
                actor.spawn()
            return

        # Defer agent creation until after the first world.tick()
        if not self._agent_initialized:
            self._init_agent(actor)
            return

        if self._route_done:
            self._apply_stop(actor)
            return

        if self._stop_if_offroad(actor):
            return

        distance_to_destination = self._distance_to_destination(actor)
        reached_xml_destination = (
            self._xml_route is not None
            and distance_to_destination is not None
            and distance_to_destination <= self._route_done_distance
        )
        if reached_xml_destination or self._agent.done():
            self._handle_route_done(actor)
            return

        control = self._agent.run_step()
        control.manual_gear_shift = False
        actor.get_actor().apply_control(control)
