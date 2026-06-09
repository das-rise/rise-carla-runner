# Copyright (c) # Copyright (c) 2018-2020 CVC.
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.


"""This module implements an agent that roams around a track following random
waypoints and avoiding other vehicles. The agent also responds to traffic lights,
traffic signs, and has different possible configurations."""

import carla
import numpy as np

from rirun.carla_agents.navigation.basic_agent import BasicAgent
from rirun.carla_agents.navigation.behavior_types import Aggressive, Cautious, Normal
from rirun.carla_agents.navigation.local_planner import RoadOption
from rirun.carla_agents.tools.misc import get_speed, positive


class BehaviorAgent(BasicAgent):
    """
    BehaviorAgent implements an agent that navigates scenes to reach a given
    target destination, by computing the shortest possible path to it.
    This agent can correctly follow traffic signs, speed limitations,
    traffic lights, while also taking into account nearby vehicles. Lane changing
    decisions can be taken by analyzing the surrounding environment such as tailgating avoidance.
    Adding to these are possible behaviors, the agent can also keep safety distance
    from a car in front of it by tracking the instantaneous time to collision
    and keeping it in a certain range. Finally, different sets of behaviors
    are encoded in the agent, from cautious to a more aggressive ones.
    """

    def __init__(
        self,
        vehicle,
        behavior="normal",
        opt_dict={},
        map_inst=None,
        grp_inst=None,
        speed_provider=None,
    ):
        """
        Constructor method.

            :param vehicle: actor to apply to local planner logic onto
            :param behavior: type of agent to apply
        """

        super().__init__(
            vehicle, opt_dict=opt_dict, map_inst=map_inst, grp_inst=grp_inst
        )
        self._look_ahead_steps = 0

        # Vehicle information
        self._speed = 0
        self._speed_limit = 0
        self._direction = None
        self._incoming_direction = None
        self._incoming_waypoint = None
        self._min_speed = 5
        self._behavior = None
        self._sampling_resolution = 4.5
        self._speed_provider = speed_provider

        # Parameters for agent behavior
        if behavior == "cautious":
            self._behavior = Cautious()

        elif behavior == "normal":
            self._behavior = Normal()

        elif behavior == "aggressive":
            self._behavior = Aggressive()

    def _update_information(self):
        """
        This method updates the information regarding the ego
        vehicle based on the surrounding world.
        """
        self._speed = get_speed(self._vehicle)
        if self._speed_provider is not None:
            self._speed_limit = self._speed_provider.get_speed_limit(
                self._vehicle.get_location()
            )
            print(
                f"OpenDrive-extracted speed limit at current location: {self._speed_limit} m/s"
            )
        else:
            self._speed_limit = self._vehicle.get_speed_limit()
        self._local_planner.set_speed(self._speed_limit)
        self._direction = self._local_planner.target_road_option
        if self._direction is None:
            self._direction = RoadOption.LANEFOLLOW

        self._look_ahead_steps = int((self._speed_limit) / 10)

        self._incoming_waypoint, self._incoming_direction = (
            self._local_planner.get_incoming_waypoint_and_direction(
                steps=self._look_ahead_steps
            )
        )
        if self._incoming_direction is None:
            self._incoming_direction = RoadOption.LANEFOLLOW

    def traffic_light_manager(self):
        """
        This method is in charge of behaviors for red lights.
        """
        actor_list = self._world.get_actors()
        lights_list = actor_list.filter("*traffic_light*")
        affected, _ = self._affected_by_traffic_light(lights_list)

        return affected

    def _tailgating(self, waypoint, vehicle_list):
        """
        This method is in charge of tailgating behaviors.

            :param location: current location of the agent
            :param waypoint: current waypoint of the agent
            :param vehicle_list: list of all the nearby vehicles
        """

        left_marking = waypoint.left_lane_marking
        right_marking = waypoint.right_lane_marking
        left_turn = (
            left_marking.lane_change
            if left_marking is not None
            else carla.LaneChange.NONE
        )
        right_turn = (
            right_marking.lane_change
            if right_marking is not None
            else carla.LaneChange.NONE
        )

        left_wpt = waypoint.get_left_lane()
        right_wpt = waypoint.get_right_lane()

        behind_vehicle_state, behind_vehicle, _ = self._vehicle_obstacle_detected(
            vehicle_list,
            max(self._behavior.min_proximity_threshold, self._speed_limit / 2),
            up_angle_th=180,
            low_angle_th=160,
        )
        if behind_vehicle_state and self._speed < get_speed(behind_vehicle):
            if (
                (
                    right_turn == carla.LaneChange.Right
                    or right_turn == carla.LaneChange.Both
                )
                and right_wpt is not None
                and waypoint.lane_id * right_wpt.lane_id > 0
                and right_wpt.lane_type == carla.LaneType.Driving
            ):
                new_vehicle_state, _, _ = self._vehicle_obstacle_detected(
                    vehicle_list,
                    max(self._behavior.min_proximity_threshold, self._speed_limit / 2),
                    up_angle_th=180,
                    lane_offset=1,
                )
                if not new_vehicle_state:
                    print("Tailgating, moving to the right!")
                    end_waypoint = self._local_planner.target_waypoint
                    self._behavior.tailgate_counter = 200
                    try:
                        self.set_destination(
                            end_waypoint.transform.location,
                            right_wpt.transform.location,
                        )
                    except (ValueError, Exception):
                        pass  # Skip if route is invalid
            elif (
                left_turn == carla.LaneChange.Left
                and left_wpt is not None
                and waypoint.lane_id * left_wpt.lane_id > 0
                and left_wpt.lane_type == carla.LaneType.Driving
            ):
                new_vehicle_state, _, _ = self._vehicle_obstacle_detected(
                    vehicle_list,
                    max(self._behavior.min_proximity_threshold, self._speed_limit / 2),
                    up_angle_th=180,
                    lane_offset=-1,
                )
                if not new_vehicle_state:
                    print("Tailgating, moving to the left!")
                    end_waypoint = self._local_planner.target_waypoint
                    self._behavior.tailgate_counter = 200
                    try:
                        self.set_destination(
                            end_waypoint.transform.location, left_wpt.transform.location
                        )
                    except (ValueError, Exception):
                        pass  # Skip if route is invalid

    def collision_and_car_avoid_manager(self, waypoint):
        """
        This module is in charge of warning in case of a collision
        and managing possible tailgating chances.

            :param location: current location of the agent
            :param waypoint: current waypoint of the agent
            :return vehicle_state: True if there is a vehicle nearby, False if not
            :return vehicle: nearby vehicle
            :return distance: distance to nearby vehicle
        """

        vehicle_list = self._world.get_actors().filter("*vehicle*")

        def dist(v):
            return v.get_location().distance(waypoint.transform.location)

        vehicle_list = [
            v for v in vehicle_list if dist(v) < 45 and v.id != self._vehicle.id
        ]

        if self._direction == RoadOption.CHANGELANELEFT:
            vehicle_state, vehicle, distance = self._vehicle_obstacle_detected(
                vehicle_list,
                max(self._behavior.min_proximity_threshold, self._speed_limit / 2),
                up_angle_th=180,
                lane_offset=-1,
            )
        elif self._direction == RoadOption.CHANGELANERIGHT:
            vehicle_state, vehicle, distance = self._vehicle_obstacle_detected(
                vehicle_list,
                max(self._behavior.min_proximity_threshold, self._speed_limit / 2),
                up_angle_th=180,
                lane_offset=1,
            )
        else:
            # Widen detection cone to 90° at junctions: ring followers on curves and
            # merge-approach vehicles can be 30–60° off the forward axis.
            _near_junc = waypoint.is_junction or (
                self._incoming_waypoint is not None
                and self._incoming_waypoint.is_junction
            )
            vehicle_state, vehicle, distance = self._vehicle_obstacle_detected(
                vehicle_list,
                max(self._behavior.min_proximity_threshold, self._speed_limit),
                up_angle_th=90 if _near_junc else 30,
            )

            # Check for tailgating
            if (
                not vehicle_state
                and self._direction == RoadOption.LANEFOLLOW
                and not waypoint.is_junction
                and self._speed > 10
                and self._behavior.tailgate_counter == 0
            ):
                self._tailgating(waypoint, vehicle_list)

        # if vehicle_state:
        # print(f"Vehicle detected at distance {distance:.2f} m, speed {get_speed(vehicle):.2f} km/h")
        return vehicle_state, vehicle, distance

    def pedestrian_avoid_manager(self, waypoint):
        """
        This module is in charge of warning in case of a collision
        with any pedestrian.

            :param location: current location of the agent
            :param waypoint: current waypoint of the agent
            :return vehicle_state: True if there is a walker nearby, False if not
            :return vehicle: nearby walker
            :return distance: distance to nearby walker
        """

        walker_list = self._world.get_actors().filter("*walker.pedestrian*")

        def dist(w):
            return w.get_location().distance(waypoint.transform.location)

        walker_list = [w for w in walker_list if dist(w) < 10]

        if self._direction == RoadOption.CHANGELANELEFT:
            walker_state, walker, distance = self._vehicle_obstacle_detected(
                walker_list,
                max(self._behavior.min_proximity_threshold, self._speed_limit / 2),
                up_angle_th=90,
                lane_offset=-1,
            )
        elif self._direction == RoadOption.CHANGELANERIGHT:
            walker_state, walker, distance = self._vehicle_obstacle_detected(
                walker_list,
                max(self._behavior.min_proximity_threshold, self._speed_limit / 2),
                up_angle_th=90,
                lane_offset=1,
            )
        else:
            walker_state, walker, distance = self._vehicle_obstacle_detected(
                walker_list,
                max(self._behavior.min_proximity_threshold, self._speed_limit / 3),
                up_angle_th=60,
            )

        return walker_state, walker, distance

    def car_following_manager(self, vehicle, distance, debug=False):
        """
        Module in charge of car-following behaviors when there's
        someone in front of us.

            :param vehicle: car to follow
            :param distance: distance from vehicle
            :param debug: boolean for debugging
            :return control: carla.VehicleControl
        """

        vehicle_speed = get_speed(vehicle)

        # If the vehicle is crossing our path, treat its speed as 0 so we brake instead of following
        ego_fwd = self._vehicle.get_transform().get_forward_vector()
        target_fwd = vehicle.get_transform().get_forward_vector()
        dot_product = (
            ego_fwd.x * target_fwd.x
            + ego_fwd.y * target_fwd.y
            + ego_fwd.z * target_fwd.z
        )
        # Ignore vehicles going in the opposite direction (dot_product < -0.7).
        # _vehicle_obstacle_detected already guarantees the vehicle is path-blocking;
        # we only need to suppress head-on opposite-lane traffic.
        if dot_product < -0.7:
            # Vehicle is going in the opposite direction; ignore for car-following.
            vehicle_speed = self._speed  # pretend no obstacle
        elif dot_product < 0.5:
            # Vehicle is crossing our path (lateral / entering from side road).
            # Force a full stop regardless of yield sign availability.
            vehicle_speed = 0.0

        # Ring vehicles have absolute priority over vehicles still on approach roads.
        # At tangential merges the heading vectors nearly align (dot > 0.5), which
        # would otherwise cause the entering vehicle to merely "follow" the ring
        # vehicle at speed instead of stopping to give way.  Override here.
        _veh_wpt = self._map.get_waypoint(vehicle.get_location())
        _ego_wpt = self._map.get_waypoint(self._vehicle.get_location())
        if _veh_wpt.is_junction and not _ego_wpt.is_junction:
            vehicle_speed = 0.0

        delta_v = (self._speed - vehicle_speed) / 3.6
        if delta_v < 0:
            # Vehicle ahead is faster than us — follow it at its speed, no collision risk
            target_speed = min(
                [
                    max(self._min_speed, vehicle_speed),
                    self._behavior.max_speed,
                    self._speed_limit - self._behavior.speed_lim_dist,
                ]
            )
            self._local_planner.set_speed(target_speed)
            return self._local_planner.run_step(debug=debug)
        delta_v = max(1, delta_v)
        ttc = distance / delta_v if delta_v != 0 else distance / np.nextafter(0.0, 1.0)

        # Emergency brake if TTC is critically low (< half safety_time) to overcome
        # PID lag when a slow/stopped vehicle is detected at close range.
        if ttc < self._behavior.safety_time / 2.0 and vehicle_speed < 1.0:
            return self.emergency_stop()

        # Under safety time distance, slow down.
        if self._behavior.safety_time > ttc > 0.0:
            target_speed = min(
                [
                    positive(vehicle_speed - self._behavior.speed_decrease),
                    self._behavior.max_speed,
                    self._speed_limit - self._behavior.speed_lim_dist,
                ]
            )
            self._local_planner.set_speed(target_speed)
            control = self._local_planner.run_step(debug=debug)

        # Actual safety distance area, try to follow the speed of the vehicle in front.
        elif 2 * self._behavior.safety_time > ttc >= self._behavior.safety_time:
            target_speed = min(
                [
                    max(self._min_speed, vehicle_speed),
                    self._behavior.max_speed,
                    self._speed_limit - self._behavior.speed_lim_dist,
                ]
            )
            self._local_planner.set_speed(target_speed)
            control = self._local_planner.run_step(debug=debug)

        # Normal behavior.
        else:
            target_speed = min(
                [
                    self._behavior.max_speed,
                    self._speed_limit - self._behavior.speed_lim_dist,
                ]
            )
            self._local_planner.set_speed(target_speed)
            control = self._local_planner.run_step(debug=debug)

        return control

    def run_step(self, debug=False):
        """
        Execute one step of navigation.

            :param debug: boolean for debugging
            :return control: carla.VehicleControl
        """
        self._update_information()

        control = None
        hazard = False
        if self._behavior.tailgate_counter > 0:
            self._behavior.tailgate_counter -= 1

        ego_vehicle_loc = self._vehicle.get_location()
        ego_vehicle_wp = self._map.get_waypoint(ego_vehicle_loc)

        # 1: Red lights and stops behavior
        if self.traffic_light_manager():
            hazard = True

        # 2.1: Pedestrian avoidance behaviors
        if not hazard:
            walker_state, walker, w_distance = self.pedestrian_avoid_manager(
                ego_vehicle_wp
            )

            if walker_state:
                distance = (
                    w_distance
                    - max(walker.bounding_box.extent.y, walker.bounding_box.extent.x)
                    - max(
                        self._vehicle.bounding_box.extent.y,
                        self._vehicle.bounding_box.extent.x,
                    )
                )
                if distance < self._behavior.braking_distance:
                    hazard = True

        # 2.2: Car following behaviors
        if not hazard:
            vehicle_state, vehicle, distance = self.collision_and_car_avoid_manager(
                ego_vehicle_wp
            )

            if vehicle_state:
                distance = (
                    distance
                    - max(vehicle.bounding_box.extent.y, vehicle.bounding_box.extent.x)
                    - max(
                        self._vehicle.bounding_box.extent.y,
                        self._vehicle.bounding_box.extent.x,
                    )
                )
                if distance < self._behavior.braking_distance:
                    hazard = True
                else:
                    control = self.car_following_manager(vehicle, distance)

        # 3: Intersection behavior
        if control is None and not hazard:
            if (
                self._incoming_waypoint is not None
                and self._incoming_waypoint.is_junction
                and (self._incoming_direction in [RoadOption.LEFT, RoadOption.RIGHT])
            ):
                target_speed = min([self._behavior.max_speed, self._speed_limit - 5])
                self._local_planner.set_speed(target_speed)
                control = self._local_planner.run_step(debug=debug)

        # 4: Normal behavior
        if control is None and not hazard:
            target_speed = min(
                [
                    self._behavior.max_speed,
                    self._speed_limit - self._behavior.speed_lim_dist,
                ]
            )
            self._local_planner.set_speed(target_speed)
            control = self._local_planner.run_step(debug=debug)

        # Always run the local planner to keep plan management active
        # (callback, auto-roam, purge) even when emergency-stopping.
        if hazard:
            self._local_planner.run_step(debug=debug)
            control = self.emergency_stop()

        return control

    def emergency_stop(self):
        """
        Overwrites the throttle a brake values of a control to perform an emergency stop.
        The steering is kept the same to avoid going out of the lane when stopping during turns

            :param speed (carl.VehicleControl): control to be modified
        """
        control = carla.VehicleControl()
        control.throttle = 0.0
        control.brake = self._max_brake
        control.hand_brake = False
        return control
