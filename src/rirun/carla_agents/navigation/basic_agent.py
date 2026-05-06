# Copyright (c) # Copyright (c) 2018-2020 CVC.
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
This module implements an agent that roams around a track following random
waypoints and avoiding other vehicles. The agent also responds to traffic lights.
It can also make use of the global route planner to follow a specifed route
"""

import carla
import math
from shapely.geometry import Polygon

from rirun.carla_agents.navigation.local_planner import LocalPlanner, RoadOption
from rirun.carla_agents.navigation.global_route_planner import GlobalRoutePlanner
from rirun.carla_agents.tools.misc import (get_speed, is_within_distance,
                                           get_trafficlight_trigger_location,
                                           compute_distance)


class BasicAgent(object):
    """
    BasicAgent implements an agent that navigates the scene.
    This agent respects traffic lights and other vehicles, but ignores stop signs.
    It has several functions available to specify the route that the agent must follow,
    as well as to change its parameters in case a different driving mode is desired.
    """

    def __init__(self, vehicle, target_speed=20, opt_dict={}, map_inst=None, grp_inst=None):
        """
        Initialization the agent paramters, the local and the global planner.

            :param vehicle: actor to apply to agent logic onto
            :param target_speed: speed (in Km/h) at which the vehicle will move
            :param opt_dict: dictionary in case some of its parameters want to be changed.
                This also applies to parameters related to the LocalPlanner.
            :param map_inst: carla.Map instance to avoid the expensive call of getting it.
            :param grp_inst: GlobalRoutePlanner instance to avoid the expensive call of getting it.

        """
        self._vehicle = vehicle
        self._world = self._vehicle.get_world()
        if map_inst:
            if isinstance(map_inst, carla.Map):
                self._map = map_inst
            else:
                print("Warning: Ignoring the given map as it is not a 'carla.Map'")
                self._map = self._world.get_map()
        else:
            self._map = self._world.get_map()
        self._last_traffic_light = None

        # Base parameters
        self._ignore_traffic_lights = False
        self._ignore_stop_signs = False
        self._ignore_vehicles = False
        self._use_bbs_detection = False
        self._target_speed = target_speed
        self._sampling_resolution = 2.0
        self._base_tlight_threshold = 5.0  # meters
        self._base_vehicle_threshold = 5.0  # meters
        self._speed_ratio = 1
        self._max_brake = 0.5
        self._offset = 0

        # Change parameters according to the dictionary
        opt_dict['target_speed'] = target_speed
        if 'ignore_traffic_lights' in opt_dict:
            self._ignore_traffic_lights = opt_dict['ignore_traffic_lights']
        if 'ignore_stop_signs' in opt_dict:
            self._ignore_stop_signs = opt_dict['ignore_stop_signs']
        if 'ignore_vehicles' in opt_dict:
            self._ignore_vehicles = opt_dict['ignore_vehicles']
        if 'use_bbs_detection' in opt_dict:
            self._use_bbs_detection = opt_dict['use_bbs_detection']
        if 'sampling_resolution' in opt_dict:
            self._sampling_resolution = opt_dict['sampling_resolution']
        if 'base_tlight_threshold' in opt_dict:
            self._base_tlight_threshold = opt_dict['base_tlight_threshold']
        if 'base_vehicle_threshold' in opt_dict:
            self._base_vehicle_threshold = opt_dict['base_vehicle_threshold']
        if 'detection_speed_ratio' in opt_dict:
            self._speed_ratio = opt_dict['detection_speed_ratio']
        if 'max_brake' in opt_dict:
            self._max_brake = opt_dict['max_brake']
        if 'offset' in opt_dict:
            self._offset = opt_dict['offset']

        self._lateral_yield_distance = 5.0
        if 'lateral_yield_distance' in opt_dict:
            self._lateral_yield_distance = opt_dict['lateral_yield_distance']

        self._last_yield_id = -1

        # Initialize the planners
        self._local_planner = LocalPlanner(self._vehicle, opt_dict=opt_dict, map_inst=self._map)
        if grp_inst:
            if isinstance(grp_inst, GlobalRoutePlanner):
                self._global_planner = grp_inst
            else:
                print("Warning: Ignoring the given map as it is not a 'carla.Map'")
                self._global_planner = GlobalRoutePlanner(self._map, self._sampling_resolution)
        else:
            self._global_planner = GlobalRoutePlanner(self._map, self._sampling_resolution)

        # Get the static elements of the scene
        self._lights_list = self._world.get_actors().filter("*traffic_light*")
        self._lights_map = {}  # Dictionary mapping a traffic light to a wp corrspoing to its trigger volume location
        self._yield_signs_list = self._world.get_actors().filter("*yield*")
        self._yield_signs_map = {}

    def add_emergency_stop(self, control):
        """
        Overwrites the throttle a brake values of a control to perform an emergency stop.
        The steering is kept the same to avoid going out of the lane when stopping during turns

            :param speed (carl.VehicleControl): control to be modified
        """
        control.throttle = 0.0
        control.brake = self._max_brake
        control.hand_brake = False
        return control

    def set_target_speed(self, speed):
        """
        Changes the target speed of the agent
            :param speed (float): target speed in Km/h
        """
        self._target_speed = speed
        self._local_planner.set_speed(speed)

    def follow_speed_limits(self, value=True):
        """
        If active, the agent will dynamically change the target speed according to the speed limits

            :param value (bool): whether or not to activate this behavior
        """
        self._local_planner.follow_speed_limits(value)

    def get_local_planner(self):
        """Get method for protected member local planner"""
        return self._local_planner

    def get_global_planner(self):
        """Get method for protected member local planner"""
        return self._global_planner

    def _is_uturn_plan(self, plan):
        """
        Check if the plan contains a U-turn (angle difference > 90-120 degrees).
        """
        if len(plan) < 2:
            return False

        vehicle_transform = self._vehicle.get_transform()
        forward_vec = vehicle_transform.get_forward_vector()
        current_loc = vehicle_transform.location

        # 1. Broad Dot Product Checks (Short window - Fix 4.0)
        # Only check immediate backward motion to avoid false-positives in curves
        # Check waypoints at index 3, 6 (approx 6-12m)
        for i in [3, 6]:
            if i < len(plan):
                p_check = plan[i][0].transform.location
                vec_to_check = p_check - current_loc
                # Normalize the check vector
                norm = math.sqrt(
                    vec_to_check.x**2 + vec_to_check.y**2 + vec_to_check.z**2
                )
                if (
                    norm > 2.0
                ):  # Only check if sufficiently far to have meaningful vector
                    dot = (
                        forward_vec.x * vec_to_check.x + forward_vec.y * vec_to_check.y
                    ) / norm
                    if dot < -0.6:  # Heading significantly backward (Relaxed from -0.3)
                        return True

        # 2. Cumulative Heading Change Check (Very short distance - Fix 4.0)
        # If the plan rotates more than 160 degrees over a short distance, it's a U-turn
        # Roundabouts typically rotate 90 degrees every 15-20 meters.
        if len(plan) > 5:
            start_yaw = plan[0][0].transform.rotation.yaw
            # Only check up to 10 waypoints (approx 20m)
            for i in range(1, min(len(plan), 10)):
                check_yaw = plan[i][0].transform.rotation.yaw
                diff = (check_yaw - start_yaw) % 360
                if diff > 180:
                    diff -= 360
                if abs(diff) > 160:  # Extremely sharp U-turn (Relaxed from 130)
                    return True

        return False

    def set_destination(self, end_location, start_location=None):
        """
        This method creates a list of waypoints between a starting and ending location,
        based on the route returned by the global router, and adds it to the local planner.
        If no starting location is passed, the vehicle local planner's target location is chosen,
        which corresponds (by default), to a location about 5 meters in front of the vehicle.

            :param end_location (carla.Location): final location of the route
            :param start_location (carla.Location): starting location of the route
        """
        if not start_location:
            start_location = self._local_planner.target_waypoint.transform.location
            clean_queue = True
        else:
            clean_queue = False

        start_waypoint = self._map.get_waypoint(start_location)
        end_waypoint = self._map.get_waypoint(end_location)

        route_trace = self.trace_route(start_waypoint, end_waypoint)

        # Validate U-turns for fresh routes (clean_queue=True).
        if clean_queue and self._is_uturn_plan(route_trace):
            raise ValueError('U-turn detected in proposed route. Rejecting destination.')

        # For spliced routes, validate the heading at the splice point.
        # The new route's first waypoints must continue roughly in the same
        # direction as the current plan's last waypoints (no sharp reversal).
        if not clean_queue and len(route_trace) >= 2:
            plan = list(self._local_planner.get_plan())
            if len(plan) >= 2:
                last_yaw = plan[-1][0].transform.rotation.yaw
                splice_yaw = route_trace[min(2, len(route_trace) - 1)][0].transform.rotation.yaw
                diff = (splice_yaw - last_yaw) % 360
                if diff > 180:
                    diff -= 360
                if abs(diff) > 130:
                    raise ValueError('U-turn at splice point. Rejecting destination.')

        self._local_planner.set_global_plan(route_trace, clean_queue=clean_queue)

    def set_global_plan(self, plan, stop_waypoint_creation=True, clean_queue=True):
        """
        Adds a specific plan to the agent.

            :param plan: list of [carla.Waypoint, RoadOption] representing the route to be followed
            :param stop_waypoint_creation: stops the automatic random creation of waypoints
            :param clean_queue: resets the current agent's plan
        """
        self._local_planner.set_global_plan(
            plan,
            stop_waypoint_creation=stop_waypoint_creation,
            clean_queue=clean_queue
        )

    def trace_route(self, start_waypoint, end_waypoint):
        """
        Calculates the shortest route between a starting and ending waypoint.

            :param start_waypoint (carla.Waypoint): initial waypoint
            :param end_waypoint (carla.Waypoint): final waypoint
        """
        start_location = start_waypoint.transform.location
        end_location = end_waypoint.transform.location
        return self._global_planner.trace_route(start_location, end_location)

    def run_step(self):
        """Execute one step of navigation."""
        hazard_detected = False

        # Retrieve all relevant actors
        vehicle_list = self._world.get_actors().filter("*vehicle*")

        vehicle_speed = get_speed(self._vehicle) / 3.6

        # Check for possible vehicle obstacles
        max_vehicle_distance = self._base_vehicle_threshold + self._speed_ratio * vehicle_speed
        affected_by_vehicle, _, _ = self._vehicle_obstacle_detected(vehicle_list, max_vehicle_distance)
        if affected_by_vehicle:
            hazard_detected = True

        # Check if the vehicle is affected by a red traffic light
        max_tlight_distance = self._base_tlight_threshold + self._speed_ratio * vehicle_speed
        affected_by_tlight, _ = self._affected_by_traffic_light(self._lights_list, max_tlight_distance)
        if affected_by_tlight:
            hazard_detected = True

        control = self._local_planner.run_step()
        if hazard_detected:
            control = self.add_emergency_stop(control)

        return control

    def done(self):
        """Check whether the agent has reached its destination."""
        return self._local_planner.done()

    def ignore_traffic_lights(self, active=True):
        """(De)activates the checks for traffic lights"""
        self._ignore_traffic_lights = active

    def ignore_stop_signs(self, active=True):
        """(De)activates the checks for stop signs"""
        self._ignore_stop_signs = active

    def ignore_vehicles(self, active=True):
        """(De)activates the checks for stop signs"""
        self._ignore_vehicles = active

    def set_offset(self, offset):
        """Sets an offset for the vehicle"""
        self._local_planner.set_offset(offset)

    def lane_change(self, direction, same_lane_time=0, other_lane_time=0, lane_change_time=2):
        """
        Changes the path so that the vehicle performs a lane change.
        Use 'direction' to specify either a 'left' or 'right' lane change,
        and the other 3 fine tune the maneuver
        """
        speed = self._vehicle.get_velocity().length()
        path = self._generate_lane_change_path(
            self._map.get_waypoint(self._vehicle.get_location()),
            direction,
            same_lane_time * speed,
            other_lane_time * speed,
            lane_change_time * speed,
            False,
            1,
            self._sampling_resolution
        )
        if not path:
            print("WARNING: Ignoring the lane change as no path was found")

        self.set_global_plan(path)

    def _affected_by_traffic_light(self, lights_list=None, max_distance=None):
        """
        Method to check if there is a red light affecting the vehicle.

            :param lights_list (list of carla.TrafficLight): list containing TrafficLight objects.
                If None, all traffic lights in the scene are used
            :param max_distance (float): max distance for traffic lights to be considered relevant.
                If None, the base threshold value is used
        """
        if self._ignore_traffic_lights:
            return (False, None)

        if not lights_list:
            lights_list = self._world.get_actors().filter("*traffic_light*")

        if not max_distance:
            max_distance = self._base_tlight_threshold

        if self._last_traffic_light:
            if self._last_traffic_light.state != carla.TrafficLightState.Red:
                self._last_traffic_light = None
            else:
                return (True, self._last_traffic_light)

        ego_vehicle_location = self._vehicle.get_location()
        ego_vehicle_waypoint = self._map.get_waypoint(ego_vehicle_location)

        for traffic_light in lights_list:
            if traffic_light.id in self._lights_map:
                trigger_wp = self._lights_map[traffic_light.id]
            else:
                trigger_location = get_trafficlight_trigger_location(traffic_light)
                trigger_wp = self._map.get_waypoint(trigger_location)
                self._lights_map[traffic_light.id] = trigger_wp

            if trigger_wp.transform.location.distance(ego_vehicle_location) > max_distance:
                continue

            if trigger_wp.road_id != ego_vehicle_waypoint.road_id:
                continue

            ve_dir = ego_vehicle_waypoint.transform.get_forward_vector()
            wp_dir = trigger_wp.transform.get_forward_vector()
            dot_ve_wp = ve_dir.x * wp_dir.x + ve_dir.y * wp_dir.y + ve_dir.z * wp_dir.z

            if dot_ve_wp < 0:
                continue

            if traffic_light.state != carla.TrafficLightState.Red:
                continue

            if is_within_distance(trigger_wp.transform, self._vehicle.get_transform(), max_distance, [0, 90]):
                self._last_traffic_light = traffic_light
                return (True, traffic_light)

        return (False, None)

    def _affected_by_yield_sign(self, actor, max_distance=None):
        """
        Method to check if there is a yield sign affecting the actor.

            :param actor (carla.Actor): actor to check.
            :param max_distance (float): max distance for yield signs to be considered relevant.
        """
        if not max_distance:
            max_distance = self._base_tlight_threshold

        actor_location = actor.get_location()
        actor_wpt = self._map.get_waypoint(actor_location)

        for yield_sign in self._yield_signs_list:
            if yield_sign.id in self._yield_signs_map:
                trigger_wp = self._yield_signs_map[yield_sign.id]
            else:
                trigger_location = yield_sign.get_location()
                trigger_wp = self._map.get_waypoint(trigger_location)
                self._yield_signs_map[yield_sign.id] = trigger_wp

            # Check distance first
            trigger_location = trigger_wp.transform.location
            dist = trigger_location.distance(actor_location)
            if dist > max_distance:
                continue

            # Check if it's "ahead" of us (relative to our forward vector)
            actor_fwd = actor_wpt.transform.get_forward_vector()
            diff = trigger_location - actor_location
            dot_ahead = diff.x * actor_fwd.x + diff.y * actor_fwd.y
            if dot_ahead < 0:
                continue

            # Check orientation: The road should have a similar forward vector (within 45 degrees)
            wp_fwd = trigger_wp.transform.get_forward_vector()
            dot_ve_wp = (
                actor_fwd.x * wp_fwd.x + actor_fwd.y * wp_fwd.y + actor_fwd.z * wp_fwd.z
            )
            if dot_ve_wp < 0.7:  # approx 45 degrees
                continue

            if self._last_yield_id != yield_sign.id:
                # print(f"Vehicle {actor.id} detected yield sign {yield_sign.id}")
                self._last_yield_id = yield_sign.id
            return True

        self._last_yield_id = -1
        return False

    def _vehicle_obstacle_detected(self, vehicle_list=None, max_distance=None, up_angle_th=90, low_angle_th=0, lane_offset=0):
        """
        Method to check if there is a vehicle in front of the agent blocking its path.

            :param vehicle_list (list of carla.Vehicle): list contatining vehicle objects.
                If None, all vehicle in the scene are used
            :param max_distance: max freespace to check for obstacles.
                If None, the base threshold value is used
        """

        def get_route_polygon():
            extent_y = self._vehicle.bounding_box.extent.y
            r_ext = extent_y + self._offset
            l_ext = -extent_y + self._offset

            # Widen the detection zone to the left only when approaching the junction,
            # so the entering vehicle can see ring traffic coming from the left.
            # Do NOT widen when already on the ring (ego_wpt.is_junction) — that would
            # cause ring vehicles to sweep over entry roads and incorrectly yield to
            # entering traffic that they have priority over.
            if is_approaching_junction:
                l_ext -= self._lateral_yield_distance

            r_vec = ego_transform.get_right_vector()
            p1 = ego_location + carla.Location(r_ext * r_vec.x, r_ext * r_vec.y)
            p2 = ego_location + carla.Location(l_ext * r_vec.x, l_ext * r_vec.y)

            right_points = [[p1.x, p1.y]]
            left_points = [[p2.x, p2.y]]

            for wp, _ in self._local_planner.get_plan():
                if ego_location.distance(wp.transform.location) > max_distance:
                    break

                r_vec = wp.transform.get_right_vector()
                p1 = wp.transform.location + carla.Location(r_ext * r_vec.x, r_ext * r_vec.y)
                p2 = wp.transform.location + carla.Location(l_ext * r_vec.x, l_ext * r_vec.y)
                right_points.append([p1.x, p1.y])
                left_points.append([p2.x, p2.y])

            # Two points don't create a polygon, nothing to check
            if len(right_points) < 2:
                return None

            route_bb = right_points + left_points[::-1]
            return Polygon(route_bb)

        if self._ignore_vehicles:
            return (False, None, -1)

        if not vehicle_list:
            vehicle_list = self._world.get_actors().filter("*vehicle*")

        # NPC mode flag — filtering moved into the per-vehicle loop below
        # so NPCs on merging/adjacent lanes are still detected.
        _is_npc_mode = getattr(self, '_npc_mode', False)

        if not max_distance:
            max_distance = self._base_vehicle_threshold

        ego_transform = self._vehicle.get_transform()
        ego_location = ego_transform.location
        ego_wpt = self._map.get_waypoint(ego_location)

        # Get the right offset
        if ego_wpt.lane_id < 0 and lane_offset != 0:
            lane_offset *= -1

        # Detect if we are approaching a junction to increase look-ahead
        incoming_wp = None
        if not ego_wpt.is_junction:
            try:
                for step in range(1, 21):
                    incoming_wp, _ = self._local_planner.get_incoming_waypoint_and_direction(steps=step)
                    if incoming_wp and incoming_wp.is_junction:
                        break
            except Exception:
                pass

        if ego_wpt.is_junction or (incoming_wp and incoming_wp.is_junction):
            if max_distance < 30.0:
                max_distance = 30.0

        # Get the transform of the front of the ego
        ego_front_transform = ego_transform
        ego_front_transform.location += carla.Location(
            self._vehicle.bounding_box.extent.x * ego_transform.get_forward_vector())

        opposite_invasion = abs(self._offset) + self._vehicle.bounding_box.extent.y > ego_wpt.lane_width / 2
        is_approaching_junction = incoming_wp and incoming_wp.is_junction
        use_bbs = (
            self._use_bbs_detection
            or opposite_invasion
            or ego_wpt.is_junction
            or is_approaching_junction
        )

        # Detect if we are at a yield sign
        ego_at_yield_sign = self._affected_by_yield_sign(self._vehicle, max_distance)

        # Get the route bounding box
        route_polygon = get_route_polygon()

        for target_vehicle in vehicle_list:
            if target_vehicle.id == self._vehicle.id:
                continue

            target_transform = target_vehicle.get_transform()
            if target_transform.location.distance(ego_location) > max_distance:
                continue

            target_wpt = self._map.get_waypoint(target_transform.location, lane_type=carla.LaneType.Any)

            # NPC mode: skip other NPCs only when ego is physically inside the
            # junction AND the vehicles are heading in opposite directions
            # (genuine deadlock risk — two NPCs facing each other on the same
            # one-way segment).  Same-direction ring followers must NOT be
            # skipped; they need normal car-following so they don't rear-end.
            if _is_npc_mode and target_vehicle.attributes.get('role_name', '') == 'npc':
                if ego_wpt.is_junction and (
                    target_wpt.road_id == ego_wpt.road_id
                    and target_wpt.lane_id == ego_wpt.lane_id
                ):
                    ego_fwd = ego_transform.get_forward_vector()
                    tgt_fwd = target_transform.get_forward_vector()
                    # Only skip if heading in opposite directions (dot < 0)
                    if ego_fwd.x * tgt_fwd.x + ego_fwd.y * tgt_fwd.y < 0:
                        continue

            # General approach for junctions and vehicles invading other lanes due to the offset
            if (use_bbs or target_wpt.is_junction) and route_polygon:

                target_bb = target_vehicle.bounding_box
                target_vertices = target_bb.get_world_vertices(target_vehicle.get_transform())
                target_list = [[v.x, v.y] for v in target_vertices]

                from shapely.geometry import MultiPoint

                # Project the target polygon forward based on its velocity to create a swept volume
                target_vel = target_vehicle.get_velocity()
                target_speed = target_vel.length()
                if target_speed > 0.5:
                    # Use 3.0s projection at junctions to stop early without flying off curves
                    projection_time = (
                        3.0
                        if (
                            ego_at_yield_sign
                            or is_approaching_junction
                            or ego_wpt.is_junction
                        )
                        else 2.0
                    )
                    target_list.extend(
                        [
                            [
                                v[0] + target_vel.x * projection_time,
                                v[1] + target_vel.y * projection_time,
                            ]
                            for v in target_list
                        ]
                    )

                target_polygon = MultiPoint(target_list).convex_hull

                if route_polygon.intersects(target_polygon):
                    # PRIORITY ARBITRATION:
                    if ego_at_yield_sign or ego_wpt.is_junction:
                        if ego_wpt.is_junction and not ego_at_yield_sign:
                            # Ring rule (right-hand traffic): vehicles already on the ring
                            # have priority over vehicles entering from approach roads.
                            # Skip target if it is not yet inside the junction.
                            if not target_wpt.is_junction:
                                continue
                            # Fallback: if the map has yield signs, honour them too.
                            if self._affected_by_yield_sign(target_vehicle, max_distance):
                                continue

                        # ASYMMETRIC PRIORITY: Yield to vehicles coming from the left in roundabouts.
                        ego_yaw = math.radians(ego_transform.rotation.yaw)
                        diff = target_transform.location - ego_location
                        target_yaw = math.atan2(diff.y, diff.x)
                        relative_yaw = math.degrees(target_yaw - ego_yaw)
                        if relative_yaw > 180:
                            relative_yaw -= 360
                        if relative_yaw < -180:
                            relative_yaw += 360

                        # Yield to circular traffic from the left. At a yield sign, also yield from the right.
                        if (-120 < relative_yaw < -15) or (
                            ego_at_yield_sign and 15 < relative_yaw < 120
                        ):
                            pass  # Yield to cross traffic!
                        else:
                            # Vehicle is roughly ahead or to the right.
                            # For ring-ring following (both inside the junction), the polygon
                            # already confirmed the target is in our path.  The road_id-based
                            # target_on_path check is unreliable inside junctions (each arc has
                            # its own road_id), so use a dot-product same-direction check instead.
                            if ego_wpt.is_junction and target_wpt.is_junction:
                                _ef = ego_transform.get_forward_vector()
                                _tf = target_transform.get_forward_vector()
                                if _ef.x * _tf.x + _ef.y * _tf.y > 0.5:
                                    pass  # same-direction ring follower — polygon confirmed, don't skip
                                else:
                                    continue  # opposite/crossing junction vehicle — skip
                            else:
                                # Off-junction: require an exact road/lane match within the plan.
                                target_on_path = False
                                for wp, _ in self._local_planner.get_plan():
                                    if ego_location.distance(wp.transform.location) > 15.0:
                                        break
                                    if (
                                        wp.road_id == target_wpt.road_id
                                        and wp.lane_id == target_wpt.lane_id
                                    ):
                                        target_on_path = True
                                        break
                                if not target_on_path:
                                    continue

                    return (
                        True,
                        target_vehicle,
                        compute_distance(target_vehicle.get_location(), ego_location),
                    )

                # Fallback: if the polygon geometry missed a junction vehicle,
                # entering vehicles still must yield to ring traffic.
                # Any junction vehicle within detection range that is NOT
                # directly behind us (>150 deg) is returned as an obstacle;
                # car_following_manager's dot-product logic then decides speed.
                elif ego_wpt.is_junction and target_wpt.is_junction:
                    # Polygon missed a ring-ring follower (tight curve geometry).
                    # Fall back to the same angle/distance check as the simplified path.
                    _tgt_fwd = target_transform.get_forward_vector()
                    _tgt_ext = target_vehicle.bounding_box.extent.x
                    _tgt_rear = target_transform
                    _tgt_rear.location -= carla.Location(
                        x=_tgt_ext * _tgt_fwd.x,
                        y=_tgt_ext * _tgt_fwd.y,
                    )
                    if is_within_distance(
                        _tgt_rear, ego_front_transform, max_distance, [low_angle_th, up_angle_th]
                    ):
                        return (
                            True,
                            target_vehicle,
                            compute_distance(target_transform.location, ego_transform.location),
                        )
                elif (
                    is_approaching_junction
                    and not ego_wpt.is_junction
                    and target_wpt.is_junction
                ):
                    _ego_yaw = math.radians(ego_transform.rotation.yaw)
                    _diff = target_transform.location - ego_location
                    _rel_yaw = math.degrees(math.atan2(_diff.y, _diff.x) - _ego_yaw)
                    if _rel_yaw > 180:
                        _rel_yaw -= 360
                    if _rel_yaw < -180:
                        _rel_yaw += 360
                    if -150 < _rel_yaw < 150:  # not directly behind us
                        return (
                            True,
                            target_vehicle,
                            compute_distance(target_vehicle.get_location(), ego_location),
                        )

            # Simplified approach, using only the plan waypoints (similar to TM)
            else:

                if target_wpt.road_id != ego_wpt.road_id or target_wpt.lane_id != ego_wpt.lane_id  + lane_offset:
                    next_wpt = self._local_planner.get_incoming_waypoint_and_direction(steps=3)[0]
                    if not next_wpt:
                        continue
                    if target_wpt.road_id != next_wpt.road_id or target_wpt.lane_id != next_wpt.lane_id  + lane_offset:
                        continue

                target_forward_vector = target_transform.get_forward_vector()
                target_extent = target_vehicle.bounding_box.extent.x
                target_rear_transform = target_transform
                target_rear_transform.location -= carla.Location(
                    x=target_extent * target_forward_vector.x,
                    y=target_extent * target_forward_vector.y,
                )

                if is_within_distance(target_rear_transform, ego_front_transform, max_distance, [low_angle_th, up_angle_th]):
                    return (True, target_vehicle, compute_distance(target_transform.location, ego_transform.location))

        return (False, None, -1)

    def _generate_lane_change_path(self, waypoint, direction='left', distance_same_lane=10,
                                distance_other_lane=25, lane_change_distance=25,
                                check=True, lane_changes=1, step_distance=2):
        """
        This methods generates a path that results in a lane change.
        Use the different distances to fine-tune the maneuver.
        If the lane change is impossible, the returned path will be empty.
        """
        distance_same_lane = max(distance_same_lane, 0.1)
        distance_other_lane = max(distance_other_lane, 0.1)
        lane_change_distance = max(lane_change_distance, 0.1)

        plan = []
        plan.append((waypoint, RoadOption.LANEFOLLOW))  # start position

        option = RoadOption.LANEFOLLOW

        # Same lane
        distance = 0
        while distance < distance_same_lane:
            next_wps = plan[-1][0].next(step_distance)
            if not next_wps:
                return []
            next_wp = next_wps[0]
            distance += next_wp.transform.location.distance(plan[-1][0].transform.location)
            plan.append((next_wp, RoadOption.LANEFOLLOW))

        if direction == 'left':
            option = RoadOption.CHANGELANELEFT
        elif direction == 'right':
            option = RoadOption.CHANGELANERIGHT
        else:
            # ERROR, input value for change must be 'left' or 'right'
            return []

        lane_changes_done = 0
        lane_change_distance = lane_change_distance / lane_changes

        # Lane change
        while lane_changes_done < lane_changes:

            # Move forward
            next_wps = plan[-1][0].next(lane_change_distance)
            if not next_wps:
                return []
            next_wp = next_wps[0]

            # Get the side lane
            if direction == 'left':
                if check and str(next_wp.lane_change) not in ['Left', 'Both']:
                    return []
                side_wp = next_wp.get_left_lane()
            else:
                if check and str(next_wp.lane_change) not in ['Right', 'Both']:
                    return []
                side_wp = next_wp.get_right_lane()

            if not side_wp or side_wp.lane_type != carla.LaneType.Driving:
                return []

            # Update the plan
            plan.append((side_wp, option))
            lane_changes_done += 1

        # Other lane
        distance = 0
        while distance < distance_other_lane:
            next_wps = plan[-1][0].next(step_distance)
            if not next_wps:
                return []
            next_wp = next_wps[0]
            distance += next_wp.transform.location.distance(plan[-1][0].transform.location)
            plan.append((next_wp, RoadOption.LANEFOLLOW))

        return plan
