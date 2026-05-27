import xml.etree.ElementTree as ET

import carla

from rirun.kinetics.trajectory import CarlaTrajectoryPoint, Trajectory


def process_trajectory_file(
    trajectory_filepath,
    heading_interpolation_mode: str = "straight",
    force_heading_interpolation: bool = False,
    mapmatch: bool = False,
    world: carla.World = None,
    offset: tuple = None,
) -> Trajectory:
    """Process a trajectory file and return a Trajectory object.

    A trajectory file is expected to be a CSV file containing lines with three to five comma-separated values:
    x-coordinate (float), y-coordinate (float), timestamp (float) [and angle (degrees)] [and speed (km/h)]. The function reads the file,
    converts the coordinates to the Carla coordinate system, and generates heading and speed information
    for the trajectory.

    Args:
        trajectory_filepath (str): The filepath to the trajectory file.
        offset (tuple, optional): A (x_offset, y_offset) tuple to add to all coordinates before applying the Carla coordinate conversion. Use this when trajectory coordinates were recorded with a subtracted origin (e.g. savant2rirun --offset). Defaults to None.
        heading_interpolation_mode (str): The mode to use for heading generation. "straight" (default): Calculate headings based on the angle between the current and the next trajectory point. "spline": Create a spline curve over the trajectory to generate heading angles.
        force_heading_interpolation (bool): If True, forces heading interpolation even if the trajectory file contains heading information. Default is False.
        mapmatch (bool): If True, project trajectory points onto the road network using the Carla map from ``world``. Default is False.
        world (carla.World): The Carla world used for map-matching. Must be provided when ``mapmatch`` is True. Default is None.

    Returns:
        Trajectory: A processed Trajectory object.
    """
    bare_route = []
    firstline = True
    num_fields = 0
    with open(trajectory_filepath) as openf:
        for line in openf:
            if firstline:
                num_fields = len(line.split(","))
                if num_fields < 3 or num_fields > 5:
                    raise ValueError(
                        f"Trajectory file {trajectory_filepath} has invalid number of fields ({num_fields}). Expected 3 to 5."
                    )
                firstline = False
                continue
            line_readings = tuple([float(reading) for reading in line.split(",")])
            bare_route.append(line_readings)
    trajectory = Trajectory(bare_route)
    if offset is not None:
        trajectory.apply_offset(offset[0], offset[1])
    trajectory.apply_carla_coord_conversion()

    if mapmatch:
        if world is None:
            raise ValueError("World must be provided for map-matching.")
        trajectory = _mapmatch_trajectory(trajectory, world)

    trajectory.gen_speeds()
    if num_fields == 3 or force_heading_interpolation:
        trajectory.gen_headings(mode=heading_interpolation_mode)

    return trajectory


def _mapmatch_trajectory(trajectory: Trajectory, world: carla.World) -> Trajectory:
    """Map-match a trajectory to the road network using the Carla map extracted from the world.

    Args:
        trajectory (Trajectory): The input trajectory to be map-matched.
        world (carla.World): The Carla world used for map-matching.

    Returns:
        Trajectory: A new Trajectory object with points adjusted to align with the road network.
    """

    ADAPTIVE_MODE = True
    DISTANCE_OFFSET = 1.5  # should be roughly half the width of a car

    carla_map = world.get_map()
    projected_route = []
    for trajectory_point in trajectory.get_trajectory():
        center_wp = carla_map.get_waypoint(
            carla.Location(x=trajectory_point.x, y=trajectory_point.y, z=0)
        )
        if ADAPTIVE_MODE:
            center_loc = center_wp.transform.location
            lane_width = center_wp.lane_width
            traj_point_loc = carla.Location(
                x=trajectory_point.x, y=trajectory_point.y, z=0
            )
            distance_vector = center_loc - traj_point_loc
            distance_threshold = max(lane_width / 2 - DISTANCE_OFFSET, 0)

            next_lane_candidate = (
                center_loc - distance_vector.make_unit_vector() * lane_width
            )  # at this location, a neighboring lane could be located

            # perform mapmatching only if the car is too close to the lane boundary and there is no lane for the car to cross into
            if (
                distance_vector.length()
                >= distance_threshold  # car too close to lane shoulder
                and not carla_map.get_waypoint(  # function returns none if there is no lane at that waypoint
                    next_lane_candidate, project_to_road=False
                )
            ):
                # project the actual trajectory point to DISTANCE_THRESHOLD meters from the center waypoint in the direction of the trajectory point
                mapmatched_loc = (
                    center_loc - distance_threshold * distance_vector.make_unit_vector()
                )
                projected_route.append(
                    (
                        mapmatched_loc.x,
                        mapmatched_loc.y,
                        trajectory_point.time,
                        center_wp.transform.rotation.yaw,
                    )
                )
            else:
                projected_route.append(
                    (
                        trajectory_point.x,
                        trajectory_point.y,
                        trajectory_point.time,
                        trajectory_point.heading,
                    )
                )

        else:
            projected_route.append(
                (
                    center_wp.transform.location.x,
                    center_wp.transform.location.y,
                    trajectory_point.time,
                    center_wp.transform.rotation.yaw,
                )
            )
    new_trajectory = Trajectory(projected_route)
    return new_trajectory


def get_first_xml_waypoint(xml_path: str) -> CarlaTrajectoryPoint:
    """
    Parse the first waypoint from an XML file and convert it to a CarlaTrajectoryPoint.

    Args:
        xml_path (str): Path to the XML file containing <route> with <waypoint> elements.

    Returns:
        CarlaTrajectoryPoint: NamedTuple with a carla.Transform, default time=0.0, speed=0.0.
    """
    tree = ET.parse(xml_path)
    root = tree.getroot()
    wp = root.find("waypoint")

    x = float(wp.get("x"))
    y = float(wp.get("y"))
    yaw = float(wp.get("yaw"))

    transform = carla.Transform(
        carla.Location(x=x, y=y, z=0.0), carla.Rotation(pitch=0.0, yaw=yaw, roll=0.0)
    )

    return CarlaTrajectoryPoint(transform=transform, time=0.0, speed=0.0)
