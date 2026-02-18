from rirun.kinetics.trajectory import Trajectory, CarlaTrajectoryPoint
import xml.etree.ElementTree as ET
import carla


def process_trajectory_file(trajectory_filepath) -> Trajectory:
    """Process a trajectory file and return a Trajectory object.

    A trajectory file is expected to be a CSV file containing lines with three to five comma-separated values:
    x-coordinate (float), y-coordinate (float), timestamp (float) [and angle (degrees)] [and speed (km/h)]. The function reads the file,
    converts the coordinates to the Carla coordinate system, and generates heading and speed information
    for the trajectory.

    Args:
        trajectory_filepath (str): The filepath to the trajectory file.

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
    trajectory.apply_carla_coord_conversion()
    if num_fields == 3:
        trajectory.gen_speeds()
        trajectory.gen_headings()
    elif num_fields == 4:
        trajectory.gen_speeds()

    return trajectory


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
