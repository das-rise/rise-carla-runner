from typing import Tuple, Union
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


def parse_openlabel_file(
    filepath: str, outpath: str, start_frame: int = None, end_frame: int = None
) -> None:
    from math import pi
    import json
    """
    Parse an OpenLabel file created by SAVANT (https://github.com/RI-SE/SAVANT/tree/main/schema) to extract trajectory files.

    Parses the given OpenLabel .json file, optionally between the given frames, 
    and generates trajectory files for every vehicle.

    Args:

        filepath (str): Path to the OpenLabel .json file.
        outpath (str): Directory where the generated trajectory files will be saved.
        start_frame (int, optional): The starting frame number to parse. Defaults to None (start from the beginning).
        end_frame (int, optional): The ending frame number to parse. Defaults to None (parse until the end).
    """

    file = open(filepath, "r")
    data = json.load(file)

    def _get_vehicle_type(id: int):
        return data["openlabel"]["objects"][str(id)]["type"]

    def _get_frame(frame: int):
        return data["openlabel"]["frames"][str(frame)]

    def _get_time(frame_id: int, framerate: int = 30):
        """Get the timestamp for a given frame ID based on the framerate."""
        return frame_id / framerate

    def _parse_bounding_box(
        frame: Union[int, dict], object_id: int
    ) -> Tuple[float, float, float]:
        """Parse a bounding box from the OpenLabel data for a given frame and object ID.

        Returns:
            A tuple containing the x and y coordinates of the bounding box center, and the angle of the bounding box in degrees.
        """
        frame_data = _get_frame(frame) if isinstance(frame, int) else frame
        object_data = frame_data["objects"][str(object_id)]["object_data"]
        bb = object_data["rbbox"][0]["val"]
        x, y, _, _, angle = bb
        angle = angle * 180 / pi  # Convert angle from radians to degrees
        return x, y, angle

    def _iter_frames(start_frame: int = None, end_frame: int = None):
        """Iterate over all frames in the OpenLabel data."""
        frames = data["openlabel"]["frames"]
        for frame_id in sorted(frames.keys(), key=int):
            if (start_frame is not None and int(frame_id) < start_frame) or (
                end_frame is not None and int(frame_id) > end_frame
            ):
                continue
            yield frame_id, frames[frame_id]

    def _iter_vehicles_in_frame(frame: Union[int, dict]):
        """Iterate over all vehicles in a given frame.

        Yields:
            A tuple containing the object ID, bounding box data, and vehicle type for each vehicle in
        """
        frame_data = _get_frame(frame) if isinstance(frame, int) else frame
        for object_id in frame_data["objects"].keys():
            yield object_id, _parse_bounding_box(
                frame_data, object_id
            ), _get_vehicle_type(object_id)

    def _write_vehicle_trajectory(vehicle_id: int, vehicle: dict, output_dir: str):
        """Write the trajectory of a vehicle to a CSV file."""
        import os

        vehicle_type = vehicle["type"]
        trajectory = vehicle["trajectory"]

        output_path = os.path.join(
            output_dir, f"vehicle_{vehicle_id}_{vehicle_type}.csv"
        )

        if not os.path.exists(output_dir):
            os.makedirs(output_dir)

        with open(output_path, "w", newline="") as csvfile:
            print("x_loc [m],y_loc [m],Time [s],Heading [deg]", file=csvfile)
            for point in trajectory:
                print(*point, sep=",", file=csvfile)

    vehicles = dict()

    for frame_id, frame_data in _iter_frames(start_frame, end_frame):
        for id, bbox, vehicle_type in _iter_vehicles_in_frame(frame_data):
            x, y, angle = bbox
            timestamp = _get_time(int(frame_id))
            trajectory_point = (x, y, timestamp, angle)
            if id not in vehicles:
                vehicles[id] = {
                    "type": vehicle_type,
                    "trajectory": [trajectory_point],
                    "frames": [frame_id],
                }
            else:
                vehicles[id]["trajectory"].append(trajectory_point)
                vehicles[id]["frames"].append(frame_id)

    for vehicle_id, vehicle in vehicles.items():
        _write_vehicle_trajectory(vehicle_id, vehicle, outpath)

    file.close()


if __name__ == "__main__":

    import argparse
    from rich_argparse import RichHelpFormatter

    MODES = ["openlabel-parse"]

    parser = argparse.ArgumentParser(
        prog="Route Tools",
        description="Convert trajectories for use with RIRUN.",
        formatter_class=RichHelpFormatter,
    )

    parser.add_argument(
        "mode",
        choices=MODES,
        type=str,
        help=f"The mode in which to run the script. Choose from {MODES}.",
    )

    parser.add_argument(
        "filepath",
        type=str,
        help="The filepath to the OpenLabel file to process.",
    )

    parser.add_argument(
        "outpath",
        type=str,
        help="The directory where the generated trajectory files will be saved.",
    )

    parser.add_argument(
        "--start_frame",
        type=int,
        default=None,
        help="The starting frame number to parse. Defaults to None (start from the beginning).",
    )

    parser.add_argument(
        "--end_frame",
        type=int,
        default=None,
        help="The ending frame number to parse. Defaults to None (parse until the end).",
    )

    args = parser.parse_args()

    if args.mode == "openlabel-parse":
        parse_openlabel_file(
            args.filepath, args.outpath, args.start_frame, args.end_frame
        )
