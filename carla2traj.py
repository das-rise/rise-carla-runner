import carla
import polars as pl
from typing import Tuple
from math import pi

from traj2x import Traj2X
from traj import TrajDF


class Carla2Traj:
    """Class to ingest CARLA simulation data into a trajectory DataFrame (TrajDF) suitable for further processing or conversion."""

    def __init__(self, world: carla.World, debug: bool = False):
        self._world = world
        self._debug = debug
        self.df = TrajDF()

        self._first_frame = None

    def process_world_snapshot(self, world_snapshot: carla.WorldSnapshot):
        """Process a CARLA WorldSnapshot and append data to the trajectory DataFrame.
        Args:
            world_snapshot: CARLA WorldSnapshot containing actor states at a specific simulation time.
        """

        timestamp = world_snapshot.timestamp.elapsed_seconds
        frame_number = self._get_frame(world_snapshot)

        # Process each actor snapshot
        for actor_snapshot in world_snapshot:

            movingobject_id_value = actor_snapshot.id
            actor = self._get_actor(movingobject_id_value)

            actor_type = self._get_type_from_carla_actor(actor)
            if actor_type is None:
                continue  # Skip unknown actor types

            bounding_box_extent = self._get_extent_from_carla_bounding_box(
                actor.bounding_box
            )
            dimension_x = bounding_box_extent[0]
            dimension_y = bounding_box_extent[1]
            dimension_z = bounding_box_extent[2]
            # use actor id from snapshot to get bounding box dimensions

            position_x = self._convert_coord_to_osi_x(
                actor_snapshot.get_transform().location.x
            )
            position_y = self._convert_coord_to_osi_y(
                actor_snapshot.get_transform().location.y
            )
            position_z = self._convert_coord_to_osi_z(
                actor_snapshot.get_transform().location.z
            )

            orientation_x = self._convert_carlaRotation_pitch_to_osi(
                actor_snapshot.get_transform().rotation.pitch
            )
            orientation_y = self._convert_carlaRotation_yaw_to_osi(
                actor_snapshot.get_transform().rotation.yaw
            )
            orientation_z = self._convert_carlaRotation_roll_to_osi(
                actor_snapshot.get_transform().rotation.roll
            )

            velocity = list(
                self._convert_carlaVector3D_to_osi(actor_snapshot.get_velocity())
            )
            acceleration = list(
                self._convert_carlaVector3D_to_osi(actor_snapshot.get_acceleration())
            )

            vehicleclassification_type = None
            vehicleclassification_role = None

            # Append data to DataFrame
            new_row = {
                "frame": frame_number,
                "timestamp": timestamp,
                "movingobject_id": movingobject_id_value,
                "dimension_x": dimension_x,
                "dimension_y": dimension_y,
                "dimension_z": dimension_z,
                "position_x": position_x,
                "position_y": position_y,
                "position_z": position_z,
                "orientation_x": orientation_x,
                "orientation_y": orientation_y,
                "orientation_z": orientation_z,
                "velocity": velocity,
                "acceleration": acceleration,
                "type": actor_type or "Other",
                "vehicleclassification_type": vehicleclassification_type or "Other",
                "vehicleclassification_role": vehicleclassification_role or "Other",
            }

            self.df().extend(pl.DataFrame([new_row]))

            if self._debug:
                print(
                    f"[CARLA2TRAJ] Processed actor ID {movingobject_id_value} at timestamp {timestamp}"
                )
                print(new_row["acceleration"])

    def convert(
        self, traj_converter: Traj2X, output_path: str, converter_args: dict = {}
    ):
        """Convert the trajectory DataFrame to another format using the specified converter.
        Args:
            traj_converter(Traj2X): A Traj2X subclass that implements the conversion logic.
            output_path(str): The file path where the converted data will be saved.
            converter_args(dict): Additional arguments to pass to the converter.
        """

        converter = traj_converter(self.df, **converter_args)
        converter(output_path)

    def save(self, filename: str = ""):
        """Save the trajectory DataFrame to a Parquet file.
        Args:
            filename(str): The file path where the DataFrame will be saved. If empty, a default filename with timestamp will be used.
        """
        from datetime import datetime

        datetime_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"traj_{datetime_str}.parquet" if filename == "" else filename
        self.df().write_parquet(filename)
        print(f">Saved trajectory DataFrame to {filename}.<")

    def load(self, filename: str):
        """Load trajectory data from a Parquet file into the trajectory DataFrame.
        Args:
            filename(str): The file path from which to load the DataFrame.
        """
        check_df = TrajDF()()
        loaded_df = pl.read_parquet(filename)
        assert (
            check_df.schema == loaded_df.schema
        ), "Loaded DataFrame schema does not match TrajDF schema (`traj.py`)."
        self.df = loaded_df

    @staticmethod
    def from_file(filename: str):
        """Create a Carla2Traj instance by loading trajectory data from a Parquet file.
        Args:
            filename(str): The file path from which to load the DataFrame.
        Returns:
            Carla2Traj: An instance of Carla2Traj with loaded trajectory data.
        """
        carla2traj = Carla2Traj(world=None)  # world is not needed for loading from file
        carla2traj.load(filename)
        return carla2traj

    def _get_frame(self, world_snapshot: carla.WorldSnapshot) -> int:
        # Calculate frame number in current run by subtracting first frame of this run
        # (Carla has a global frame counter counting from when the simulator was started)
        if self._first_frame is None:
            self._first_frame = world_snapshot.frame
        return world_snapshot.frame - self._first_frame

    def _get_actor(self, actor_id: int) -> carla.Actor:
        try:
            return [
                actor for actor in self._world.get_actors() if actor.id == actor_id
            ][0]
        except IndexError:
            raise ValueError(f"No actor found with ID {actor_id}")

    def _get_extent_from_carla_bounding_box(
        self, bounding_box: carla.BoundingBox
    ) -> Tuple[float, float, float]:
        """Convert CARLA bounding box extent to OSI dimensions (x, y, z).

        Args:
            bounding_box: CARLA bounding box

        Returns:
            Extents in OSI coordinates as (ext_x, ext_y, ext_z)
        """
        extent = bounding_box.extent
        # The bounding box vector in carla is half-extent, so double it
        return tuple(
            2 * abs(coord) for coord in self._convert_carlaVector3D_to_osi(extent)
        )

    def _get_type_from_carla_actor(self, actor: carla.Actor) -> str:
        TYPES = ["Other", "Vehicle", "Pedestrian", "Animal"]
        type_id = actor.type_id
        if type_id.split(".")[0] == "vehicle":
            return "Vehicle"

    def _convert_coord_to_osi_x(self, x: float) -> float:
        return x

    def _convert_coord_to_osi_y(self, y: float) -> float:
        # invert y-axis
        return -y

    def _convert_coord_to_osi_z(self, z: float) -> float:
        return z

    def _convert_carlaVector3D_to_osi(
        self, vector: carla.Vector3D
    ) -> Tuple[float, float, float]:
        x_osi = self._convert_coord_to_osi_x(vector.x)
        y_osi = self._convert_coord_to_osi_y(vector.y)
        z_osi = self._convert_coord_to_osi_z(vector.z)
        return (x_osi, y_osi, z_osi)

    def _convert_carlaRotation_pitch_to_osi(self, pitch: float) -> float:
        # check for reference https://github.com/DLR-TS/Carla-OSI-Service/blob/main/src/carla_osi/Geometry.cpp
        # and reference with https://opensimulationinterface.github.io/osi-antora-generator/asamosi/latest/gen/structosi3_1_1Orientation3d.html
        return pitch * pi / 180.0

    def _convert_carlaRotation_yaw_to_osi(self, yaw: float) -> float:
        # check for reference https://github.com/DLR-TS/Carla-OSI-Service/blob/main/src/carla_osi/Geometry.cpp
        # and reference with https://opensimulationinterface.github.io/osi-antora-generator/asamosi/latest/gen/structosi3_1_1Orientation3d.html
        return yaw * pi / 180.0 * (-1)

    def _convert_carlaRotation_roll_to_osi(self, roll: float) -> float:
        # check for reference https://github.com/DLR-TS/Carla-OSI-Service/blob/main/src/carla_osi/Geometry.cpp
        # and reference with https://opensimulationinterface.github.io/osi-antora-generator/asamosi/latest/gen/structosi3_1_1Orientation3d.html
        return roll * pi / 180.0

    # TODO: add traffic lights


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(
        description="Convert CARLA trajectory data to OmegaPrime-compliant OSI or OpenLabel format",
        epilog="""
Example usage: python carla2traj.py traj.parquet osi 752 0.1.0 '' 'Town01.xodr' -o output.osi
To generate the CARLA trajectory parquet file, please confer the README.md in this repository.
        """,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "parquet_file",
        type=str,
        help="Path to the input parquet file containing trajectory data",
    )
    parser.add_argument(
        "format",
        type=str,
        choices=["osi", "openlabel"],
        help="Output format: 'osi' or 'openlabel'",
    )
    parser.add_argument(
        "arg1",
        type=int,
        help="For OSI: ISO country_code (int), For OpenLabel: dummy argument A",
    )
    parser.add_argument(
        "arg2",
        type=str,
        help="For OSI: version (X.Y.Z), For OpenLabel: dummy argument B",
    )
    parser.add_argument(
        "arg3",
        type=str,
        nargs="?",
        default=None,
        help="For OSI: proj_string (str) (optional for OpenLabel)",
    )
    parser.add_argument(
        "arg4",
        type=str,
        nargs="?",
        default=None,
        help="For OSI: map_reference (str) (optional for OpenLabel)",
    )
    parser.add_argument(
        "-o",
        "--output",
        type=str,
        default=None,
        help="Output file path (default: auto-generated based on format)",
    )

    args = parser.parse_args()

    # Load trajectory data from parquet file
    print(f"Loading trajectory data from {args.parquet_file} ...", end=" ")
    traj = Carla2Traj.from_file(args.parquet_file)
    print(f"Loaded {traj.df.height} frames.")

    # Determine output path
    if args.output is None:
        from datetime import datetime

        datetime_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        if args.format == "osi":
            output_path = f"trajectory_osi_{datetime_str}.osi"
        else:
            output_path = f"trajectory_openlabel_{datetime_str}.json"
    else:
        output_path = args.output

    # Convert based on selected format
    if args.format == "osi":
        if args.arg3 is None or args.arg4 is None:
            parser.error(
                "OSI format requires all 4 arguments: country_code, version, proj_string, map_reference"
            )

        from traj2osi import Traj2OSI  # Assuming this import exists

        converter_args = {
            "country_code": args.arg1,
            "version": args.arg2,
            "proj_string": args.arg3,
            "map_reference": args.arg4,
        }

        print(f"Converting to OSI format with:")
        print(f"  country_code: {args.arg1}")
        print(f"  version: {args.arg2}")
        print(f"  proj_string: {args.arg3}")
        print(f"  map_reference: {args.arg4}")

        traj.convert(Traj2OSI, output_path, converter_args)

    else:  # openlabel
        from traj2openlabel import Traj2OpenLabel  # Assuming this import exists

        converter_args = {"dummy_a": args.arg1, "dummy_b": args.arg2}

        print(f"Converting to OpenLabel format with dummy arguments:")
        print(f"  A: {args.arg1}")
        print(f"  B: {args.arg2}")

        traj.convert(Traj2OpenLabel, output_path, converter_args)

    print(f"Conversion complete! Output saved to: {output_path}")
