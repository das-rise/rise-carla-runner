from traj import TrajDF
from polars import DataFrame
from typing import Generator, Any


class Traj2X:
    """Base class for trajectory format converters."""

    def __init__(self, traj_df: TrajDF):
        self._traj_df = traj_df

    def __call__(self, output_path: str):
        pass

    def _traverse_frames(self) -> Generator[DataFrame, Any, None]:
        # Generator to traverse the trajectory DataFrame frame by frame
        for frame, frame_data in self._traj_df.sort("frame").group_by("frame"):
            yield frame_data
