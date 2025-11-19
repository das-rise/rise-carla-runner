"""
Trajectory data management module.

This module provides the definition of the properties that are available in the trajectory DataFrame.
"""

import polars as pl


class TrajDF(pl.DataFrame):
    def __init__(self):
        self.df = pl.DataFrame(
            schema={
                "frame": pl.Int64,
                "timestamp": pl.Float64,
                "movingobject_id": pl.Int64,
                "dimension_x": pl.Float64,
                "dimension_y": pl.Float64,
                "dimension_z": pl.Float64,
                "position_x": pl.Float64,
                "position_y": pl.Float64,
                "position_z": pl.Float64,
                "orientation_x": pl.Float64,  # roll
                "orientation_y": pl.Float64,  # pitch
                "orientation_z": pl.Float64,  # yaw
                "velocity": pl.List(pl.Float64),  # [vx, vy, vz]
                "acceleration": pl.List(pl.Float64),  # [ax, ay, az]
                "type": pl.Utf8,
                "vehicleclassification_type": pl.Utf8,
                "vehicleclassification_role": pl.Utf8,
            }
        )

    def __call__(self):
        return self.df
