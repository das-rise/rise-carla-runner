"""
Statistical analysis module for RiRun simulations.

Provides classes for calculating vehicle trajectory statistics, including
average distance between reference and interpolated positions.

Classes:
    Statistic: Abstract base class for statistical calculations
    Average_Distance_Interpolated: Calculates average positional accuracy
"""

import carla
from typing import Tuple

class Statistic:
    """Interface to keep track of vehicular dynamic statistics"""

    def add(self, data=Tuple) -> None:
        """Add data to the statistic."""
        return

    def evaluate(self) -> None:
        """Produce the result of the statistic."""


class Average_Distance_Interpolated(Statistic):
    """
    Calculate the average distance between the reference and interpolated position
    for all timestamps in a trajectory.
    """

    def __init__(self) -> None:
        self._data = []

    def add(self, data=Tuple) -> None:
        """
        Add a point to the statistic.

        Args:
            data (Tuple): Reference time (seconds, from raw trajectory), reference location (carla.Location, from raw trajectory), reference_speed (carla.Vector3D km/h, from raw trajectory), simulation time (seconds), simulation location (carla.Location)
        """
        self._data.append(data)

    def evaluate(self) -> float:
        """Produce the average distance.

        Returns:
            float: The average Euclidean distance between the reference locations
                   and their corresponding interpolated locations across all timestamps.
        """
        distances = []
        for (
            reference_time,
            reference_location,
            reference_speed,
            sim_time,
            sim_location,
        ) in self._data:
            interpolated_location = self.interpolate_location(
                reference_time, reference_speed, sim_time, sim_location
            )
            distances.append(interpolated_location.distance(reference_location))

        return sum(distances) / len(distances)

    def interpolate_location(
        self,
        reference_time: float,
        reference_speed: carla.Vector3D,
        sim_time: float,
        sim_location: carla.Location,
    ) -> carla.Location:
        """
        Interpolate the location at the reference time (seconds) using the simulation time (seconds),
        simulation location (meters) and simulation speed (in km/h).

        Args:
            reference_time (float): The reference timestamp in seconds.
            reference_speed (carla.Vector3D): The reference speed vector in km/h.
            sim_time (float): The simulation timestamp in seconds.
            sim_location (carla.Location): The location at the simulation time in meters.

        Returns:
            carla.Location: The interpolated location in meters.
        """
        time_factor = (
            reference_time - sim_time
        ) / 3.6  # time_factor includes km/h to m/s conversion
        x_interpolated = sim_location.x + time_factor * reference_speed.x
        y_interpolated = sim_location.y + time_factor * reference_speed.y
        z_interpolated = sim_location.z + time_factor * reference_speed.z

        return carla.Location(x_interpolated, y_interpolated, z_interpolated)
