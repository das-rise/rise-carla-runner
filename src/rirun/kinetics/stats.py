"""
Statistical analysis module for RiRun simulations.

Provides classes for calculating vehicle trajectory statistics, including
average distance between reference and interpolated positions.

Classes:
    Statistic: Abstract base class for statistical calculations
    Average_Distance_Interpolated: Calculates average positional accuracy
"""

from rirun.kinetics.trajectory import CarlaTrajectoryPoint


class Statistic:
    """Interface to keep track of vehicular dynamic statistics"""

    def add(self, data) -> None:
        """Add data to the statistic."""
        return

    def evaluate(self) -> None:
        """Produce the result of the statistic."""


class Average_Distance_True(Statistic):
    """
    Calculate the average distance between the reference and true position in a trajectory.

    This is done by comparing the position of the vehicle at each tick with the reference position at the same timestamp.
    The reference position is obtained from the raw trajectory, while the true position is obtained from the simulation measurements.
    """

    def __init__(self) -> None:
        self._reference_trajectory = []
        self._measurements = []

    def add(self, data) -> None:
        """
        Add a point to the statistic.

        Args:
            data (Tuple): Either (CarlaTrajectoryPoint) or (CurrentTransform, sim_time)
        """
        if isinstance(data, CarlaTrajectoryPoint):
            loc = data.transform.location
            sim_time = data.time
            self._reference_trajectory.append((loc, sim_time))
        elif len(data) == 2:
            loc = data[0].location
            sim_time = data[1]
            self._measurements.append((loc, sim_time))
        else:
            raise ValueError(
                f"Invalid data format for Average_Distance_True statistic. Expected either (CarlaTrajectoryPoint) or (CurrentTransform, sim_time), got {data}"
            )

    def evaluate(self) -> float:
        """Produce the average distance.

        Returns:
            float: The average Euclidean distance between the reference locations
                   and their corresponding true locations across all timestamps.
        """

        distances = []
        for measurement_loc, measurement_time in self._measurements:
            # find the reference location with the closest timestamp to the measurement
            closest_reference = min(
                self._reference_trajectory,
                key=lambda ref: abs(ref[1] - measurement_time),
            )
            distances.append(measurement_loc.distance(closest_reference[0]))

        return sum(distances) / len(distances)
