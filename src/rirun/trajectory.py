import carla
from typing import Tuple, List, Generator, NamedTuple
from math import sqrt, acos, pi
import carla_tools


class CarlaTrajectoryPoint(NamedTuple):
    transform: carla.Transform
    time: float
    speed: float


class TrajectoryPoint(NamedTuple):
    x: float
    y: float
    time: float
    heading: float
    speed: float


class Trajectory:

    def __init__(self, bare_route: List[Tuple]) -> None:
        """
        Create an instance of Trajectory containing a bare route.

        Args:
            bare_route (List[Tuple):   A route passed as a list of (x=easting, y=northing, time s, [heading deg], [speed km/h]) tuples
        """
        self._x = []
        self._y = []
        self._t = []
        self._h = []
        self._s = []
        self._len_trajectory = 0

        num_fields = len(bare_route[0])

        if num_fields == 3:
            # No headings, no speeds
            self._has_headings = False
            self._has_speeds = False

            for x, y, t in bare_route:
                self._x.append(x)
                self._y.append(y)
                self._t.append(t)
                self._len_trajectory += 1

        elif num_fields == 4:
            # Has headings, no speeds
            self._has_headings = True
            self._has_speeds = False

            for x, y, t, h in bare_route:
                self._x.append(x)
                self._y.append(y)
                self._t.append(t)
                self._h.append(h)
                self._len_trajectory += 1

        elif num_fields == 5:
            # Has headings, has speeds
            self._has_headings = True
            self._has_speeds = True

            for x, y, t, h, s in bare_route:
                self._x.append(x)
                self._y.append(y)
                self._t.append(t)
                self._h.append(h)
                self._s.append(s)
                self._len_trajectory += 1

        else:
            raise ValueError(
                f"Bare route has invalid number of fields ({num_fields}). Expected 3, 4 or 5."
            )

    def get_trajectory(self) -> Generator[TrajectoryPoint, None, None]:
        """
        Return a generator over the entire trajectory, consisting of named tuples
        """
        for i in range(self._len_trajectory):
            yield TrajectoryPoint(
                self._x[i],
                self._y[i],
                self._t[i],
                self._h[i] if self._has_headings else None,
                self._s[i] if self._has_speeds else None,
            )

    def get_carla_trajectory(self) -> Generator[CarlaTrajectoryPoint, None, None]:
        """
        Return a generator over the entire trajectory, consisting of carla.Transform, timestamps, and speeds
        """
        for i in range(self._len_trajectory):
            yield CarlaTrajectoryPoint(
                carla.Transform(
                    carla.Location(self._x[i], self._y[i], 0),
                    carla.Rotation(0, self._h[i] if self._has_headings else None, 0),
                ),
                self._t[i],
                self._s[i] if self._has_speeds else None,
            )

    def apply_carla_coord_conversion(self) -> None:
        """
        Apply the shift from y -> -y (inverting the northing) that is required for correct translation to Carla.
        """
        self._y = [-y for y in self._y]

    def add_headings(self, headings: List[carla.Rotation]) -> None:
        """
        Add headings to a trajectory.

        Args:
            headings (List[carla.Rotation]): The current heading in the trajectory
        """
        assert (
            len(headings) == self._len_trajectory
        ), f"Number of headings [{len(headings)}] does not equal length of trajectory [{self._len_trajectory}]"
        self._h = []
        for h in headings:
            self._h.append(h)
        self._has_headings = True

    def add_speeds(self, speeds: List[float]) -> None:
        """
        Add speeds to a trajectory.

        Args:
            speeds (List[float]): The current speed in the trajectory in km/h
        """
        assert (
            len(speeds) == self._len_trajectory
        ), f"Number of headings [{len(speeds)}] does not equal length of trajectory [{self._len_trajectory}]"
        self._s = []
        for s in speeds:
            self._s.append(s)
        self._has_speeds = True

    def gen_speeds(self) -> None:
        """
        Generate the speed values in km/h by calculating them from the bare route.
        """
        assert not self._has_speeds, "Already has speeds, will not overwrite."

        def speed_kmh(
            x1: float, y1: float, t1: float, x2: float, y2: float, t2: float
        ) -> float:
            return sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2) / (t2 - t1) * 3.6

        speeds = []
        speeds.append(None)
        traj_curr = self.get_trajectory()
        traj_next = self.get_trajectory()

        traj_point_curr = None
        traj_point_next = None

        while True:
            try:
                traj_point_next = next(traj_next)
                if traj_point_curr is None:
                    traj_point_curr = next(traj_curr)
                    continue

                speed = speed_kmh(
                    traj_point_curr.x,
                    traj_point_curr.y,
                    traj_point_curr.time,
                    traj_point_next.x,
                    traj_point_next.y,
                    traj_point_next.time,
                )
                speeds.append(speed)
                traj_point_curr = next(traj_curr)

            except StopIteration:
                break

        speeds[0] = speeds[1]

        self.add_speeds(speeds)

    def gen_headings(self) -> None:
        """
        Generate the headings by calculating them from the bare route
        """
        assert not self._has_headings, "Already has headings, will not overwrite."

        def get_angle(x1: float, y1: float, x2: float, y2: float) -> float:
            """
            Calculate the yaw (in Carla lingo) as the angle in the plane relative to the x-axis
            """
            vector = [x2 - x1, y2 - y1]
            vector_len = sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
            unit = [1, 0]
            angle_rad = acos((vector[0] * unit[0] + vector[1] * unit[1]) / vector_len)
            angle_deg = angle_rad * 180 / pi
            if vector[1] < 0:  # correct for vectors in the left half plane
                angle_deg = 360 - angle_deg
            return angle_deg

        headings = []
        headings.append(None)
        traj_curr = self.get_trajectory()
        traj_next = self.get_trajectory()

        traj_point_curr = None
        traj_point_next = None

        placeholder_is_active = False
        placeholder = "placeholder"
        while True:
            try:
                traj_point_next = next(traj_next)
                if traj_point_curr is None:
                    pass

                elif (
                    traj_point_curr.x == traj_point_next.x
                    and traj_point_curr.y == traj_point_next.y
                ):
                    # in this case, the vehicle is stationary and we set a placeholder
                    headings.append(placeholder)
                    placeholder_is_active = True

                else:
                    heading = get_angle(
                        traj_point_curr.x,
                        traj_point_curr.y,
                        traj_point_next.x,
                        traj_point_next.y,
                    )
                    headings.append(heading)

                    if placeholder_is_active:
                        # that means that the vehicle was stationary until now. In that case, replace all headings of the just-ended stationary phase with the current heading
                        for i, h in enumerate(headings):
                            if h == placeholder:
                                headings[i] = heading
                        placeholder_is_active = False

                traj_point_curr = next(traj_curr)

            except StopIteration:
                break

        headings[0] = headings[1]
        self.add_headings(headings)

    def plot(self, world: carla.World) -> None:
        """
        Visualize the trajectory in the world

        Args:
            world (carla.World):    The Carla world in which to plot the trajectory.
        """
        for trafo, _, _ in self.get_carla_trajectory():
            loc = trafo.location
            loc.z += 0.2
            carla_tools.highlight_location(world, loc)
