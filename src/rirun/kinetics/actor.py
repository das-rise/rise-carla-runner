import carla
from kinetics.trajectory import CarlaTrajectoryPoint


class Actor:
    """Abstract class for actors"""

    def __init__(self, world: carla.World, name: str) -> None:
        self.name = name
        self._world = world
        self._spawned = False
        self._destroyed = False

    def step(self, simulation_time: float) -> None:
        """Execute one step of the simulation"""
        raise NotImplementedError

    def spawn(self, transform: carla.Transform = None) -> None:
        """Spawn the actor at the given transform"""
        raise NotImplementedError

    def destroy(self) -> None:
        """Destroy the actor"""
        raise NotImplementedError

    def advance_trajectory(self) -> None:
        """Advance the Actor's trajectory"""
        raise NotImplementedError

    def is_spawned(self) -> bool:
        """Check if the actor has been spawned"""
        raise NotImplementedError

    def get_actor(self) -> carla.Actor:
        """Return carla.Actor of this Actor"""
        raise NotImplementedError

    def get_current_trajectory_point(self) -> CarlaTrajectoryPoint:
        """Get current trajectory point of this Actor"""
        raise NotImplementedError

    def get_xml_route(self) -> str:
        """Get xml route for PCLA agents"""
        raise NotImplementedError
