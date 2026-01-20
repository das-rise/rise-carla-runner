"""
Tools essential for manipulating Carla from the client-side
"""

import carla

from carla_agents.navigation.global_route_planner import GlobalRoutePlanner


def create_route(
    start: carla.Location,
    end: carla.Location,
    town_map: carla.Map,
    sampling_resolution: int,
) -> list:
    """
    Plan a route between two locations in carla

    Args:
        start (carla.Location): start location
        end (carla.Location): end location
        town_map (carla.libcarla.Map): the map along which to plan the route from start to end
        sampling_resolution (int): unclear
    """
    global_route_planner = GlobalRoutePlanner(town_map, sampling_resolution)
    route = global_route_planner.trace_route(start, end)

    return route


def show_route_in_world(world: carla.World, route: list) -> None:
    """
    Show a route in the Carla world

    Args:
        world (carla.World): The world in which to draw the route
        route (list): a list of Carla waypoints
    """

    route_locations = [entry[0].transform.location for entry in route[::10]]

    for location in route_locations:
        world.debug.draw_point(
            location, size=0.15, life_time=0, color=carla.Color(255, 0, 0)
        )

    # world.debug.draw_point(route_locations[0], size=0.19, life_time=0, color=carla.Color(0,255,0))
    # world.debug.draw_point(route_locations[-1], size=0.19, life_time=0, color=carla.Color(255,0,0))


def highlight_location(world: carla.World, loc: carla.Location) -> None:
    """
    Highlight a location in the Carla world

    Args:
        world (carla.World): The world in which to draw the route
        route (list): a list of Carla waypoints
    """
    world.debug.draw_point(loc, size=0.15, life_time=0, color=carla.Color(0, 255, 0))


def draw_origin(world: carla.World, scale: float = 1.0) -> None:
    """
    Draw a coordinate system at the origin of the Carla world.

    Args:
        world (carla.World): The world in which to draw the origin
        scale (float): Scale the size of the origin
    """
    origin = carla.Location(0, 0, 0)
    x_axis = carla.Location(x=scale, y=0, z=0)
    y_axis = carla.Location(x=0, y=scale, z=0)
    z_axis = carla.Location(x=0, y=0, z=scale)

    # Draw X axis in red
    world.debug.draw_arrow(
        origin,
        x_axis,
        thickness=0.1 * scale,
        arrow_size=0.2 * scale,
        color=carla.Color(255, 0, 0),
        life_time=0,
    )

    # Draw Y axis in green
    world.debug.draw_arrow(
        origin,
        y_axis,
        thickness=0.1 * scale,
        arrow_size=0.2 * scale,
        color=carla.Color(0, 255, 0),
        life_time=0,
    )

    # Draw Z axis in blue
    world.debug.draw_arrow(
        origin,
        z_axis,
        thickness=0.1 * scale,
        arrow_size=0.2 * scale,
        color=carla.Color(0, 0, 255),
        life_time=0,
    )
