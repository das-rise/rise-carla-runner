"""
Tools essential for manipulating Carla from the client-side
"""

import carla
import logging
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


def _load_from_opendrive(client: carla.Client, xodr_filepath: str) -> None:
    """
    Load a CARLA world from an OpenDRIVE (.xodr) file.

    Args:
        client (carla.Client): The CARLA client instance.
        xodr_filepath (str): Path to the OpenDRIVE file.
    """
    xodr_string = ""
    with open(xodr_filepath, "r") as openf:
        xodr_string = openf.read()
    logging.info(f"Loading world from OpenDrive file {xodr_filepath}...")
    client.generate_opendrive_world(
        xodr_string,
        carla.OpendriveGenerationParameters(
            wall_height=0, smooth_junctions=False, additional_width=0
        ),
    )


def _load_from_carla_map(client: carla.Client, map_name: str) -> None:
    """
    Load a CARLA world directly from a CARLA map.

    Args:
        client (carla.Client): The CARLA client instance.
        map_path (str): Path to the CARLA map.
    """
    logging.info(f"Loading world from CARLA map {map_name}...")
    client.load_world(map_name)


def load_map(client: carla.Client, map_file: str) -> None:
    """
    Load a CARLA world from either an OpenDRIVE (.xodr) file or a CARLA map (.snet) file.

    Args:
        client (carla.Client): The CARLA client instance.
        map_path (str): Path to the OpenDRIVE or CARLA map file.
    """
    if map_file.endswith(".xodr"):
        _load_from_opendrive(client, map_file)
    else:
        _load_from_carla_map(client, map_file)
