"""
Tools essential for manipulating Carla from the client-side
"""

import logging

import carla

from rirun.carla_agents.navigation.global_route_planner import GlobalRoutePlanner


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
        loc (carla.Location): a location to highlight
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


def _load_from_opendrive(client: carla.Client, xodr_filepath: str) -> carla.World:
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
    return client.generate_opendrive_world(
        xodr_string,
        carla.OpendriveGenerationParameters(
            wall_height=0, smooth_junctions=False, additional_width=0
        ),
    )


def _load_from_carla_map(client: carla.Client, map_name: str) -> carla.World:
    """
    Load a CARLA world directly from a CARLA map.

    Args:
        client (carla.Client): The CARLA client instance.
        map_path (str): Path to the CARLA map.
    """
    logging.info(f"Loading world from CARLA map {map_name}...")
    try:
        return client.load_world(map_name)
    except RuntimeError:
        logging.error(f"Failed to load CARLA map {map_name}")
        logging.error(
            "Ensure that the map name is correct and the map is available server-side."
        )
        raise


def load_map(client: carla.Client, map_file: str) -> carla.World:
    """
    Load a CARLA world from either an OpenDRIVE (.xodr) file or a CARLA map (.snet) file.

    Args:
        client (carla.Client): The CARLA client instance.
        map_file (str): Path to the OpenDRIVE or CARLA map file.
    """
    if map_file.endswith(".xodr"):
        return _load_from_opendrive(client, map_file)
    else:
        return _load_from_carla_map(client, map_file)


class OpenDriveSpeedProvider:
    """
    Class for parsing speed information from an OpenDRIVE file and providing speed limits for given locations.
    """

    _xml_tree = None
    _map = None
    _previous_speed_limit = None

    def __init__(self, xodr_filepath: str, map: carla.Map):
        """Initialize the OpenDriveSpeedProvider by parsing the OpenDRIVE file.
        Args:
            xodr_filepath (str): Path to the OpenDRIVE file to parse.
        """
        self._map = map
        import xml.etree.ElementTree as ET

        self._xml_tree = ET.parse(xodr_filepath).getroot()

    def get_speed_limit(self, location: carla.Location) -> float:
        """Get the speed limit (m/s) at a given location by finding the corresponding position in the OpenDRIVE file.
        Args:
            location (carla.Location): The location for which to get the speed limit.

        Returns:
            float: The speed limit at the given location in m/s, or None if no speed limit is found.
        """
        waypoint = self._map.get_waypoint(location, project_to_road=True)

        road_id = waypoint.road_id
        s = waypoint.s
        lane_id = waypoint.lane_id

        for road in self._xml_tree.findall("road"):
            if road.get("id") == str(road_id):
                for lane_section in road.findall("lanes/laneSection"):
                    if float(lane_section.get("s")) <= s:
                        for lanetype in lane_section:  # lanetype = left, center, right
                            for lane in lanetype.findall("lane"):
                                if lane.get("id") == str(lane_id):
                                    speed_element = lane.find("speed")
                                    if speed_element is not None:
                                        speed_limit_ms = float(speed_element.get("max"))
                                        self._previous_speed_limit = speed_limit_ms
                                        return speed_limit_ms

        return self._previous_speed_limit or 0
