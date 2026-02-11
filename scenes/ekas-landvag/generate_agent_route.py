import os
import sys
import carla

# PCLA import #

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
PCLA_ROOT = "/".join(CURRENT_DIR.split("/")[:-3]) + "/PCLA"
sys.path.append(PCLA_ROOT)
import PCLA

# Constants

TOWN_XODR = "ekas-bash.xodr"

# Helpers


def load_from_opendrive(client: carla.Client, xodr_filepath: str) -> None:
    """
    Load a CARLA world from an OpenDRIVE (.xodr) file.
    Args:
    client (carla.Client): The CARLA client instance.
    """
    xodr_string = ""
    with open(xodr_filepath, "r") as openf:
        xodr_string = openf.read()
    client.generate_opendrive_world(
        xodr_string,
        carla.OpendriveGenerationParameters(
            wall_height=0, smooth_junctions=True, additional_width=2
        ),
    )


client = carla.Client("localhost", 2000)
client.set_timeout(10.0)
load_from_opendrive(client, TOWN_XODR)
world = client.get_world()

spawn_points = world.get_map().get_spawn_points()
print(f"spawn points: {[(s.location.x, s.location.y) for s in spawn_points]}")

ROUTE_START = 1
ROUTE_END = 29
try:
    route_waypoints = PCLA.location_to_waypoint(
        client, spawn_points[ROUTE_START].location, spawn_points[ROUTE_END].location
    )
except IndexError:
    print(
        f"Invalid route start or end. Number of waypoints: {len(spawn_points)}, while route_start={route_start}, route_end={route_end}"
    )
    sys.exit(1)

xml_path = "agent_route.xml"
PCLA.route_maker(route_waypoints, xml_path)
print(f"Saved xml route to {xml_path}")
