import os
import sys
import carla

# PCLA import #

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
PCLA_ROOT = "/".join(CURRENT_DIR.split("/")[:-3]) + "/PCLA"
sys.path.append(PCLA_ROOT)
import PCLA

# Constants

TOWN_XODR = "Town01.xodr"

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
route_range = [0, 30]
route_waypoints = PCLA.location_to_waypoint(
    client, spawn_points[route_range[0]].location, spawn_points[route_range[1]].location
)

xml_path = "agent_route.xml"
PCLA.route_maker(route_waypoints, xml_path)
print(f"Saved xml route to {xml_path}")