import argparse
from datetime import datetime
import os
import time
from rich_argparse import RichHelpFormatter
import signal
import sys
import logging
import subprocess
import carla
from rirun.kinetics.vehicle_tools import Vehicle
from rirun.kinetics.movement import PCLA_Movement
from rirun.kinetics.trajectory_utils import process_trajectory_file
from rirun.utils.video_tools import StreamingCamera
from rirun.utils.spinner import Spinner
from rirun.utils.bling import rirun
import math
from rirun.kinetics.stats import Average_Distance_Interpolated
from rirun.PCLA.PCLA_agents import PCLA_Agent, check_agent_env
from traj_convert.carla2traj import Carla2Traj
from rirun.utils.carla_tools import load_map

# Helper functions


def cleanup(sig: signal.Signals) -> None:
    """
    Perform cleanup and log the received signal
    """
    logging.warning(f"Execution aborted through signal {sig}.")
    # Add any cleanup code here


def signal_handler(sig: signal.Signals, frame) -> None:
    """
    Handle signals send to this script and exit
    """
    cleanup(sig)
    sys.exit(1)


def execute(bash_str: str) -> None:
    """
    Execute given string as process.
    Args:
        bash_str (str): The command to execute.
    """
    try:
        logging.info(f"Calling '{bash_str}'")
        bash_str_list = bash_str.split(" ")
        result = subprocess.run(
            bash_str_list, check=True, capture_output=True, text=True
        )
        logging.info(f"Output:\n{result.stdout}")
    except subprocess.CalledProcessError as e:
        logging.error(f"Error when running: {e}")


def parse_arguments():
    parser = argparse.ArgumentParser(
        prog="RiRun - RISE Carla Scene Runner",
        description="Run a scene in Carla from a file containing vehicle trajectories + an OpenDrive/Carla map file",
        formatter_class=RichHelpFormatter,
    )
    parser.add_argument(
        "simulation_duration", type=float, help="Duration of the simulation in seconds."
    )
    parser.add_argument(
        "map_filepath",
        type=str,
        help="Path to OpenDrive (*.xodr) or Carla map (*.snet) file.",
    )
    parser.add_argument(
        "trajectory_filepaths",
        type=str,
        nargs="+",
        help="Path(s) to trajectories csv file. First path passed belongs to potential ego vehicle.",
    )
    parser.add_argument(
        "--camera_output_dir",
        type=str,
        default="output/camera",
        help="Where to store the output files. Default: output/camera",
    )
    parser.add_argument(
        "--traj_output_dir",
        type=str,
        default="output/traj",
        help="Where to store the output files. Default: output/traj",
    )
    parser.add_argument(
        "--timestep", type=float, help="Set time step in seconds. Default: 0.01"
    )
    parser.add_argument(
        "--asynchronous", action="store_true", help="Turn off synchronous mode."
    )
    parser.add_argument(
        "--carla_address", type=str, help="IP of Carla server. Default: localhost:2000"
    )
    parser.add_argument(
        "--movement",
        choices=["pid", "teleport"],
        default="teleport",
        help="How vehicles move along trajectories (default: teleport).",
    )
    parser.add_argument(
        "--camera_mode",
        choices=["ego", "overhead"],
        default="overhead",
        help="Attach camera to ego vehicle or choose overhead camera.",
    )
    parser.add_argument(
        "--overhead_camera_position",
        type=float,
        nargs=3,
        metavar=("X", "Y", "Z"),
        help="Start camera at given coordinates (overhead view).",
    )
    parser.add_argument(
        "--pcla_agent",
        type=str,
        choices=[agent.value for agent in PCLA_Agent],
        help="Add a PCLA autonomous driving agent to the simulation.",
    )
    parser.add_argument(
        "--pcla_route",
        type=str,
        help="Path to XML file containing the route for the PCLA agent (required if --pcla_agent is specified).",
    )
    parser.add_argument(
        "--offset_time",
        type=float,
        default=0.0,
        help="Time offset in seconds to start the simulation at (default: 0.0).",
    )

    args = parser.parse_args()

    # Validate that pcla_route is provided if pcla_agent is specified
    if args.pcla_agent and not args.pcla_route:
        parser.error("--pcla_route is required when --pcla_agent is specified")
    if args.pcla_route and not args.pcla_agent:
        parser.error("--pcla_agent is required when --pcla_route is specified")

    # Validate required environment variables based on chosen PCLA agent
    if args.pcla_agent:
        try:
            check_agent_env(PCLA_Agent(args.pcla_agent))
        except RuntimeError as e:
            parser.error(str(e))

    return args


def main() -> None:
    args = parse_arguments()

    if not args.carla_address:
        carla_host = "localhost"
        carla_ip = 2000
    else:
        carla_host = args.carla_address.split(":")[0]
        carla_ip = int(args.carla_address.split(":")[1])

    rirun()

    spinner = Spinner("Setting up simulation", autostart=True)
    spinner.setup_logging(
        level=logging.INFO, format="%(asctime)s [%(levelname)s] - %(message)s"
    )

    # Connect to Carla server
    client = carla.Client(carla_host, carla_ip)
    load_map(client, args.map_filepath)
    world = client.reload_world()
    # Wait until the map is fully loaded

    timeout = 10.0  # seconds
    start_time = time.time()
    while world.get_map() is None:
        if time.time() - start_time > timeout:
            raise RuntimeError("Map failed to load in time")
        time.sleep(0.1)

    # Set server to fixed time-step and synchronous (unless --asynchronous)
    settings = world.get_settings()
    timestep = None
    if not args.asynchronous:
        settings.synchronous_mode = True
        timestep = args.timestep if args.timestep else 0.01
        settings.fixed_delta_seconds = timestep
        # fixed_delta_seconds <= max_substep_delta_time * max_substeps
        max_substeps = 20
        settings.max_substeps = max_substeps
        settings.max_substep_delta_time = timestep / max_substeps
    world.apply_settings(settings)

    logging.info(f"Carla world settings: \n{world.get_settings()}")

    traj_recorder = Carla2Traj(world, debug=False)

    # Process trajectories to convert them to Carla format and add speed and heading info
    trajectories_list = []
    for file in args.trajectory_filepaths:
        try:
            vehicle_name = os.path.basename(file).split(".")[0]
            trajectories_list.append((process_trajectory_file(file), vehicle_name))
        except Exception as e:
            logging.warning(
                f"Got exception <<{e}>> upon processing trajectory file {file}. Skipping..."
            )

    # Prepare vehicles on rails and initial speed vectors
    vehicles = []
    start_vectors = []
    for trajectory, vehicle_name in trajectories_list:
        statistic = Average_Distance_Interpolated()
        new_vehicle = Vehicle(
            world,
            trajectory,
            vehicle_name,
            movement=args.movement,
            deviation_statistics=statistic,
        )
        vehicles.append(new_vehicle)
        heading, speed_kmh = (
            new_vehicle.first_trajectory_point.transform.rotation.yaw,
            new_vehicle.first_trajectory_point.speed,
        )
        heading_radians = math.radians(heading)
        x = math.cos(heading_radians)
        y = math.sin(heading_radians)
        # create speed vector, convert km/h to m/s
        start_vector = carla.Vector3D(x * speed_kmh / 3.6, y * speed_kmh / 3.6, 0)
        start_vectors.append(start_vector)

    # Prepare PCLA agent vehicle (optional)
    if args.pcla_agent:
        logging.info(
            f"Adding PCLA agent: {args.pcla_agent} with route: {args.pcla_route}"
        )
        pcla_agent_enum = PCLA_Agent(args.pcla_agent)
        pcla_movement = PCLA_Movement(pcla_agent_enum, client)
        pcla_movement.set_throttle_exponent(1)
        pcla_vehicle = Vehicle(
            world,
            args.pcla_route,
            "pcla_agent",
            pcla_movement,
            blueprint="vehicle.audi.etron",
        )
        vehicles.append(pcla_vehicle)

    logging.info(f"Vehicle list: {','.join([v.name for v in vehicles])}")

    success = True
    start_ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    try:
        spinner.update_message("Stepping simulation")

        # Define t0 at spawn time
        start_time = world.get_snapshot().timestamp.elapsed_seconds

        # Start at t=0 relative to spawn, plus potential offset
        adjusted_elapsed_sim_seconds = args.offset_time

        # step vehicles once to spawn those that spawn at the start
        [v.step(adjusted_elapsed_sim_seconds) for v in vehicles]

        ## CAMERA

        # Start camera
        cam = StreamingCamera(
            world,
            loc=(
                args.overhead_camera_position
                if args.overhead_camera_position
                else (0, 0, 50)
            ),
            fps=1 / timestep if timestep else 30,  # fallback for asynchronous mode
            output_dir=args.camera_output_dir,
            video_name=f"camera_{start_ts}",
            preferred_fourccs=["mp4v"],
            ego_vehicle=(
                [v for v in vehicles if hasattr(v, "_actor")][0]
                if args.camera_mode == "ego"
                else None
            ),
        )

        cam.timestamp_offset = -start_time
        cam.start_recording()

        while (
            adjusted_elapsed_sim_seconds < args.simulation_duration + args.offset_time
        ):
            world.tick()
            snapshot = world.get_snapshot()
            traj_recorder.process_world_snapshot(snapshot)
            adjusted_elapsed_sim_seconds = (
                snapshot.timestamp.elapsed_seconds - start_time + args.offset_time
            )
            [v.step(adjusted_elapsed_sim_seconds) for v in vehicles]
            active_vehicles = [v for v in vehicles if not v._destroyed and v.is_spawned()]

            # check if any vehicle has fallen off the road
            for v in vehicles:
                if not v.has_valid_z():
                    # raise Exception(
                    #     f"Vehicle {v.name} has invalid z-coordinate at sim time {adjusted_elapsed_sim_seconds}. Current z: {v.get_actor().get_transform().location.z}"
                    # )
                    pass

            spinner.update_message(
                f"Stepping simulation {round((adjusted_elapsed_sim_seconds - args.offset_time) / args.simulation_duration * 100)}%. #active 🚗: {len(active_vehicles)}"
            )

    except Exception as e:
        logging.error(f"Exception! --> {e}", exc_info=True)
        success = False

    finally:
        if spinner is not None:
            spinner.stop()
        logging.info(
            f"Client: Stopped sending `ticks` after {adjusted_elapsed_sim_seconds - args.offset_time} adjusted elapsed simulation seconds."
        )
        cam.stop_recording()
        print(f"Video saved to: {cam.video_path}")
        traj_output_filepath = os.path.join(
            args.traj_output_dir, f"traj_{start_ts}.parquet"
        )
        os.makedirs(os.path.dirname(traj_output_filepath), exist_ok=True)
        traj_recorder.save(traj_output_filepath)
        if success:
            logging.info("Simulation completed successfully.")
            quit(0)
        else:
            logging.warning("Simulation did not complete successfully.")
            quit(1)


if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    main()
