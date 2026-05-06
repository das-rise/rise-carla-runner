import argparse
import logging
import math
import os
import random
import signal
import sys
import threading as _threading
import time
from datetime import datetime, timezone

import carla
import numpy as np
from rich_argparse import RichHelpFormatter
from traj_convert.carla2traj import Carla2Traj

from rirun.kinetics.movement import BehaviorMovement, PCLA_Movement
from rirun.kinetics.trajectory_utils import process_trajectory_file
from rirun.kinetics.vehicle_tools import Vehicle, spawn_behavior_npcs
from rirun.PCLA.PCLA_agents import PCLA_Agent, check_agent_env
from rirun.utils.bling import rirun
from rirun.utils.carla_tools import load_map
from rirun.utils.spinner import Spinner
from rirun.utils.video_tools import StreamingCamera

# Helper functions


def timestamp_now_dataprov() -> str:
    """Returns the current timestamp in ISO 8601 format for use with dataprov."""
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


_stop_event = _threading.Event()


def cleanup(sig: signal.Signals) -> None:
    """
    Perform cleanup and log the received signal
    """
    logging.warning(f"Execution aborted through signal {sig}.")
    # Add any cleanup code here


def signal_handler(sig: signal.Signals, frame) -> None:
    """
    Handle signals send to this script and exit.
    First press: set a stop flag so the main loop exits cleanly after the
    current world.tick() returns (avoids interrupting a blocking C call).
    Second (impatient) press: hard-exit immediately.
    """
    cleanup(sig)
    if not _stop_event.is_set():
        _stop_event.set()
    else:
        logging.warning(f"Second signal {sig} received, exiting immediately.")
        os._exit(1)


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
        "--npc_trajectory_filepaths",
        "-tf",
        type=str,
        nargs="*",
        dest="npc_trajectory_filepaths",
        default=[],
        help="Path(s) to NPC trajectory CSV file(s).",
    )
    parser.add_argument(
        "--output_dir",
        type=str,
        default="output",
        help="Where to store output files. Default: output",
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
        "--npc_movement",
        choices=["pid", "pid_ts", "teleport", "behavior_agent"],
        default="teleport",
        help="How vehicles move along trajectories (default: teleport). "
        "Use 'behavior_agent' to hand off each trajectory vehicle to a BehaviorAgent "
        "after spawning at its first trajectory position.",
    )
    parser.add_argument(
        "--npc_behavior",
        choices=["cautious", "normal", "aggressive"],
        default=None,
        help="BehaviorAgent driving style used when --npc_movement behavior_agent is set (default: cautious).",
    )
    parser.add_argument(
        "--display_camera",
        choices=["ego_overhead", "stationary_overhead", "ego_dashcam"],
        default=None,
        help="Primary camera for display. If omitted, uses the first --record_cameras mode; "
        "if neither is provided, uses stationary_overhead.",
    )
    parser.add_argument(
        "--record_cameras",
        nargs="+",
        choices=["ego_dashcam", "ego_overhead", "stationary_overhead"],
        default=None,
        metavar="MODE",
        help="Record one or more camera views simultaneously: ego_dashcam, ego_overhead, stationary_overhead. "
        "Each mode produces a separate MP4 file named camera_<mode>_<ts>.mp4. "
        "If not specified, records only the display camera.",
    )
    parser.add_argument(
        "--overhead_camera_position",
        type=float,
        nargs=3,
        action="append",
        metavar=("X", "Y", "Z"),
        help="Overhead camera position as X Y Z. Repeat the flag once per overhead camera mode "
        "(ego_overhead, stationary_overhead) in the same order as --record_cameras. "
        "A single occurrence applies to all overhead modes. "
        "For stationary_overhead this is a fixed world position; "
        "for ego_overhead it is a relative offset from the followed vehicle. "
        "Example (two overhead modes, different positions): "
        "--record_cameras ego_overhead stationary_overhead "
        "--overhead_camera_position 0 0 50 "
        "--overhead_camera_position 1505 -1145 100",
    )
    parser.add_argument(
        "--ego_agent",
        type=str,
        choices=[agent.value for agent in PCLA_Agent] + ["behavior_agent"],
        help="Add an autonomous driving agent to the simulation. Use 'behavior_agent' for BehaviorAgent, or specify a PCLA agent name.",
    )
    parser.add_argument(
        "--ego_behavior",
        type=str,
        choices=["cautious", "normal", "aggressive"],
        default=None,
        help="BehaviorAgent driving style used when --ego_agent behavior_agent is set (default: cautious).",
    )
    parser.add_argument(
        "--ego_route_filepath",
        type=str,
        help="Path to XML file containing the route for the ego agent (required if --ego_agent is specified).",
    )
    parser.add_argument(
        "--ego_spawn_time",
        type=float,
        default=-1.0,
        help="The simulation time at which the ego agent's vehicle should be spawned and start following the trajectory. If set to a negative value, the vehicle will be spawned immediately at the first trajectory point. Defaults to -1.0.",
    )
    parser.add_argument(
        "--offset_time",
        type=float,
        default=0.0,
        help="Time offset in seconds to start the simulation at (default: 0.0).",
    )
    parser.add_argument(
        "--npc_trajectory_offset",
        type=float,
        nargs=2,
        metavar=("X_OFFSET", "Y_OFFSET"),
        default=None,
        help="Add (x, y) offset to all trajectory coordinates before Carla conversion. "
        "Use when trajectories were extracted with a subtracted origin (e.g. savant2rirun --offset X Y).",
    )
    parser.add_argument(
        "--trajectory_statistics",
        type=str,
        choices=["average_distance_true"],
        help="Choice of trajectory deviation statistics to compute during the run.",
    )
    parser.add_argument(
        "--heading_interpolation_mode",
        type=str,
        choices=["straight", "spline"],
        default="spline",
        help="Heading interpolation mode for trajectory processing (default: spline).",
    )
    parser.add_argument(
        "--force_heading_interpolation",
        action="store_true",
        help="Force heading interpolation even when heading data is present.",
    )
    parser.add_argument(
        "--mapmatch",
        action="store_true",
        help="Clamp trajectory points to the road/lane boundary using the Carla map when they are within the matching threshold; otherwise keep the original points.",
    )
    parser.add_argument(
        "--use-dataprov",
        action="store_true",
        help="Use dataprov library to create a provenance chain for the conversion",
    )
    parser.add_argument(
        "--dataprov-input-provenance-files",
        type=str,
        nargs="*",
        default=[],
        help="List of dataprov provenance files in the order 'OpenLabel, georef' to include as inputs in the provenance chain. One or both can be 'None'. Only used if --use-dataprov is set.",
    )
    parser.add_argument(
        "--num_random_behavior_npcs",
        type=int,
        default=0,
        help="Number of additional BehaviorAgent-driven roaming NPC vehicles (default: 0).",
    )
    parser.add_argument(
        "--no_z_check",
        action="store_true",
        help="Disable the z-coordinate sanity check that stops the simulation when a vehicle falls off the road.",
    )
    parser.add_argument(
        "--on_npc_behavior_agent_route_done",
        choices=["stop", "destroy", "roam"],
        default="stop",
        help="Behavior when an NPC BehaviorAgent completes its route: "
        "stop (default) = brake and hold; destroy = remove from simulation; "
        "roam = pick a new random destination and keep driving.",
    )
    parser.add_argument(
        "--on_ego_behavior_agent_route_done",
        choices=["stop", "roam"],
        default="stop",
        help="Behavior when the ego BehaviorAgent completes its route: "
        "stop (default) = brake and hold; roam = pick a new random destination and keep driving.",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=None,
        help="Random seed for reproducible simulations (affects BehaviorAgent destination sampling).",
    )

    args = parser.parse_args()

    # Validate that ego_route_filepath is provided if ego_agent is specified
    if args.ego_agent and not args.ego_route_filepath:
        parser.error("--ego_route_filepath is required when --ego_agent is specified")
    if args.ego_route_filepath and not args.ego_agent:
        parser.error("--ego_agent is required when --ego_route_filepath is specified")
    if args.ego_behavior and args.ego_agent != "behavior_agent":
        parser.error("--ego_behavior can only be used with --ego_agent behavior_agent")
    if args.npc_behavior and args.npc_movement != "behavior_agent":
        parser.error(
            "--npc_behavior can only be used with --npc_movement behavior_agent"
        )
    if not args.npc_trajectory_filepaths and not args.ego_agent:
        parser.error("At least one trajectory file or --ego_agent must be provided")

    # Validate required environment variables based on chosen ego agent
    if args.ego_agent and args.ego_agent != "behavior_agent":
        try:
            check_agent_env(PCLA_Agent(args.ego_agent))
        except RuntimeError as e:
            parser.error(str(e))

    return args


def log_simulation_provenance(
    run_start_time: str, args: argparse.Namespace, start_ts: str
) -> None:
    """Log simulation provenance using the dataprov library.

    Creates a provenance chain capturing inputs, outputs, tool metadata, and
    runtime environment, then saves it as a JSON file in the output directory.

    Args:
        run_start_time: ISO 8601 timestamp string marking when the simulation started.
        args: Parsed command-line arguments containing simulation configuration.
        start_ts: Timestamp string used to identify output files for this run.
    """
    import hashlib
    from importlib.metadata import PackageNotFoundError, metadata

    from dataprov import ProvenanceChain

    unique_hash = hashlib.sha256(
        (str(vars(args)) + str(time.time_ns())).encode()
    ).hexdigest()
    tool_name = "rirun"
    try:
        meta = metadata(tool_name)
        tool_version = meta["Version"]
    except PackageNotFoundError:
        tool_version = "unknown"
    entity_id = tool_name + "_" + unique_hash[:8]

    if args.dataprov_input_provenance_files:
        input_provenance_files = args.dataprov_input_provenance_files
    else:
        input_provenance_files = None

    mode_string = "real trajectories" if args.npc_trajectory_filepaths else ""
    mode_string += (
        " and autonomous agents"
        if args.ego_agent and mode_string
        else "autonomous agents"
        if args.ego_agent
        else ""
    )

    raw_inputs = []
    raw_input_formats = []

    if args.npc_trajectory_filepaths:
        raw_inputs.extend(args.npc_trajectory_filepaths)
        raw_input_formats.extend(["CSV"] * len(args.npc_trajectory_filepaths))

    if args.map_filepath:
        raw_inputs.append(args.map_filepath)
        raw_input_formats.append("OpenDrive/Carla map")

    if args.ego_route_filepath:
        raw_inputs.append(args.ego_route_filepath)
        raw_input_formats.append("XML")

    inputs = raw_inputs
    input_formats = raw_input_formats
    chain = ProvenanceChain.create(
        entity_id=entity_id,
        initial_source=args.npc_trajectory_filepaths,
        description=f"rirun simulation results created using {mode_string} on map {args.map_filepath}",
        tags=[
            "RIRUN",
            "trajectory",
            "Synergies",
        ],
    )

    _record_modes = list(args.record_cameras) if args.record_cameras else []

    chain.add(
        started_at=run_start_time,
        ended_at=timestamp_now_dataprov(),
        tool_name=tool_name,
        tool_version=tool_version,
        arguments=" ".join(sys.argv[1:]),
        operation="Execute simulation with provided parameters and/or trajectories and/or autonomous agents and produce resulting trajectories as file and/or video output.",
        inputs=inputs,
        input_formats=input_formats,
        outputs=[args.output_dir + f"/traj/traj_{start_ts}.parquet"]
        + [f"{args.output_dir}/{_mode}_{start_ts}.mp4" for _mode in _record_modes],
        output_formats=["Parquet", "MP4"],
        input_provenance_files=input_provenance_files,
        capture_environment=True,
    )

    chain.save(f"{args.output_dir}/{entity_id}_prov.json")


def main() -> None:
    run_start_time = timestamp_now_dataprov()

    args = parse_arguments()

    # Seed all random generators for reproducibility
    _seed = args.seed
    if _seed is not None:
        random.seed(_seed)
        np.random.seed(_seed)
        logging.getLogger().info(f"Random seed set to {_seed}")
    _rng = random.Random(_seed)  # RNG for NPC trajectory vehicles using behavior_agent

    if not args.carla_address:
        carla_host = "localhost"
        carla_port = 2000
    else:
        carla_host = args.carla_address.split(":")[0]
        carla_port = int(args.carla_address.split(":")[1])

    rirun()

    spinner = Spinner("Setting up simulation", autostart=True)
    spinner.setup_logging(
        level=logging.INFO, format="%(asctime)s [%(levelname)s] - %(message)s"
    )

    # Setup output directories
    os.makedirs(args.output_dir + "/camera", exist_ok=True)
    os.makedirs(args.output_dir + "/traj", exist_ok=True)

    # Connect to Carla server
    client = carla.Client(carla_host, carla_port)
    world = load_map(client, args.map_filepath)
    if world is None:
        world = client.get_world()

    # Wait until the map is fully loaded
    timeout = 10.0  # seconds
    map_start_time = time.time()
    while world.get_map() is None:
        if time.time() - map_start_time > timeout:
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
        max_substeps = 10
        settings.max_substeps = max_substeps
        settings.max_substep_delta_time = timestep / (max_substeps - 1)
    world.apply_settings(settings)

    logging.info(f"Carla world settings: \n{world.get_settings()}")

    # Setup trajectory recorder
    traj_recorder = Carla2Traj(world, debug=False)

    # Process trajectories to convert them to Carla format and add speed and heading info
    trajectories_list = []
    for file in args.npc_trajectory_filepaths:
        try:
            vehicle_name = os.path.basename(file).split(".")[0]
            trajectories_list.append(
                (
                    process_trajectory_file(
                        file,
                        heading_interpolation_mode=args.heading_interpolation_mode,
                        force_heading_interpolation=args.force_heading_interpolation,
                        mapmatch=args.mapmatch,
                        world=world,
                        offset=args.npc_trajectory_offset,
                    ),
                    vehicle_name,
                )
            )
        except Exception as e:
            logging.warning(
                f"Got exception <<{e}>> upon processing trajectory file {file}. Skipping...",
                exc_info=True,
            )

    # Prepare vehicles on rails and initial speed vectors
    vehicles = []
    start_vectors = []
    for trajectory, vehicle_name in trajectories_list:
        if args.npc_movement == "behavior_agent":
            movement = BehaviorMovement(
                client,
                behavior=args.npc_behavior or "cautious",
                on_route_done=args.on_npc_behavior_agent_route_done,
                rng=_rng,
            )
        else:
            movement = args.npc_movement
        new_vehicle = Vehicle(
            world,
            trajectory,
            vehicle_name,
            movement=movement,
            deviation_statistics=args.trajectory_statistics,
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

    # Prepare ego agent vehicle (optional)
    if args.ego_agent:
        if args.ego_agent == "behavior_agent":
            ego_behavior = args.ego_behavior or "cautious"
            logging.info(
                f"Adding BehaviorAgent ego: behavior={ego_behavior}, route={args.ego_route_filepath}"
            )
            behavior_movement = BehaviorMovement(
                client,
                behavior=ego_behavior,
                xml_route=args.ego_route_filepath,
                on_route_done=args.on_ego_behavior_agent_route_done,
            )
            ego_vehicle = Vehicle(
                world,
                args.ego_route_filepath,
                "ego_agent",
                behavior_movement,
                blueprint="vehicle.audi.etron",
                role_name="hero",
            )
        else:
            logging.info(
                f"Adding PCLA agent: {args.ego_agent} with route: {args.ego_route_filepath}"
            )
            pcla_agent_enum = PCLA_Agent(args.ego_agent)
            pcla_movement = PCLA_Movement(pcla_agent_enum, client, args.ego_spawn_time)
            pcla_movement.set_throttle_exponent(1)
            ego_vehicle = Vehicle(
                world,
                args.ego_route_filepath,
                "ego_agent",
                pcla_movement,
                blueprint="vehicle.audi.etron",
                role_name="hero",
            )
        vehicles.append(ego_vehicle)

    # Prepare BehaviorAgent roaming NPCs (optional)
    if args.num_random_behavior_npcs > 0:
        logging.info(
            f"Spawning {args.num_random_behavior_npcs} BehaviorAgent NPC vehicles..."
        )
        behavior_npcs = spawn_behavior_npcs(
            world,
            client,
            num=args.num_random_behavior_npcs,
            npc_behavior="cautious",
            min_spacing=15.0,
        )
        vehicles.extend(behavior_npcs)

    logging.info(f"Vehicle list: {','.join([v.name for v in vehicles])}")

    success = True
    start_ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    adjusted_elapsed_sim_seconds = args.offset_time
    num_ticks = 0
    display_screen = None
    try:
        spinner.update_message("Stepping simulation")

        # Define t0 at spawn time
        simulation_start_time = world.get_snapshot().timestamp.elapsed_seconds

        # Start at t=0 relative to spawn, plus potential offset
        active_vehicles = []

        # step vehicles once to spawn those that spawn at the start
        for v in vehicles:
            v.step(adjusted_elapsed_sim_seconds)

        ## CAMERA

        # Start camera
        # Determine initial ego camera target (priority: ego_agent > first NPC)
        def _pick_ego_vehicle(vehicle_list):
            for name in ("ego_agent",):
                v = next(
                    (v for v in vehicle_list if v.name == name and v.is_spawned()), None
                )
                if v:
                    return v
            return next((v for v in vehicle_list if v.is_spawned()), None)

        _record_modes = list(args.record_cameras) if args.record_cameras else []
        _display_mode = args.display_camera or (
            _record_modes[0] if _record_modes else "stationary_overhead"
        )
        # Build list of camera modes to record; default to just the display camera.
        if not _record_modes:
            _record_modes = [_display_mode]
        # Ensure the display camera is always recorded.
        if _display_mode not in _record_modes:
            _record_modes.append(_display_mode)
        # Primary display index: the resolved display camera mode.
        _display_idx = _record_modes.index(_display_mode)

        cams = []  # list of [mode, ego_vehicle_or_None, StreamingCamera]
        _positions = args.overhead_camera_position  # list of (x, y, z) or None
        _overhead_pos_idx = (
            0  # counts only overhead modes; dashcam does not consume a triplet
        )
        for _mode in _record_modes:
            _cam_ego = (
                _pick_ego_vehicle(vehicles)
                if _mode in ("ego_overhead", "ego_dashcam")
                else None
            )
            # ego_dashcam uses a fixed vehicle-relative transform; overhead_camera_position does not apply.
            if _mode in ("ego_overhead", "stationary_overhead"):
                if _positions:
                    _cam_pos = _positions[min(_overhead_pos_idx, len(_positions) - 1)]
                else:
                    _cam_pos = (0, 0, 50)
                _overhead_pos_idx += 1
            else:
                _cam_pos = (0, 0, 50)  # unused by dashcam
            _c = StreamingCamera(
                world,
                loc=_cam_pos,
                fps=1 / timestep if timestep else 30,
                output_dir=args.output_dir + "/camera",
                video_name=f"camera_{_mode}_{start_ts}",
                preferred_fourccs=["mp4v"],
                ego_vehicle=_cam_ego,
                display_camera=_mode,
            )
            _c.timestamp_offset = -simulation_start_time
            cams.append([_mode, _cam_ego, _c])

        for _, _, _c in cams:
            _c.start_recording()

        # Optional pygame display: enabled automatically when a display server is available.
        has_attached_display = bool(
            os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")
        )
        if has_attached_display:
            try:
                import pygame as _pg

                _pg.init()
                display_screen = _pg.display.set_mode((1280, 720))
                _pg.display.set_caption("RiRun Camera")
            except Exception as e:
                logging.warning(
                    f"Display detected but pygame preview could not start: {e}"
                )
                display_screen = None
        else:
            logging.info("No display detected; running without live pygame preview.")

        while (
            adjusted_elapsed_sim_seconds < args.simulation_duration + args.offset_time
            and not _stop_event.is_set()
        ):
            world.tick()
            num_ticks += 1
            snapshot = world.get_snapshot()
            traj_recorder.process_world_snapshot(snapshot)
            adjusted_elapsed_sim_seconds = (
                snapshot.timestamp.elapsed_seconds
                - simulation_start_time
                + args.offset_time
            )
            for v in vehicles:
                v.step(adjusted_elapsed_sim_seconds)
            active_vehicles = [
                v for v in vehicles if not v._destroyed and v.is_spawned()
            ]

            # Reattach any ego_overhead/ego_dashcam cameras whose target was destroyed
            if active_vehicles:
                for _cam_entry in cams:
                    _cam_mode, _cam_ego, _cam = _cam_entry
                    if _cam_mode in ("ego_overhead", "ego_dashcam"):
                        if (
                            _cam_ego is None
                            or _cam_ego._destroyed
                            or not _cam_ego.is_spawned()
                        ):
                            _new_ego = _pick_ego_vehicle(active_vehicles)
                            if _new_ego is not None:
                                _cam.reattach(_new_ego)
                                _cam_entry[1] = _new_ego

            # check if any vehicle has fallen off the road
            if not args.no_z_check:
                for v in vehicles:
                    if not v.has_valid_z():
                        raise Exception(
                            f"Vehicle {v.name} has invalid z-coordinate at sim time {adjusted_elapsed_sim_seconds}. "
                            f"Current z: {v.get_actor().get_transform().location.z}"
                        )
            if num_ticks % 30 == 0:
                spinner.update_message(
                    f"Stepping simulation {round((adjusted_elapsed_sim_seconds - args.offset_time) / args.simulation_duration * 100)}%; "
                    f"#active 🚗: {len(active_vehicles)}; ticks: {num_ticks}"
                )

            # Render latest camera frame to pygame window
            if display_screen is not None:
                for event in _pg.event.get():
                    if event.type == _pg.QUIT or (
                        event.type == _pg.KEYDOWN and event.key == _pg.K_ESCAPE
                    ):
                        raise KeyboardInterrupt("pygame window closed")
                _display_cam = cams[_display_idx][2] if cams else None
                if _display_cam is not None and _display_cam._latest_frame is not None:
                    frame_rgb = _display_cam._latest_frame[:, :, ::-1]  # BGR -> RGB
                    surf = _pg.surfarray.make_surface(frame_rgb.swapaxes(0, 1))
                    scaled = _pg.transform.scale(surf, display_screen.get_size())
                    display_screen.blit(scaled, (0, 0))
                    _pg.display.flip()

    except Exception as e:
        logging.error(f"Exception! --> {e}", exc_info=True)
        success = False

    finally:
        if spinner is not None:
            spinner.stop()
        logging.info(
            f"Client: Stopped sending `ticks` after {adjusted_elapsed_sim_seconds - args.offset_time} "
            f"adjusted elapsed simulation seconds. Total ticks: {num_ticks}"
        )
        spinner.update_message("Saving outputs...")
        spinner.start()
        if "cams" in locals():
            for _, _, _c in cams:
                _c.stop_recording(
                    num_ticks
                )  # flush all cameras to disk before proceeding
        traj_recorder.save(args.output_dir + f"/traj/traj_{start_ts}.parquet")

        spinner.stop()

        if display_screen is not None:
            _pg.quit()

        # Destroy all remaining spawned vehicles
        try:
            for v in active_vehicles:
                v.destroy()
        except UnboundLocalError:
            pass

        if args.use_dataprov:
            log_simulation_provenance(run_start_time, args, start_ts)

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
