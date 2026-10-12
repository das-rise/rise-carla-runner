import os
import sys
import json
import carla
import time
import csv
import argparse
import statistics
import traceback
from datetime import datetime
from io import StringIO

# Set CUBLAS workspace config before any torch imports (required by some agents like carl)
os.environ['CUBLAS_WORKSPACE_CONFIG'] = ':4096:8'

# Disable DeepSpeed custom op builds (avoids nvcc requirement on hosts without CUDA toolkit)
os.environ.setdefault('DS_BUILD_OPS', '0')

# Disable Weights & Biases in non-interactive test runs to avoid login prompts
os.environ['WANDB_DISABLED'] = 'true'
os.environ['WANDB_SILENT'] = 'true'
os.environ.setdefault('WANDB_MODE', 'offline')

# Setup path early so we can import PCLA
pcla_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if pcla_dir not in sys.path:
    sys.path.insert(0, pcla_dir)

def load_agents_config(pcla_dir):
    """Load agents.json and return (agent_list, total_count, skipped_count, skipped_names)."""
    config_path = os.path.join(pcla_dir, "agents.json")
    with open(config_path, 'r') as f:
        agents_config = json.load(f)
    
    # Agents to skip (missing deps or heavy models you don't want to load now)
    # skip_agents = {'if_if', 'simlingo_simlingo'}
    skip_agents = {}
    # Keep lmdrive excluded for now; tfv3 is enabled again.
    skip_families = {}
    
    agent_list = []
    total_agents = 0
    skipped_agents = 0
    skipped_names = []
    # Seeded agents: only apply seeds to specific agent families
    seeded_families = {
        'carl': ('plant', 'roach', 'carl'),
        'tfv4': ('l6', 'lav', 'wp', 'aim'),
        'plant2': ('plant2')
    }
    
    for agent_family, variants in agents_config.items():
        # Skip entire families (e.g., lmdrive) if requested
        family_skipped = agent_family in skip_families
        for variant_name, variant_config in variants.items():
            # Check if this family/variant combo needs a seed
            seed_suffix = ''
            if agent_family in seeded_families and variant_name in seeded_families[agent_family]:
                seed_suffix = '_0'  # Use only seed 0
            
            agent_full_name = f"{agent_family}_{variant_name}{seed_suffix}"
            total_agents += 1
            
            # Skip known incompatible agents
            if family_skipped or agent_full_name in skip_agents:
                skipped_agents += 1
                skipped_names.append(agent_full_name)
                continue
            
            agent_list.append(agent_full_name)
    
    return agent_list, total_agents, skipped_agents, skipped_names

def _summarize_timing(infer_ms, loop_ms, steady):
    """Turn per-frame timings into steady-state stats.

    Agents with a temporal sensor queue (e.g. tt/minddrive fill ~31 frames before
    running the full model) emit trivial control during warmup, so we report the
    mean of the LAST `steady` frames as the steady-state figure, plus the median
    and max over all frames for context.
    """
    if not infer_ms:
        return None
    w = infer_ms[-steady:] if len(infer_ms) >= steady else infer_ms
    lw = loop_ms[-steady:] if len(loop_ms) >= steady else loop_ms
    infer_mean = statistics.mean(w)
    loop_mean = statistics.mean(lw)
    return {
        # n_frames: how many frames were actually timed for this agent (== --frames,
        #           minus any that errored). The full sample the stats are drawn from.
        'n_frames': len(infer_ms),
        # n_steady: how many of the final frames were averaged for the steady-state
        #           figures below (== --steady). Averaging only the tail excludes the
        #           warmup phase (model init + queue-filling) from the headline numbers.
        'n_steady': len(w),
        # infer_ms: THE inference latency. Mean wall-clock of get_action() over the last
        #           n_steady frames, in milliseconds. This is pure agent compute (sensor
        #           preprocessing + model forward + control post-processing); reading the
        #           control forces a CUDA sync, so it captures real GPU time. Independent
        #           of the CARLA server / network.
        'infer_ms': infer_mean,
        # infer_fps: the same thing expressed as a rate: 1000 / infer_ms. "How many
        #            control outputs per second the agent alone can produce."
        'infer_fps': (1000.0 / infer_mean) if infer_mean > 0 else float('inf'),
        # infer_median_ms: median get_action() over ALL frames (not just the tail). For a
        #                  queue-based agent this sits between the fast warmup and the slow
        #                  steady state, so it is mainly a sanity check against infer_ms.
        'infer_median_ms': statistics.median(infer_ms),
        # infer_max_ms: slowest single get_action() over the whole run — the worst-case
        #               latency, usually the very first full forward pass (lazy CUDA init
        #               / cuDNN autotuning). Useful as a cold-start / tail-latency figure.
        'infer_max_ms': max(infer_ms),
        # loop_ms: end-to-end wall-clock of one full iteration over the last n_steady
        #          frames = get_action() + apply_control() + world.tick(). Unlike infer_ms
        #          this INCLUDES the CARLA server step and the network round-trip to the
        #          (remote) server, so it reflects the rate you actually achieve in the loop.
        'loop_ms': loop_mean,
        # loop_fps: 1000 / loop_ms — the real end-to-end frame rate "you receive" driving
        #           this agent against this server. Always <= infer_fps (extra work per frame).
        'loop_fps': (1000.0 / loop_mean) if loop_mean > 0 else float('inf'),
    }


def test_agent(agent_name, pcla_dir, world, vehicle, route_path, client, n_frames=45, steady=10):
    """
    Test a single agent: initialize, run `n_frames`, timing each get_action().
    Returns (passed: bool, error_msg: str or None, timing: dict or None).
    """
    try:
        import torch
        # Reset torch dtype to default (some agents like neat may set bfloat16)
        torch.set_default_dtype(torch.float32)

        # Reset deterministic algorithms (carl sets True, causing cumsum issues in tfv4)
        torch.use_deterministic_algorithms(False)

        # Mock stdin for agents that prompt for user input (e.g., LAV fast)
        # Supply "3" as the default choice
        old_stdin = sys.stdin
        sys.stdin = StringIO("3\n")

        try:
            # Import PCLA (sys.path already set at module level)
            from PCLA import PCLA

            pcla = None
            try:
                pcla = PCLA(agent_name, vehicle, route_path, client)

                infer_ms, loop_ms = [], []
                for frame in range(n_frames):
                    try:
                        t0 = time.perf_counter()
                        ego_action = pcla.get_action()          # inference: forward pass + control
                        t1 = time.perf_counter()                # get_action reads control -> forces GPU sync
                        vehicle.apply_control(ego_action)
                        world.tick()                            # server step + network round-trip (remote)
                        t2 = time.perf_counter()
                        infer_ms.append((t1 - t0) * 1000.0)
                        loop_ms.append((t2 - t0) * 1000.0)
                    except Exception as e:
                        pcla.cleanup()
                        return False, f"Frame {frame}: {str(e)}", None

                pcla.cleanup()
                return True, None, _summarize_timing(infer_ms, loop_ms, steady)
            except KeyboardInterrupt:
                # PCLA's load watchdog raises KeyboardInterrupt via interrupt_main().
                # Distinguish that (agent too slow -> skip and continue) from a genuine
                # Ctrl+C (re-raise -> abort the whole suite).
                wd = getattr(pcla, '_watchdog', None)
                watchdog_fired = (pcla is None) or (wd is not None and not wd.get_status())
                if pcla is not None:
                    try:
                        pcla.cleanup()
                    except Exception:
                        pass
                if watchdog_fired:
                    return (False,
                            f"Timed out loading/running (> {os.environ.get('PCLA_WATCHDOG_SEC')}s "
                            f"watchdog); raise --watchdog if the model is just slow", None)
                raise
        finally:
            sys.stdin = old_stdin
    except Exception as e:
        error_msg = f"{type(e).__name__}: {str(e)}\n{traceback.format_exc()}"
        return False, error_msg, None

def main():
    # Setup CARLA
    global pcla_dir

    ap = argparse.ArgumentParser(
        description="Run each PCLA agent for a few frames and report inference time + FPS.")
    ap.add_argument("--host", default="localhost",
                    help="CARLA server IP. Use the remote server's IP if CARLA runs "
                         "elsewhere, e.g. --host 10.21.13.26")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--town", default="Town02")
    ap.add_argument("--frames", type=int, default=45,
                    help="frames per agent; must exceed the longest sensor-queue warmup "
                         "(~31 for tt/minddrive) so steady-state inference is reached")
    ap.add_argument("--steady", type=int, default=10,
                    help="number of final frames averaged for the steady-state figure")
    ap.add_argument("--agents", nargs="+", metavar="NAME", default=None,
                    help="run only these agents (full names, e.g. tfv4_lav_0 if_if) "
                         "for a quick timing check instead of the whole suite")
    ap.add_argument("--tm-port", type=int, default=8000,
                    help="Traffic Manager port. If 8000 is taken (stale TM or another "
                         "user on a shared server), the script tries the next few ports.")
    ap.add_argument("--watchdog", type=int, default=900,
                    help="seconds allowed for an agent to load/build before it is timed "
                         "out (large VLAs / LMDrive load slowly cold; default 900 = 15 min)")
    args = ap.parse_args()

    # PCLA reads this when it constructs its load watchdog (see PCLA.py).
    os.environ['PCLA_WATCHDOG_SEC'] = str(args.watchdog)

    # Change to PCLA directory so relative paths in agent configs work
    os.chdir(pcla_dir)

    client = carla.Client(args.host, args.port)
    # Generous RPC timeout: a remote server plus heavy agents (LMDrive, VLAs) can be slow.
    client.set_timeout(60.0)
    print(f"Connecting to CARLA at {args.host}:{args.port} ...")
    client.load_world(args.town)
    
    try:
        world = client.get_world()

        # Traffic Manager: only needed to keep TM-controlled traffic in sync, and this
        # test spawns none (the ego is agent-controlled). So try to bind one on the
        # requested port, fall through a few alternates if it's taken (stale TM or a
        # co-tenant on a shared server holding 8000), and continue without it if none
        # bind rather than aborting the whole run.
        traffic_manager = None
        for p in range(args.tm_port, args.tm_port + 10):
            try:
                traffic_manager = client.get_trafficmanager(p)
                traffic_manager.set_synchronous_mode(True)
                print(f"Traffic Manager bound on port {p}.")
                break
            except RuntimeError:
                continue
        if traffic_manager is None:
            print(f"WARNING: no free Traffic Manager port in "
                  f"{args.tm_port}..{args.tm_port + 9}; continuing without one "
                  f"(fine here — no traffic is spawned).")

        settings = world.get_settings()
        if not settings.synchronous_mode:
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = 0.05
        world.apply_settings(settings)
        
        # Vehicle & spectator setup helpers
        bpLibrary = world.get_blueprint_library()
        vehicleBP = bpLibrary.filter('model3')[0]
        vehicle_spawn_points = world.get_map().get_spawn_points()
        spectator = world.get_spectator()

        def spawn_vehicle():
            # Try preferred spawn, then fall back through the list
            vehicle_candidates = [31] + list(range(len(vehicle_spawn_points)))
            veh = None
            for idx in vehicle_candidates:
                try:
                    veh = world.try_spawn_actor(vehicleBP, vehicle_spawn_points[idx])
                except IndexError:
                    continue
                if veh is not None:
                    break
            if veh is None:
                raise RuntimeError("Failed to spawn vehicle")
            # Give the sim a moment to stabilize to avoid attachment errors
            time.sleep(0.5)
            spectator.set_transform(carla.Transform(carla.Location(x=-8, y=108, z=7), carla.Rotation(pitch=-19, yaw=0, roll=0)))
            world.tick()
            return veh

        vehicle = spawn_vehicle()
        
        # Load all agents
        agent_list, total_agents, skipped_agents, skipped_names = load_agents_config(pcla_dir)
        if args.agents:   # quick subset run
            requested = set(args.agents)
            unknown = requested - set(agent_list)
            if unknown:
                print(f"WARNING: unknown agent name(s) ignored: {sorted(unknown)}")
            agent_list = [a for a in agent_list if a in requested]
        results = {}
        timings = {}   # agent_name -> timing dict (for passed agents)

        print(f"\n{'='*70}")
        print(f"Testing {len(agent_list)} agents (total {total_agents}, skipping {skipped_agents})...")
        print(f"Per agent: {args.frames} frames, steady-state = last {args.steady}.")
        if skipped_names:
            print(f"Skipped: {', '.join(skipped_names)}")
        print(f"{'='*70}\n")

        route_path = os.path.join(pcla_dir, "sample_route.xml")

        for i, agent_name in enumerate(agent_list, 1):
            print(f"[{i}/{len(agent_list)}] Testing: {agent_name}")

            # Ensure a fresh vehicle for each agent (previous cleanup may destroy it)
            if vehicle is None or not vehicle.is_alive:
                vehicle = spawn_vehicle()

            passed, error_msg, timing = test_agent(
                agent_name, pcla_dir, world, vehicle, route_path, client,
                n_frames=args.frames, steady=args.steady)

            # PCLA.cleanup may destroy the vehicle; reset reference
            vehicle = None

            if passed:
                results[agent_name] = "passed"
                timings[agent_name] = timing
                if timing:
                    print(f"  ✓ PASSED | inference {timing['infer_ms']:.1f} ms "
                          f"({timing['infer_fps']:.1f} fps) | end-to-end {timing['loop_fps']:.1f} fps "
                          f"| peak {timing['infer_max_ms']:.0f} ms")
                else:
                    print(f"  ✓ PASSED (no timing)")
            else:
                results[agent_name] = error_msg
                print(f"  ✗ FAILED: {error_msg.split(chr(10))[0][:80]}")
                print(f"  Skipping to next agent...")

            print()
        
        # Cleanup vehicle (if any remains)
        try:
            if vehicle is not None and vehicle.is_alive:
                vehicle.destroy()
        except Exception:
            pass

        # Write results to file
        documents_dir = os.path.join(pcla_dir, "documents")
        os.makedirs(documents_dir, exist_ok=True)
        results_file = os.path.join(documents_dir, "agent_test_results.txt")
        with open(results_file, 'w') as f:
            f.write(f"Agent Test Results - {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"{'='*70}\n\n")
            
            passed_count = sum(1 for v in results.values() if v == "passed")
            failed_count = len(results) - passed_count
            
            f.write(f"Summary: {passed_count} passed, {failed_count} failed out of {len(results)} agents\n\n")
            f.write(f"Skipped agents: {skipped_agents}\n")
            if skipped_names:
                f.write("Skipped list: " + ", ".join(skipped_names) + "\n")
            f.write(f"{'='*70}\n\n")
            
            # Timing table (steady-state inference + FPS), fastest first.
            f.write("INFERENCE TIME & FPS  (steady-state = mean of last "
                    f"{args.steady} of {args.frames} frames):\n")
            f.write("-" * 70 + "\n")
            f.write(f"{'agent':22s} {'infer_ms':>9s} {'infer_fps':>10s} "
                    f"{'loop_fps':>9s} {'peak_ms':>8s}\n")
            for agent_name, t in sorted(timings.items(), key=lambda kv: kv[1]['infer_ms']):
                f.write(f"{agent_name:22s} {t['infer_ms']:9.1f} {t['infer_fps']:10.1f} "
                        f"{t['loop_fps']:9.1f} {t['infer_max_ms']:8.0f}\n")

            f.write(f"\n{'='*70}\n\n")
            f.write("PASSED AGENTS:\n")
            f.write("-" * 70 + "\n")
            for agent_name, status in results.items():
                if status == "passed":
                    f.write(f"{agent_name}\n")

            f.write(f"\n{'='*70}\n\n")
            f.write("FAILED AGENTS:\n")
            f.write("-" * 70 + "\n")
            for agent_name, error_msg in results.items():
                if error_msg != "passed":
                    f.write(f"\n{agent_name}:\n")
                    f.write(f"{error_msg}\n")
                    f.write("-" * 70 + "\n")

        # Machine-readable timing for the paper / plots.
        csv_file = os.path.join(documents_dir, "agent_fps.csv")
        with open(csv_file, 'w', newline='') as cf:
            w = csv.writer(cf)
            w.writerow(["agent", "infer_ms_steady", "infer_fps", "infer_median_ms",
                        "infer_max_ms", "loop_fps", "frames", "steady_frames"])
            for agent_name, t in sorted(timings.items(), key=lambda kv: kv[1]['infer_ms']):
                w.writerow([agent_name, f"{t['infer_ms']:.2f}", f"{t['infer_fps']:.2f}",
                            f"{t['infer_median_ms']:.2f}", f"{t['infer_max_ms']:.2f}",
                            f"{t['loop_fps']:.2f}", t['n_frames'], t['n_steady']])

        print(f"\nResults saved to: {results_file}")
        print(f"Timing CSV saved to: {csv_file}")
        
    finally:
        settings = world.get_settings()
        settings.synchronous_mode = False
        world.apply_settings(settings)

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nTest suite interrupted.")
    except Exception as e:
        print(f"\nFatal error: {e}")
        traceback.print_exc()
