#!/usr/bin/env bash
# Run a GbgSaroRound XODR-world simulation.
#
# Usage:
#   bash run_GbgSaroRound_XODR.sh [OPTIONS]
#
# Options:
#   --agent                Include the PCLA ego agent (default: on)
#   --npcs                 Include trajectory NPC vehicles (default: on)
#   --npc_movement  <mode>     NPC movement: teleport | behavior_agent  (default: behavior_agent)
#   --npc_behavior  <style>    BehaviorAgent style: cautious | normal | aggressive (default: cautious)
#   --display_camera <mode>   ego_overhead | ego_dashcam | stationary_overhead  (default: ego_dashcam)
#   --record_cameras <modes>  Space-separated list of views to record: ego_dashcam ego_overhead stationary_overhead
#                          (default: records only --display_camera)
#   --overhead_camera_position <x> <y> <z>  Overhead camera position: world-space for stationary_overhead,
#                          relative to followed vehicle for ego_overhead (default: unset)
#   --duration  <secs>     Simulation duration in seconds  (default: 15)
#   --offset_time <secs>   Simulation clock offset to align CSV timestamps (default: 1013)
#   --timestep  <secs>     Fixed simulation timestep (default: 0.033333...)
#   --map       <path>     Path to .xodr map file (default: scenes/roundabout-saro/maps/GbgSaroRound.xodr)
#   --ego_agent <name>    Ego agent name (default: behavior_agent)
#   --on_ego_behavior_agent_route_done <mode> BehaviorAgent ego end-of-route: stop | roam (default: stop)
#   --ego_route_filepath <path>    XML route file for ego agent (default: scenes/roundabout-saro/agent_routes/GbgSaroRound.xml)
#   --traj_files <glob>    Glob pattern for trajectory CSV files (default: data/GbgSaroRound_DT/*Car.csv)
#   --npc_trajectory_offset <x> <y> Trajectory coordinate offset (default: 678710.40 6374600.42)
#   --on_npc_behavior_agent_route_done <mode> BehaviorAgent end-of-route: stop | destroy | roam (default: stop)
#   --seed <int>           Random seed for reproducible runs (default: unset)
#   --no_z_check           Disable z-coordinate fall-off-road check (default: on)
#                          Display preview is auto-detected by run.py.
#
# Examples:
#   # PCLA agent only, dashcam view:
#   bash run_GbgSaroRound_XODR.sh --agent --display_camera ego_dashcam
#
#   # PCLA agent + teleport NPCs:
#   bash run_GbgSaroRound_XODR.sh --agent --npcs --npc_movement teleport
#
#   # PCLA agent + behavior NPCs (cautious):
#   bash run_GbgSaroRound_XODR.sh
#
#   # Behavior NPCs only, no agent:
#   bash run_GbgSaroRound_XODR.sh --npcs --npc_movement behavior_agent --display_camera ego_overhead

set -euo pipefail

# ── Defaults ──────────────────────────────────────────────────────────────────
AGENT=true
NPCS=true
NPC_MOVEMENT=behavior_agent
NPC_BEHAVIOR=cautious
DISPLAY_CAMERA=stationary_overhead
# RECORD_CAMERAS="ego_dashcam ego_overhead stationary_overhead"
RECORD_CAMERAS="ego_dashcam stationary_overhead"
OVERHEAD_POS="1505 -1145 100"
DURATION=40
OFFSET_TIME=1013
TIMESTEP=0.0333333333333333333
MAP=scenes/roundabout-saro/maps/GbgSaroRound.xodr
EGO_AGENT=behavior_agent
EGO_BEHAVIOR=cautious
ON_EGO_BEHAVIOR_AGENT_ROUTE_DONE=stop
EGO_ROUTE_FILEPATH=scenes/roundabout-saro/agent_routes/GbgSaroRound.xml
TRAJ_FILES="../data/GbgSaroRound_DT/*Car.csv"
TRAJ_OFFSET_X=678710.40
TRAJ_OFFSET_Y=6374600.42
ON_NPC_BEHAVIOR_AGENT_ROUTE_DONE=roam
SEED=""
NO_Z_CHECK=false

# ── Argument parsing ───────────────────────────────────────────────────────────
while [[ $# -gt 0 ]]; do
  case "$1" in
    --agent)        AGENT=true ;;
    --npcs)         NPCS=true ;;
    --npc_movement)     NPC_MOVEMENT="$2";         shift ;;
    --npc_behavior)     NPC_BEHAVIOR="$2";         shift ;;
    --display_camera)  DISPLAY_CAMERA="$2";      shift ;;
    --record_cameras) RECORD_CAMERAS=""; while [[ $# -gt 1 && "${2}" != --* ]]; do RECORD_CAMERAS="$RECORD_CAMERAS $2"; shift; done ;;
    --overhead_camera_position) OVERHEAD_POS="$2 $3 $4"; shift 3 ;;
    --duration)     DURATION="$2";         shift ;;
    --offset_time)  OFFSET_TIME="$2";      shift ;;
    --timestep)     TIMESTEP="$2";         shift ;;
    --map)          MAP="$2";              shift ;;
    --ego_agent)   EGO_AGENT="$2";       shift ;;
    --ego_behavior)   EGO_BEHAVIOR="$2";       shift ;;
    --on_ego_behavior_agent_route_done) ON_EGO_BEHAVIOR_AGENT_ROUTE_DONE="$2"; shift ;;
    --ego_route_filepath)   EGO_ROUTE_FILEPATH="$2";       shift ;;
    --traj_files)   TRAJ_FILES="$2";       shift ;;
    --npc_trajectory_offset)  TRAJ_OFFSET_X="$2"; TRAJ_OFFSET_Y="$3"; shift 2 ;;
    --on_npc_behavior_agent_route_done) ON_NPC_BEHAVIOR_AGENT_ROUTE_DONE="$2"; shift ;;
    --seed)         SEED="$2";           shift ;;
    --no_z_check)   NO_Z_CHECK=true ;;
    *) echo "Unknown argument: $1"; exit 1 ;;
  esac
  shift
done

if [[ "$AGENT" == false && "$NPCS" == false ]]; then
  echo "ERROR: at least one of --agent or --npcs must be specified." >&2
  exit 1
fi

# ── Docker setup ───────────────────────────────────────────────────────────────
. helpers/docker-quick.sh

carla_docker_restart

# ── Build command ──────────────────────────────────────────────────────────────
export DIRECT=0

CMD=(
  python3 src/rirun/run.py
  "$DURATION"
  "$MAP"
  --timestep "$TIMESTEP"
  --display_camera "$DISPLAY_CAMERA"
  --offset_time "$OFFSET_TIME"
)

if [[ -n "$RECORD_CAMERAS" ]]; then
  # shellcheck disable=SC2086
  CMD+=( --record_cameras $RECORD_CAMERAS )
fi

if [[ -n "$OVERHEAD_POS" ]]; then
  # shellcheck disable=SC2086
  CMD+=( --overhead_camera_position $OVERHEAD_POS )
fi

if $AGENT; then
  CMD+=(
    --ego_agent "$EGO_AGENT"
    --ego_route_filepath "$EGO_ROUTE_FILEPATH"
  )
  if [[ "$EGO_AGENT" == "behavior_agent" ]]; then
    CMD+=( --ego_behavior "$EGO_BEHAVIOR" --on_ego_behavior_agent_route_done "$ON_EGO_BEHAVIOR_AGENT_ROUTE_DONE" )
  fi
fi

if $NPCS; then
  # shellcheck disable=SC2086
  CMD+=(
    --npc_trajectory_filepaths $TRAJ_FILES
    --npc_movement "$NPC_MOVEMENT"
    --npc_trajectory_offset "$TRAJ_OFFSET_X" "$TRAJ_OFFSET_Y"
  )
  if [[ "$NPC_MOVEMENT" == "behavior_agent" ]]; then
    CMD+=( --npc_behavior "$NPC_BEHAVIOR" --on_npc_behavior_agent_route_done "$ON_NPC_BEHAVIOR_AGENT_ROUTE_DONE" )
  fi
fi

if [[ -n "$SEED" ]]; then
  CMD+=( --seed "$SEED" )
fi

if $NO_Z_CHECK; then
  CMD+=( --no_z_check )
fi

echo "Running: ${CMD[*]}"
exec "${CMD[@]}"
