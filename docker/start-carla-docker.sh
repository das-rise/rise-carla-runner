#!/bin/bash

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd "$script_dir/.." && pwd)"
env_file="$project_root/.env"

CARLA_IMAGE=""
if [[ -f "$env_file" ]]; then
    source "$env_file"
fi

CONTAINER_NAME="CARLA-0.9.15"

# Pick the runtime image. Order:
#   1. CARLA_IMAGE from .env (explicit, trusted as-is)
#   2. carla-0.9.15:dtt-map-import (fixed engine, see build-carla-docker.sh)
#   3. official prebuilt CARLA, built from ./Dockerfile (fallback)
DTT_IMAGE="carla-0.9.15:dtt-map-import"
OFFICIAL_IMAGE="carla-synergies-0.9.15"
if [[ -n "$CARLA_IMAGE" ]]; then
    RUNTIME_IMAGE="$CARLA_IMAGE"
elif docker image inspect "$DTT_IMAGE" >/dev/null 2>&1; then
    RUNTIME_IMAGE="$DTT_IMAGE"
else
    echo "Note: ${DTT_IMAGE} not found, falling back to the official prebuilt CARLA image." >&2
    echo "WARNING: Digital Twin maps crash on the official image at map load (carla-simulator/carla#8696)." >&2
    echo "Build the fixed image with: bash docker/build-carla-docker.sh" >&2
    docker build -t "$OFFICIAL_IMAGE" -f "$script_dir/Dockerfile" "$script_dir" || { echo "Errors during docker build, exiting."; exit 1; }
    RUNTIME_IMAGE="$OFFICIAL_IMAGE"
fi

usage() {
    echo "Start a CARLA container with optional DTT map import."
    echo ""
    echo "Usage: $0 [path/to/cooked_map.tar.gz]"
    echo "       $0 --help"
    echo ""
    echo "Image selection (first match wins):"
    echo "  1. CARLA_IMAGE from $env_file (explicit override)"
    echo "  2. carla-0.9.15:dtt-map-import (built by docker/build-carla-docker.sh)"
    echo "  3. official carlasim/carla:0.9.15 via docker/Dockerfile (DTT maps"
    echo "     crash on it - carla-simulator/carla#8696)"
}

case "${1:-}" in
    --help|-h)
        usage
        exit 0
        ;;
esac

map_archive=""
if [[ -n "${1:-}" ]]; then
    if ! [[ -f "$1" ]]; then
        echo "Map archive not found: $1" >&2
        echo "Run with --help for usage." >&2
        exit 1
    fi
    map_archive="$(realpath "$1")"
fi

# Only match this project's server images, not e.g. carla-editor containers.
containerid=$(docker ps --format '{{.ID}} {{.Image}}' \
    | awk '$2 ~ /carla-0\.9\.15|carlasim\/carla|carla-synergies/ {print $1}')
if ! [[ -z "$containerid" ]]; then
    trap 'echo; echo "Exiting."; exit 130' INT
    printf "Found running Carla container(s): %s\n" "$containerid"
    read -p "Choose: [k]ill existing and start new (default), or [e]xit? [k/e]: " response
    case $response in
        "" | [kK]*)
            docker kill $containerid > /dev/null 2>&1
            docker rm $containerid > /dev/null 2>&1
            echo "Existing container(s) killed."
            ;;
        [eE]*)
            echo "Exiting."
            exit 0
            ;;
        *)
            echo "Invalid choice: '$response'" >&2
            usage >&2
            exit 1
            ;;
    esac
fi


map_args=()
# -NOLOGTOMEMORY: packaged (Shipping) games log to memory only by default.
#   (The packaged build may still not write a log file; the crash report below
#   is the reliable debugging artifact.)
# Crash reporting: callstack (Diagnostics.txt) + context in
#   /home/carla/.config/Epic/CarlaUE4/Saved/Crashes/crashinfo-CarlaUE4-pid-<pid>-<guid>/
#   Headless (no DISPLAY) makes UE treat the run as unattended, which skips the
#   whole crash-report block. Enable it two ways (either suffices):
#   - DISPLAY=:99 on docker run (dummy value; SDL just falls back to its dummy driver)
#   - bAgreeToCrashUpload=True appended to Engine.ini before the server starts
crash_cfg_cmd="printf '\\n[CrashReportClient]\\nbAgreeToCrashUpload=True\\n' >> ./CarlaUE4/Config/Engine.ini; "
start_cmd="${crash_cfg_cmd}exec ./CarlaUE4.sh -RenderOffScreen -NOLOGTOMEMORY"
if [[ -n "$map_archive" ]]; then
    map_name="$(basename "$map_archive")"
    map_args=(-v "$map_archive:/tmp/map-import/$map_name:ro")
    # import, print a marker line with the result, and only start CARLA if the import worked
    start_cmd="cp \"/tmp/map-import/$map_name\" Import/ && ./ImportAssets.sh; rc=\$?; "
    start_cmd+="if [ \$rc -ne 0 ]; then echo \"[map-import] FAILED (exit code \$rc)\"; exit \$rc; fi; "
    start_cmd+="echo \"[map-import] DONE: $map_name imported\"; ${crash_cfg_cmd}exec ./CarlaUE4.sh -RenderOffScreen -stdout -FullStdOutLogOutput -NOLOGTOMEMORY"
fi
docker rm "$CONTAINER_NAME" >/dev/null 2>&1  # remove a stopped leftover with the same name
docker run -d --privileged \
    --gpus=all -p 2000-2002:2000-2002 \
    "${map_args[@]}" \
    -e DISPLAY=:99 \
    --name "$CONTAINER_NAME" \
    "$RUNTIME_IMAGE" \
    /bin/bash -c "cd /home/carla && $start_cmd" || exit 1


# report the map import result on the host
if [[ -n "$map_archive" ]]; then
    echo "Importing $(basename "$map_archive")..."
    exec 3< <(docker logs -f "$CONTAINER_NAME" 2>&1)
    logs_pid=$!
    import_status="exited"
    result_line=""
    while IFS= read -r line <&3; do
        [[ -n "${VERBOSE:-}" ]] && printf '%s\n' "$line"
        case "$line" in
            "[map-import] DONE"*)   import_status="ok";     result_line="$line"; break ;;
            "[map-import] FAILED"*) import_status="failed"; result_line="$line"; break ;;
        esac
    done
    kill "$logs_pid" 2>/dev/null
    exec 3<&-
    case $import_status in
        ok)     echo "Map import succeeded: $(basename "$map_archive")"
                echo "CARLA is starting in container $CONTAINER_NAME. Follow the logs with: docker logs -f $CONTAINER_NAME" ;;
        failed) echo "Map import FAILED: ${result_line#\[map-import\] }" >&2
                echo "See the import output with: docker logs $CONTAINER_NAME" >&2
                exit 1 ;;
        *)      echo "Container exited before the import finished." >&2
                echo "See the output with: docker logs $CONTAINER_NAME" >&2
                exit 1 ;;
    esac
fi
