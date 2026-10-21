#!/bin/bash

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd "$script_dir/.." && pwd)"
env_file="$project_root/.env"

CARLA_ROOT=""
PYTHON_VERSION="3.8"
if [[ -f "$env_file" ]]; then
    source "$env_file"
fi

CONTAINER_NAME="CARLA-0.9.15"

# Function to display usage
usage() {
	echo "Usage: $0 [path/to/cooked_map.tar.gz]"
	echo "Set CARLA_ROOT in $env_file to mount a locally built CARLA package."
	echo "CARLA_ROOT should contain CarlaUE4.sh and PythonAPI/carla/dist/*.egg."
	echo "A map archive can only be given when CARLA_ROOT is empty."
}

map_archive=""
if [[ -n "$1" ]]; then
    if [[ -n "$CARLA_ROOT" ]]; then
        echo "Warning: a map archive was given, but CARLA_ROOT is set in $env_file. Clear CARLA_ROOT to import a map." >&2
        exit 1
    fi
    if ! [[ -f "$1" ]]; then
        echo "Map archive not found: $1" >&2
        exit 1
    fi
    map_archive="$(realpath "$1")"
fi

containerid=$(docker ps | grep carla | awk ' { print $1 } ')
if ! [[ -z "$containerid" ]]; then
    trap 'echo; echo "Exiting."; exit 130' INT
    printf "Found running Carla container(s): %s\n" "$containerid"
    read -p "Choose: [k]ill existing and start new, [s]tart another, or [e]xit? [k/s/e]: " response
    case $response in
        [kK]*)
            docker kill $containerid > /dev/null 2>&1
            docker rm $containerid > /dev/null 2>&1
            echo "Existing container(s) killed."
            ;;
        [eE]*)
            echo "Exiting."
            exit 0
            ;;
        *)
            echo "Starting new container..."
            ;;
    esac
fi

# build the container
docker build -t carla-synergies-0.9.15 -f "$script_dir/Dockerfile" "$script_dir" || { echo "Errors during docker build, exiting."; exit 1; }

if ! [[ -z "$CARLA_ROOT" ]]; then # if using a locally built CARLA package instead of the container
    if ! [[ -f "$CARLA_ROOT/CarlaUE4.sh" ]]; then
        echo "Could not find CarlaUE4.sh in CARLA_ROOT: $CARLA_ROOT" >&2
        exit 1
    fi
    docker run -d --privileged \
        --gpus=all -p 2000-2002:2000-2002 \
        -v "$CARLA_ROOT:/opt/carla-custom:rw" \
        --tmpfs /opt/carla-custom/CarlaUE4/Content/Carla/Maps/OpenDrive:uid=1000,gid=1000,mode=775 \
        --tmpfs /opt/carla-custom/CarlaUE4/Content/Carla/Maps/Nav:uid=1000,gid=1000,mode=775 \
        -e CARLA_ROOT=/opt/carla-custom \
        --name "$CONTAINER_NAME" \
        carla-synergies-0.9.15 \
        /bin/bash /opt/carla-custom/CarlaUE4.sh -RenderOffScreen
else # using the container directly
    map_args=()
    start_cmd="exec ./CarlaUE4.sh -RenderOffScreen"
    if [[ -n "$map_archive" ]]; then
        map_name="$(basename "$map_archive")"
        map_args=(-v "$map_archive:/tmp/map-import/$map_name:ro")
        # import, print a marker line with the result, and only start CARLA if the import worked
        start_cmd="cp \"/tmp/map-import/$map_name\" Import/ && ./ImportAssets.sh; rc=\$?; "
        start_cmd+="if [ \$rc -ne 0 ]; then echo \"[map-import] FAILED (exit code \$rc)\"; exit \$rc; fi; "
        start_cmd+="echo \"[map-import] DONE: $map_name imported\"; exec ./CarlaUE4.sh -RenderOffScreen -stdout -FullStdOutLogOutput"
    fi
    docker rm "$CONTAINER_NAME" >/dev/null 2>&1  # remove a stopped leftover with the same name
    docker run -d --privileged \
        --gpus=all -p 2000-2002:2000-2002 \
        "${map_args[@]}" \
        --name "$CONTAINER_NAME" \
        carla-synergies-0.9.15 \
        /bin/bash -c "cd /home/carla && $start_cmd" || exit 1
fi

# report the map import result on the host
if [[ -n "$map_archive" && -z "$CARLA_ROOT" ]]; then
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
