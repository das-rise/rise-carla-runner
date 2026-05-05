#!/bin/bash

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd "$script_dir/.." && pwd)"
env_file="$project_root/.env"

CARLA_ROOT=""
PYTHON_VERSION="3.8"
if [[ -f "$env_file" ]]; then
    source "$env_file"
fi

# Function to display usage
usage() {
	echo "Usage: $0 [--ue5]"
	echo "Choose --fixed-time for repeatable simulations that"
	echo "simulate the world at FPS frames per second. Default: FPS=10."
	echo "--ue5 will start a Carla 0.10 container that may not be fully compatible,"
	echo "but uses the latest graphics."
	echo "Set CARLA_ROOT in $env_file to mount a locally built CARLA package."
	echo "CARLA_ROOT should contain CarlaUE4.sh and PythonAPI/carla/dist/*.egg."
}


# Initialize variables
fixed_time=false
fps=10
extra_command_args=""


# Check for arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --ue5)
            ue5=true
            shift
            ;;
        *)
            echo "Invalid option: $1" >&2
            usage
            exit 1
            ;;
    esac
done


containerid=$(docker ps | grep carla | awk ' { print $1 } ')
if ! [[ -z "$containerid" ]]; then
    trap 'echo; echo "Exiting."; exit 130' INT
    printf "Found running Carla container(s): %s\n" "$containerid"
    read -p "Choose: [k]ill existing and start new, [s]tart another, or [e]xit? [k/s/e]: " response
    case $response in
        [kK]*)
            docker kill $containerid > /dev/null 2>&1
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

if  [ "$ue5" = true ]; then
    # use pre-built container for carla 0.10.0
    docker run -d --privileged \
        --gpus=all --net=host \
        carlasim/carla:0.10.0 \
        /bin/bash ./CarlaUE5.sh -RenderOffScreen
else
    # use our custom Dockerfile to include the scenario-runner in the container,
    # will use carla 0.9.15
    docker build -t carla-synergies-0.9.15 -f "$script_dir/Dockerfile" "$script_dir" || { echo "Errors during docker build, exiting."; exit 1; }
    if ! [[ -z "$CARLA_ROOT" ]]; then
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
            carla-synergies-0.9.15 \
            /bin/bash /opt/carla-custom/CarlaUE4.sh -RenderOffScreen
    else
        docker run -d --privileged \
            --gpus=all -p 2000-2002:2000-2002 \
            carla-synergies-0.9.15 \
            /bin/bash ./CarlaUE4.sh -RenderOffScreen
    fi
fi
