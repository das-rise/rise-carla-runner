#!/bin/bash


# Function to display usage
usage() {
	echo "Usage: $0 [--ue5]"
	echo "Choose --fixed-time for repeatable simulations that"
	echo "simulate the world at FPS frames per second. Default: FPS=10."
	echo "--ue5 will start a Carla 0.10 container that may not be fully compatible,"
	echo "but uses the latest graphics."
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
    docker build -t carla-synergies-0.9.15 -f "$(dirname "$0")/Dockerfile" "$(dirname "$0")" || { echo "Errors during docker build, exiting."; exit 1; }
    docker run -d --privileged \
	--gpus=all -p 2000-2002:2000-2002 \
	carla-synergies-0.9.15 \
	/bin/bash ./CarlaUE4.sh -RenderOffScreen
fi
