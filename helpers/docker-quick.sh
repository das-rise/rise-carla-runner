#!/usr/bin/env bash

DOCKER_SCRIPTS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../docker" && pwd)"

carla_docker_start() {
    bash "$DOCKER_SCRIPTS_DIR/start-carla-docker.sh" "$@"
    sleep 5
}

carla_docker_stop() {
    bash "$DOCKER_SCRIPTS_DIR/stop-carla-docker.sh" --force
    sleep 5
}

carla_docker_restart() {
    carla_docker_stop
    carla_docker_start
}
