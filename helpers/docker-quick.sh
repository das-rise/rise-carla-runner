#!/usr/bin/env bash

DOCKER_SCRIPTS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../docker" && pwd)"
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOCAL_CARLA_PIDFILE="$PROJECT_ROOT/.carla_local.pid"
LOCAL_CARLA_LOGFILE="$PROJECT_ROOT/.carla_local.log"

load_project_env_if_present() {
    if [[ -z "${CARLA_ROOT:-}" && -f "$PROJECT_ROOT/.env" ]]; then
        set -a
        # shellcheck source=/dev/null
        . "$PROJECT_ROOT/.env"
        set +a
    fi
}

should_use_local_carla() {
    [[ -n "${CARLA_ROOT:-}" ]]
}

is_port_2000_in_use() {
    ss -ltn "sport = :2000" | grep -q ":2000"
}

stop_untracked_local_carla_processes() {
    local pids
    pids="$(pgrep -f 'CarlaUE4-Linux-Shipping|CarlaUE5-Linux-Shipping|CarlaUE4.sh|CarlaUE5.sh' | sort -u || true)"
    if [[ -z "$pids" ]]; then
        return 0
    fi

    local killed_any=false
    while IFS= read -r pid; do
        [[ -z "$pid" ]] && continue
        local args
        args="$(ps -p "$pid" -o args= 2>/dev/null || true)"
        if [[ -n "$args" ]] && [[ "$args" == *"$CARLA_ROOT"* ]]; then
            echo "Stopping untracked local CARLA process (PID $pid)."
            kill -KILL "$pid" 2>/dev/null || true
            killed_any=true
        fi
    done <<< "$pids"

    return 0
}

get_local_carla_launcher() {
    if [[ -x "$CARLA_ROOT/CarlaUE4.sh" ]]; then
        echo "$CARLA_ROOT/CarlaUE4.sh"
        return 0
    fi
    if [[ -x "$CARLA_ROOT/CarlaUE5.sh" ]]; then
        echo "$CARLA_ROOT/CarlaUE5.sh"
        return 0
    fi
    return 1
}

start_local_carla() {
    local launcher
    launcher="$(get_local_carla_launcher)" || {
        echo "CARLA_ROOT is set to '$CARLA_ROOT' but no launcher was found (expected CarlaUE4.sh or CarlaUE5.sh)." >&2
        return 1
    }

    if [[ -f "$LOCAL_CARLA_PIDFILE" ]]; then
        local existing_pid
        existing_pid="$(cat "$LOCAL_CARLA_PIDFILE" 2>/dev/null || true)"
        if [[ -n "$existing_pid" ]] && kill -0 "$existing_pid" 2>/dev/null; then
            echo "Local CARLA already running with PID $existing_pid."
            return 0
        fi
        rm -f "$LOCAL_CARLA_PIDFILE"
    fi

    stop_untracked_local_carla_processes

    echo "Starting local CARLA from $launcher"
    nohup "$launcher" -RenderOffScreen >"$LOCAL_CARLA_LOGFILE" 2>&1 &
    local pid=$!
    echo "$pid" >"$LOCAL_CARLA_PIDFILE"
    sleep 5
    if kill -0 "$pid" 2>/dev/null; then
        echo "Local CARLA started (PID $pid). Log: $LOCAL_CARLA_LOGFILE"
        return 0
    fi

    echo "Failed to start local CARLA. Check log: $LOCAL_CARLA_LOGFILE" >&2
    if is_port_2000_in_use; then
        echo "Port 2000 owner: $(port_2000_owner_line)" >&2
    fi
    rm -f "$LOCAL_CARLA_PIDFILE"
    return 1
}

stop_local_carla() {
    if [[ ! -f "$LOCAL_CARLA_PIDFILE" ]]; then
        stop_untracked_local_carla_processes
        return 0
    fi

    local pid
    pid="$(cat "$LOCAL_CARLA_PIDFILE" 2>/dev/null || true)"
    if [[ -z "$pid" ]]; then
        rm -f "$LOCAL_CARLA_PIDFILE"
        return 0
    fi

    if ! kill -0 "$pid" 2>/dev/null; then
        rm -f "$LOCAL_CARLA_PIDFILE"
        echo "Local CARLA PID $pid is not running."
        return 0
    fi

    echo "Stopping local CARLA (PID $pid)."
    kill -KILL "$pid" 2>/dev/null || true
    rm -f "$LOCAL_CARLA_PIDFILE"
    stop_untracked_local_carla_processes
    return 0
}

carla_docker_start() {
    load_project_env_if_present
    if should_use_local_carla; then
        start_local_carla
        return $?
    fi
    bash "$DOCKER_SCRIPTS_DIR/start-carla-docker.sh" "$@"
    sleep 5
}

carla_docker_stop() {
    load_project_env_if_present
    if should_use_local_carla; then
        stop_local_carla
        return $?
    fi
    bash "$DOCKER_SCRIPTS_DIR/stop-carla-docker.sh" --force
    sleep 5
}

carla_docker_restart() {
    carla_docker_stop
    stop_untracked_local_carla_processes
    carla_docker_start
}
