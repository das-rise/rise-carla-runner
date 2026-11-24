"""Unit tests for xodr_plot.carla_server (subprocess mocked — no docker)."""

import pytest
from xodr_plot import carla_server
from xodr_plot.carla_server import CarlaServerError, CarlaTestServer

# ── container name resolution ─────────────────────────────────────────────────


def test_resolve_container_name_explicit_wins(monkeypatch):
    monkeypatch.setenv("CARLA_CONTAINER", "env-name")
    assert (
        CarlaTestServer("map.xodr", container_name="explicit")._container_name
        == "explicit"
    )


def test_resolve_container_name_env_var(monkeypatch):
    monkeypatch.setenv("CARLA_CONTAINER", "env-name")
    assert CarlaTestServer("map.xodr")._container_name == "env-name"


def test_resolve_container_name_default(monkeypatch):
    monkeypatch.delenv("CARLA_CONTAINER", raising=False)
    assert (
        CarlaTestServer("map.xodr")._container_name == carla_server.USER_CONTAINER_NAME
    )


# ── fake docker ───────────────────────────────────────────────────────────────


class _FakeCompleted:
    def __init__(self, returncode=0, stdout="", stderr=""):
        self.returncode = returncode
        self.stdout = stdout
        self.stderr = stderr


def _fake_docker(monkeypatch, state):
    """Stateful fake for the docker calls of the container lifecycle.

    ``state`` keys: exists, running, port (str).
    Returns the list of docker commands issued.
    """
    calls = []

    def fake_run(cmd, *args, **kwargs):
        calls.append(cmd)
        if cmd[:2] == ["docker", "inspect"]:
            fmt = cmd[cmd.index("-f") + 1]
            if "{{.Id}}" in fmt:
                return _FakeCompleted(
                    0 if state["exists"] else 1,
                    "cid\n" if state["exists"] else "",
                )
            if "HostConfig.PortBindings" in fmt:
                return _FakeCompleted(0, state["port"] + "\n")
            if "{{.State.Running}}" in fmt:
                return _FakeCompleted(0, "true\n" if state["running"] else "false\n")
            return _FakeCompleted(0, "true\n")
        if cmd[:2] == ["docker", "start"]:
            if state.get("start_rc", 0) != 0:
                return _FakeCompleted(state["start_rc"], "", "cannot start container")
            state["running"] = True
            return _FakeCompleted(0)
        if cmd[:2] == ["docker", "stop"]:
            state["running"] = False
            return _FakeCompleted(0)
        return _FakeCompleted(0)

    monkeypatch.setattr(carla_server.subprocess, "run", fake_run)
    return calls


def _live_server():
    server = CarlaTestServer("map.xodr")
    server._container = server._container_name
    server._port = 2000
    server._client = type(
        "C",
        (),
        {
            "get_server_version": lambda self: "0.9.15",
            "set_timeout": lambda self, t: None,
        },
    )()
    server._world = type("W", (), {"get_map": lambda self: "MAP"})()
    server._map = "MAP"
    return server


def _start_env(monkeypatch):
    monkeypatch.setattr(carla_server, "RETRY_DELAY_S", 0)


# ── _alive / ensure_ready ─────────────────────────────────────────────────────


def test_alive_true(monkeypatch):
    _fake_docker(monkeypatch, {"exists": True, "running": True, "port": "2000"})
    assert _live_server()._alive()


def test_alive_false_when_container_dead(monkeypatch):
    _fake_docker(monkeypatch, {"exists": True, "running": False, "port": "2000"})
    assert not _live_server()._alive()


def test_alive_false_when_client_lost(monkeypatch):
    _fake_docker(monkeypatch, {"exists": True, "running": True, "port": "2000"})
    server = _live_server()

    def _raise(self):
        raise RuntimeError("connection refused")

    server._client = type(
        "C",
        (),
        {"get_server_version": _raise, "set_timeout": lambda self, t: None},
    )()
    assert not server._alive()


def test_ensure_ready_reuses_live_server(monkeypatch):
    _fake_docker(monkeypatch, {"exists": True, "running": True, "port": "2000"})
    server = _live_server()
    messages = []
    server._start = pytest.fail  # must not restart
    assert server.ensure_ready(progress_cb=messages.append) == "MAP"
    assert any("Reusing" in m for m in messages)


def test_ensure_ready_restarts_dead_server(monkeypatch):
    _fake_docker(monkeypatch, {"exists": True, "running": False, "port": "2000"})
    server = _live_server()
    restarted = []

    def fake_start(progress_cb):
        restarted.append(True)
        server._map = "NEWMAP"

    server._start = fake_start
    assert server.ensure_ready() == "NEWMAP"
    assert restarted == [True]


# ── teardown / SIGINT ─────────────────────────────────────────────────────────


def test_teardown_leaves_container_running(monkeypatch):
    state = {"exists": True, "running": True, "port": "2000"}
    calls = _fake_docker(monkeypatch, state)
    server = _live_server()
    server.teardown()
    server.teardown()
    assert state["running"] is True  # the container is the user's
    assert not [c for c in calls if c[:2] in (["docker", "kill"], ["docker", "rm"])]
    assert not [c for c in calls if c[:2] == ["docker", "stop"]]


# ── _start ────────────────────────────────────────────────────────────────────


def test_start_fails_without_docker(monkeypatch):
    _fake_docker(monkeypatch, {"exists": True, "running": False, "port": "2000"})
    monkeypatch.setattr(carla_server, "_docker_available", lambda: False)
    server = CarlaTestServer("map.xodr")
    with pytest.raises(CarlaServerError, match="docker is not available"):
        server._start(None)


def test_start_fails_when_container_missing(monkeypatch):
    _start_env(monkeypatch)
    _fake_docker(monkeypatch, {"exists": False, "running": False, "port": "2000"})
    server = CarlaTestServer("map.xodr")
    with pytest.raises(CarlaServerError, match="not found"):
        server._start(None)


def test_start_fails_without_published_port(monkeypatch):
    _start_env(monkeypatch)
    _fake_docker(monkeypatch, {"exists": True, "running": False, "port": ""})
    server = CarlaTestServer("map.xodr")
    with pytest.raises(CarlaServerError, match="no published"):
        server._start(None)


def test_start_starts_stopped_container(monkeypatch):
    _start_env(monkeypatch)
    state = {"exists": True, "running": False, "port": "2000"}
    calls = _fake_docker(monkeypatch, state)
    server = CarlaTestServer("map.xodr")
    grace = []

    def fake_wait(carla_mod, port, cb, apply_grace):
        grace.append(apply_grace)
        server._client = "CLIENT"

    server._wait_for_server = fake_wait
    server._load_map = lambda carla_mod, cb: setattr(server, "_map", "MAP")
    assert server.ensure_ready() == "MAP"
    assert [c for c in calls if c[:2] == ["docker", "start"]] == [
        ["docker", "start", carla_server.USER_CONTAINER_NAME]
    ]
    assert grace == [True]  # we started it → full grace period
    assert server._container == carla_server.USER_CONTAINER_NAME
    assert server.port == 2000


def test_start_connects_to_running_container(monkeypatch):
    _start_env(monkeypatch)
    state = {"exists": True, "running": True, "port": "2000"}
    calls = _fake_docker(monkeypatch, state)
    server = CarlaTestServer("map.xodr")
    grace = []

    def fake_wait(carla_mod, port, cb, apply_grace):
        grace.append(apply_grace)
        server._client = "CLIENT"

    server._wait_for_server = fake_wait
    server._load_map = lambda carla_mod, cb: setattr(server, "_map", "MAP")
    assert server.ensure_ready() == "MAP"
    assert not [c for c in calls if c[:2] == ["docker", "start"]]
    assert grace == [False]  # already running → check immediately


def test_start_fails_when_docker_start_fails(monkeypatch):
    _start_env(monkeypatch)
    state = {"exists": True, "running": False, "port": "2000", "start_rc": 125}
    _fake_docker(monkeypatch, state)
    server = CarlaTestServer("map.xodr")
    with pytest.raises(CarlaServerError, match="docker start failed"):
        server._start(None)


# ── _start retry behaviour ────────────────────────────────────────────────────


def test_start_retries_after_deadlock(monkeypatch):
    _start_env(monkeypatch)
    state = {"exists": True, "running": False, "port": "2000"}
    calls = _fake_docker(monkeypatch, state)
    server = CarlaTestServer("map.xodr")
    n = {"wait": 0}

    def flaky_wait(carla_mod, port, cb, apply_grace):
        n["wait"] += 1
        if n["wait"] == 1:
            raise CarlaServerError("engine deadlocked during initialisation")

    server._wait_for_server = flaky_wait
    server._load_map = lambda carla_mod, cb: setattr(server, "_map", "MAP")
    assert server.ensure_ready() == "MAP"
    # Attempt 1 starts the container, deadlocks, and _discard_quietly stops it
    # (fresh engine boot for attempt 2); the container is never removed.
    assert len([c for c in calls if c[:2] == ["docker", "start"]]) == 2
    assert len([c for c in calls if c[:2] == ["docker", "stop"]]) == 1
    assert not [c for c in calls if c[:2] == ["docker", "rm"]]


def test_start_gives_up_after_max_attempts(monkeypatch):
    _start_env(monkeypatch)
    state = {"exists": True, "running": False, "port": "2000"}
    calls = _fake_docker(monkeypatch, state)
    server = CarlaTestServer("map.xodr")

    def always_dead(carla_mod, port, cb, apply_grace):
        raise CarlaServerError("engine deadlocked during initialisation")

    server._wait_for_server = always_dead
    server._load_map = lambda carla_mod, cb: None
    with pytest.raises(
        CarlaServerError, match=f"after {carla_server.MAX_START_ATTEMPTS} attempts"
    ):
        server._start(None)
    assert (
        len([c for c in calls if c[:2] == ["docker", "start"]])
        == carla_server.MAX_START_ATTEMPTS
    )
    # Each failed attempt stops the container for a fresh next attempt;
    # the container itself is the user's and is never removed.
    assert (
        len([c for c in calls if c[:2] == ["docker", "stop"]])
        == carla_server.MAX_START_ATTEMPTS
    )
    assert not [c for c in calls if c[:2] == ["docker", "rm"]]
    assert state["running"] is False  # left stopped, never running on exit
