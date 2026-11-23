"""
carla_server.py — Session-scoped CARLA test server (reuses the user's container).

Instead of spawning its own throwaway container, xodr-plot uses the user's
regular CARLA container — the same one the rirun experiments run against:

  * First route test  → look up the CARLA container (``CARLA-0.9.15`` by
                        default, ``CARLA_CONTAINER`` env var or
                        ``--carla-container`` to override); start it with
                        ``docker start`` if it is stopped; wait for the
                        server; load the xodr map (this replaces the
                        container's world, exactly like the rirun flow).
  * Later route tests → reuse the running container and the loaded map
                        (self-healing: if the container or server died in
                        between, it is started again transparently).
  * Tool exit         → the container is left as found/running: it is the
                        user's, with the user's lifecycle (no kill, no rm).

Why not spawn our own container: on this machine, fresh ``docker run``
containers of the CARLA image deadlock during engine initialisation with a
very high probability (the engine's render thread stalls inside the Vulkan
driver), while the user's long-lived container — started with
``docker start`` — boots reliably. Using the user's container is therefore
both the working path and the path the experiments already use.
"""

from __future__ import annotations

import os
import signal
import socket
import subprocess
import threading
import time
from typing import Callable, List, Optional

# The user's regular CARLA container (see docker/start-carla-docker.sh).
USER_CONTAINER_NAME = "CARLA-0.9.15"
# A healthy boot finishes in ~3–6 min (measured on this machine); a failed
# boot stalls indefinitely, so a 10-min per-attempt cutoff wastes little
# time on the good case and bounds the bad ones.
SERVER_START_TIMEOUT_S = 600
MAP_LOAD_TIMEOUT_S = 1200
# A failed boot (crash or stall) often succeeds on a fresh attempt.
MAX_START_ATTEMPTS = 3
RETRY_DELAY_S = 5.0
# After a (re)start, the engine's RPC port opens early but the
# world/render subsystem takes several more minutes to come up. During that
# window we do not touch the RPC at all (probing it during initialisation
# has been observed to deadlock the engine); only after this grace period
# do we run the single readiness check.
RPC_GRACE_PERIOD_S = 300


class CarlaServerError(RuntimeError):
    """Raised when the CARLA container cannot be started or used."""


def log(message: str) -> None:
    print(f"[xodr-plot] {message}", flush=True)


def _docker_available() -> bool:
    result = subprocess.run(
        ["docker", "version"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
    )
    return result.returncode == 0


def _tail_logs(name: str, lines: int = 40) -> str:
    """Last `lines` log lines of a (possibly exited) container."""
    result = subprocess.run(
        ["docker", "logs", "--tail", str(lines), name],
        capture_output=True,
        text=True,
    )
    text = (result.stdout + "\n" + result.stderr).strip()
    return text or "(no container logs)"


def _resolve_container_name(explicit: Optional[str]) -> str:
    return explicit or os.environ.get("CARLA_CONTAINER") or USER_CONTAINER_NAME


class CarlaTestServer:
    """Manages the session's use of the user's CARLA container."""

    def __init__(
        self,
        xodr_filepath: str,
        host: str = "localhost",
        container_name: Optional[str] = None,
    ):
        self._xodr = xodr_filepath
        self._host = host
        self._container_name = _resolve_container_name(container_name)
        self._container: Optional[str] = None
        self._port: Optional[int] = None
        self._client = None
        self._world = None
        self._map = None
        self._shutdown_done = False
        self._interrupted = False
        self._prev_sigint = None
        try:
            # Only possible from the main thread (both entry points create
            # the server there); embedded/other-thread use simply skips it.
            self._prev_sigint = signal.signal(signal.SIGINT, self._handle_sigint)
        except ValueError:
            pass

    # ── State ────────────────────────────────────────────────────────────────

    @property
    def ready(self) -> bool:
        return self._map is not None

    @property
    def container_name(self) -> Optional[str]:
        return self._container

    @property
    def port(self) -> Optional[int]:
        return self._port

    def _progress(
        self, progress_cb: Optional[Callable[[str], None]], message: str
    ) -> None:
        if progress_cb is not None:
            progress_cb(message)

    @staticmethod
    def _docker_inspect(name: str, fmt: str) -> Optional[str]:
        result = subprocess.run(
            ["docker", "inspect", "-f", fmt, name],
            capture_output=True,
            text=True,
        )
        if result.returncode != 0:
            return None
        return result.stdout.strip()

    def _container_exists(self) -> bool:
        return self._docker_inspect(self._container_name, "{{.Id}}") is not None

    def _container_running(self) -> bool:
        return (
            self._docker_inspect(self._container_name, "{{.State.Running}}") == "true"
        )

    def _container_port(self) -> Optional[int]:
        """The host port of the container's RPC (container) port 2000."""
        raw = self._docker_inspect(
            self._container_name,
            "{{range $p, $conf := .HostConfig.PortBindings}}"
            '{{if eq $p "2000/tcp"}}{{(index $conf 0).HostPort}}{{end}}'
            "{{end}}",
        )
        if raw is None:
            return None
        try:
            return int(raw)
        except ValueError:
            return None

    def _alive(self) -> bool:
        """True if the container, server and loaded map are all still usable."""
        if self._container is None or self._client is None or self._world is None:
            return False
        if not self._container_running():
            return False
        try:
            # The client may carry a long timeout from the map import — the
            # liveness check must not hang.
            self._client.set_timeout(5.0)
            self._client.get_server_version()
            return self._world.get_map() is not None
        except Exception:
            return False

    # ── Public API ───────────────────────────────────────────────────────────

    def ensure_ready(
        self, progress_cb: Optional[Callable[[str], None]] = None
    ) -> "object":
        """Return the loaded carla.Map, starting the container if needed."""
        if self._alive():
            self._progress(progress_cb, "Reusing running CARLA container…")
            return self._map

        # A half-dead session (container gone or server unreachable) is
        # discarded and the container is (re)started.
        self._discard_quietly()
        self._start(progress_cb)
        return self._map

    def teardown(self) -> None:
        """Finish the session. The container is the user's and is left alone."""
        if self._shutdown_done:
            return
        self._shutdown_done = True
        if self._prev_sigint is not None:
            try:
                signal.signal(signal.SIGINT, self._prev_sigint)
            except (TypeError, ValueError):
                pass
            self._prev_sigint = None
        if self._container is not None and self._container_running():
            log(
                f"Leaving CARLA container '{self._container}' running "
                f"(it is your container — stop it with "
                f"`docker stop {self._container}` to free the GPU)."
            )

    def _handle_sigint(self, signum: int, frame: "object") -> None:
        """Ctrl+C: exit immediately, leaving the user's container untouched."""
        self._interrupted = True
        log("Interrupted — leaving the CARLA container as is.")
        os._exit(130)

    # ── Internals ────────────────────────────────────────────────────────────

    def _discard_quietly(self) -> None:
        """Drop the in-use session state without user-facing logging.

        If the container is still running (e.g. the engine crashed or
        deadlocked), stop it so the next attempt gets a fresh engine boot.
        The container itself is kept — it is the user's.
        """
        self._client = None
        self._world = None
        self._map = None
        name, self._container = self._container, None
        self._port = None
        if (
            name is not None
            and self._docker_inspect(name, "{{.State.Running}}") == "true"
        ):
            subprocess.run(
                ["docker", "stop", "-t", "10", name],
                capture_output=True,
                timeout=90,
            )

    def _start(self, progress_cb: Optional[Callable[[str], None]]) -> None:
        """Start (if needed) the user's CARLA container, wait, load the map.

        The full cycle is retried up to MAX_START_ATTEMPTS times: a failed
        boot (crash or stall) often succeeds on a fresh attempt, so a
        failed attempt stops the container and the next one restarts it.
        """
        import carla  # local import: keep this module usable without the extra

        if not _docker_available():
            raise CarlaServerError("docker is not available — is Docker running?")

        if not self._container_exists():
            raise CarlaServerError(
                f"CARLA container '{self._container_name}' not found — create it "
                "once with `bash docker/start-carla-docker.sh`, or point at "
                "another container with --carla-container / CARLA_CONTAINER"
            )

        port = self._container_port()
        if port is None:
            raise CarlaServerError(
                f"CARLA container '{self._container_name}' has no published "
                "RPC port (2000/tcp) — was it created with -p 2000:2000?"
            )
        self._port = port

        failures: List[str] = []
        for attempt in range(1, MAX_START_ATTEMPTS + 1):
            if self._interrupted:
                raise CarlaServerError("interrupted")
            attempt_note = (
                f" (attempt {attempt}/{MAX_START_ATTEMPTS})"
                if MAX_START_ATTEMPTS > 1
                else ""
            )
            started_by_us = not self._container_running()
            if started_by_us:
                self._progress(
                    progress_cb,
                    f"Starting CARLA container '{self._container_name}' "
                    f"(port {port})…{attempt_note}",
                )
                result = subprocess.run(
                    ["docker", "start", self._container_name],
                    capture_output=True,
                    text=True,
                )
                if result.returncode != 0:
                    raise CarlaServerError(
                        f"docker start failed:\n{result.stderr.strip()}"
                    )
            else:
                self._progress(
                    progress_cb,
                    f"Connecting to running CARLA container "
                    f"'{self._container_name}' (port {port})…{attempt_note}",
                )
            try:
                self._wait_for_server(
                    carla, port, progress_cb, apply_grace=started_by_us
                )
                self._load_map(carla, progress_cb)
                self._container = self._container_name
                return
            except CarlaServerError as exc:
                self._discard_quietly()
                if started_by_us and self._container_running():
                    # A failed attempt leaves a half-initialised (usually
                    # deadlocked) engine behind — stop it so the next
                    # attempt boots fresh.
                    subprocess.run(
                        ["docker", "stop", "-t", "10", self._container_name],
                        capture_output=True,
                        timeout=90,
                    )
                failures.append(f"attempt {attempt}: {exc}")
                if attempt < MAX_START_ATTEMPTS:
                    self._progress(
                        progress_cb, "CARLA server failed to come up — retrying…"
                    )
                    time.sleep(RETRY_DELAY_S)

        raise CarlaServerError(
            f"CARLA server did not come up after {MAX_START_ATTEMPTS} attempts "
            f"(container '{self._container_name}'):\n" + "\n".join(failures)
        )

    def _port_open(self, port: int) -> bool:
        try:
            with socket.create_connection((self._host, port), timeout=2.0):
                return True
        except OSError:
            return False

    def _wait_for_server(
        self,
        carla: "object",
        port: int,
        progress_cb: Optional[Callable[[str], None]],
        apply_grace: bool,
    ) -> None:
        """Wait until the engine's world subsystem answers.

        If we (re)started the engine (``apply_grace``), the RPC port opens
        early in initialisation while the world subsystem is still minutes
        away; touching the RPC during that window has been observed to
        deadlock the engine, so after the port opens we wait out the grace
        period and then do exactly one full check (version + world). If the
        container was already running, it is either ready or dead, so the
        check runs immediately.
        """
        name = self._container_name
        self._progress(progress_cb, "Waiting for CARLA server…")
        deadline = time.time() + SERVER_START_TIMEOUT_S
        port_open_since: Optional[float] = None
        while True:
            if self._docker_inspect(name, "{{.State.Running}}") != "true":
                raise CarlaServerError(
                    "CARLA container exited during startup\n"
                    f"Last container log lines:\n{_tail_logs(name)}"
                )
            if time.time() > deadline:
                raise CarlaServerError(
                    f"CARLA server did not come up within {SERVER_START_TIMEOUT_S} s\n"
                    f"Last container log lines:\n{_tail_logs(name)}"
                )
            if port_open_since is None:
                if not self._port_open(port):
                    time.sleep(5.0)
                    continue
                port_open_since = time.time()
                if apply_grace:
                    self._progress(
                        progress_cb,
                        "RPC port open — letting the engine finish initialising…",
                    )
                    # The world subsystem is still minutes away; touching the
                    # RPC during that window (even a bare TCP connection) has
                    # been observed to deadlock the engine, so wait it out
                    # without touching the port at all.
                    while time.time() - port_open_since < RPC_GRACE_PERIOD_S:
                        if self._docker_inspect(name, "{{.State.Running}}") != "true":
                            raise CarlaServerError(
                                "CARLA container exited during startup\n"
                                f"Last container log lines:\n{_tail_logs(name)}"
                            )
                        if time.time() > deadline:
                            raise CarlaServerError(
                                f"CARLA server did not come up within "
                                f"{SERVER_START_TIMEOUT_S} s\n"
                                f"Last container log lines:\n{_tail_logs(name)}"
                            )
                        time.sleep(5.0)
            client = carla.Client(self._host, port)
            client.set_timeout(10.0)
            try:
                # Readiness means the world map actually answers, not just
                # the version RPC (a deadlocked engine answers the latter
                # while the world subsystem stays dead).
                client.get_server_version()
                if client.get_world().get_map() is not None:
                    self._client = client
                    self._progress(progress_cb, "CARLA server ready.")
                    return
            except Exception:
                pass
            raise CarlaServerError(
                "engine deadlocked during initialisation (RPC port opened "
                "but the world subsystem never came up — a known CARLA "
                "0.9.15 startup stall; retrying with a fresh engine)\n"
                f"Last container log lines:\n{_tail_logs(name)}"
            )

    def _load_map(
        self,
        carla: "object",
        progress_cb: Optional[Callable[[str], None]],
    ) -> None:
        assert self._client is not None
        self._progress(progress_cb, "Loading map…")
        with open(self._xodr, "r") as fh:
            xodr_text = fh.read()
        # Map import is a long-running *blocking* RPC (it only returns once
        # the map is fully generated), so the short handshake timeout must
        # not apply to it — and some maps make the engine crash mid-import,
        # which a blocking call cannot detect on its own. Run it in a worker
        # thread and watch the container.
        self._client.set_timeout(float(MAP_LOAD_TIMEOUT_S))
        outcome: dict = {}

        def _do_import() -> None:
            try:
                outcome["world"] = self._client.generate_opendrive_world(
                    xodr_text,
                    carla.OpendriveGenerationParameters(
                        wall_height=0, smooth_junctions=False, additional_width=0
                    ),
                )
            except Exception as exc:
                outcome["error"] = exc

        worker = threading.Thread(target=_do_import, daemon=True)
        worker.start()
        deadline = time.time() + MAP_LOAD_TIMEOUT_S
        while worker.is_alive():
            if not self._container_running():
                raise CarlaServerError(
                    "CARLA container exited while loading the map — "
                    "the engine crashed during import. Some OpenDRIVE maps "
                    "crash the CARLA 0.9.15 importer; if this is a DTT map, "
                    "try the processed/fixed variant used by the scenes\n"
                    f"Last container log lines:\n{_tail_logs(self._container_name)}"
                )
            if time.time() > deadline:
                raise CarlaServerError(
                    f"map did not finish loading within {MAP_LOAD_TIMEOUT_S} s\n"
                    f"Last container log lines:\n{_tail_logs(self._container_name)}"
                )
            time.sleep(1.0)
        worker.join(timeout=5.0)
        if "error" in outcome:
            raise CarlaServerError(
                f"map import failed: {outcome['error']}\n"
                f"Last container log lines:\n{_tail_logs(self._container_name)}"
            )
        world = outcome["world"]
        deadline = time.time() + MAP_LOAD_TIMEOUT_S
        while True:
            if not self._container_running():
                raise CarlaServerError(
                    "CARLA container exited while loading the map\n"
                    f"Last container log lines:\n{_tail_logs(self._container_name)}"
                )
            if time.time() > deadline:
                raise CarlaServerError(
                    f"map did not finish loading within {MAP_LOAD_TIMEOUT_S} s\n"
                    f"Last container log lines:\n{_tail_logs(self._container_name)}"
                )
            try:
                if world.get_map() is not None:
                    break
            except Exception:
                pass  # RPC hiccup; the checks above catch a dead container
            time.sleep(0.5)

        self._world = world
        self._map = world.get_map()
        self._progress(progress_cb, "Map loaded, route planner ready.")
