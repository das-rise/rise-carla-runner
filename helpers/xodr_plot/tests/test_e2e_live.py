"""Live end-to-end tests: run --test-route against the user's CARLA container.

Each test takes a few minutes (engine boot when the container is stopped +
map load + route trace) and needs docker with the NVIDIA toolkit, a GPU, and
the existing `CARLA-0.9.15` container (the one docker/start-carla-docker.sh
creates; override with CARLA_CONTAINER). Note: these tests replace the
container's world, so don't run a simulation on it while they run.

Run explicitly with:

    XODR_PLOT_LIVE_E2E=1 uv run pytest tests/test_e2e_live.py -v
"""

import os
import subprocess
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[3]
FIXTURES = Path(__file__).parent / "fixtures"

LIVE = os.environ.get("XODR_PLOT_LIVE_E2E") == "1"


def _docker_ok() -> bool:
    return subprocess.run(["docker", "version"], capture_output=True).returncode == 0


pytestmark = pytest.mark.skipif(
    not (LIVE and _docker_ok()),
    reason="set XODR_PLOT_LIVE_E2E=1 (needs docker + GPU + a CARLA image)",
)


def run_test_route(xodr: Path, route: Path) -> subprocess.CompletedProcess:
    return subprocess.run(
        [
            sys.executable,
            "-m",
            "xodr_plot",
            str(xodr),
            "--test-route",
            str(route),
        ],
        capture_output=True,
        text=True,
        timeout=20 * 60,
    )


def assert_container_left_alone(log: str) -> None:
    # The container is the user's: the tool must leave it in place (no
    # kill/rm), announcing that it is left running or untouched.
    assert "Leaving CARLA container" in log or "Interrupted" in log


def assert_user_container_still_exists() -> None:
    result = subprocess.run(
        ["docker", "inspect", "--format", "{{.Name}}", "CARLA-0.9.15"],
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, (
        "the user's CARLA-0.9.15 container was removed — the tool must never delete it"
    )


def test_town01_known_good_route():
    result = run_test_route(
        REPO_ROOT / "scenes/town01/maps/Town01.xodr",
        REPO_ROOT / "scenes/town01/agent_routes/agent_turn.xml",
    )
    log = result.stdout
    assert result.returncode == 0, log
    assert "all segments routable" in log
    assert_container_left_alone(log)
    assert_user_container_still_exists()


def test_disconnected_map_fails_with_clean_report():
    result = run_test_route(
        FIXTURES / "disconnected.xodr", FIXTURES / "disconnected_route.xml"
    )
    log = result.stdout
    assert result.returncode == 1, log
    assert "no path in the road network" in log
    assert "Route test: FAILED" in log
    assert_container_left_alone(log)
    assert_user_container_still_exists()


def test_offroad_waypoint_warns_but_still_routes():
    # Second waypoint is far outside the Town01 map boundary: it gets
    # projected onto the nearest road (warning) and the segment still routes.
    result = run_test_route(
        REPO_ROOT / "scenes/town01/maps/Town01.xodr",
        FIXTURES / "offroad_route.xml",
    )
    log = result.stdout
    assert result.returncode == 0, log
    assert "off the nearest road" in log
    assert "all segments routable" in log
    assert_container_left_alone(log)
    assert_user_container_still_exists()


def test_dtt_map_routes():
    result = run_test_route(
        REPO_ROOT / "scenes/BorasEkas/maps/BorasEkas_small_14_fixed_wider.xodr",
        REPO_ROOT / "scenes/BorasEkas/agent_route.xml",
    )
    log = result.stdout
    assert result.returncode == 0, log
    assert "all segments routable" in log
    assert_container_left_alone(log)
    assert_user_container_still_exists()
