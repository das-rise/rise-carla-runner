# xodr-plot

A CLI tool to visualise **OpenDRIVE (.xodr)** road-network files with Matplotlib,
**create and edit agent routes** interactively, and **test routes against a live
CARLA server** before using them in a simulation.

## Features

- **Dark map aesthetic** — dark background with colour-coded filled lanes
- **Lane types** — driving, shoulder, sidewalk, parking, median, biking, restricted, and more, each with a distinct colour
- **Junctions** — highlighted with brighter lane tints and dashed orange reference lines
- **Reference lines** — gold for normal roads, dashed orange for junction connecting roads
- **All geometry primitives** — line, arc, clothoid/spiral, poly3, paramPoly3
- **Route editor** — click to place route waypoints with heading arrows; move,
  insert, delete and re-aim them; export as PCLA agent-route XML
- **Route testing** — traces the route with the same `GlobalRoutePlanner` the
  PCLA agents use at runtime, on your existing CARLA container (started with
  `docker start` if it is stopped, left running afterwards)

## Requirements

- Python ≥ 3.8 (route testing: 3.8–3.10; the dev venv is pinned to
  3.10 via `.python-version` — the newest Python with a `carla` wheel and
  a working Tk GUI in uv-managed interpreters)
- `matplotlib` ≥ 3.5, `numpy` ≥ 1.21
- Route testing additionally needs `carla` and `networkx` (installed
  automatically with xodr-plot on **Python 3.8–3.10**; the `carla` wheel
  does not exist for newer interpreters, where only the visualisation part
  is available) and [Docker](https://docs.docker.com/) with the
  [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html),
  a GPU, and an existing CARLA container — by default
  `CARLA-0.9.15`, the one created by `docker/start-carla-docker.sh`
  (override with the `CARLA_CONTAINER` env var or `--carla-container`).
  The route test loads the map into *your* container (replacing its world,
  exactly like the rirun experiments do), so it cannot run at the same
  time as your own simulation on that container.

## Installation

```bash
pip install .
```

This installs the `xodr-plot` command (visualisation + route testing).
Alternatively, run directly without installing:
`python -m xodr_plot path/to/map.xodr`.

## Usage

```bash
xodr-plot path/to/map.xodr
```

A Matplotlib window opens showing the road network.

### Creating and editing a route

| Action | Effect |
|--------|--------|
| Left-click empty map | Add a waypoint (heading panel activates) |
| Left-click a waypoint | Select it (heading panel shows its angle) |
| Drag a waypoint | Move it |
| Right-click a waypoint | Delete it |
| Left-click the route line | Insert a waypoint between the two endpoints |
| Slider in the bottom-left panel | Set the selected waypoint's heading (live preview arrow) |
| `Delete waypoint` button | Delete the selected waypoint |
| `Test route` button | Test the route against CARLA (see below) |
| `Clear` button | Remove all waypoints |
| `Save` button | Write the route to the `--output` file (or print to stdout) |

All waypoints are shown at all times as numbered markers with red heading
arrows. On closing the window, the route XML is printed to `stdout` (copy-paste
it into an `agent-route.xml` file) and, if `--output` was given, the route is
also written there.

```bash
# Open an existing route for editing / re-testing, save to a file
xodr-plot path/to/map.xodr --load scenes/town01/agent_routes/agent_turn.xml --output my_route.xml
```

### Testing a route

The `Test route` button (or the headless mode below) verifies that CARLA's
`GlobalRoutePlanner` — the same planner the PCLA agents use at runtime via
`interpolate_trajectory()` — can trace a route through all consecutive
waypoint pairs. A route that passes is a route the agents can drive.

**Container lifecycle:** the test uses your regular CARLA container —
`CARLA-0.9.15` by default (the one `docker/start-carla-docker.sh` creates),
overridable with `--carla-container <name>` or the `CARLA_CONTAINER` env var.
If it is stopped, the tool `docker start`s it and waits for the engine (a
boot takes a few minutes); if it is already running, it connects right away.
When the tool exits (or on Ctrl+C), the container is **left running** — it
is your container, with your lifecycle; `docker stop CARLA-0.9.15` to free
the GPU. Note that the test **replaces the container's world** with the
`.xodr` map being tested, so don't run it while a simulation is using the
container:

```
[xodr-plot] Starting CARLA container 'CARLA-0.9.15' (port 2000)…
[xodr-plot] Waiting for CARLA server…
[xodr-plot] RPC port open — letting the engine finish initialising…
[xodr-plot] CARLA server ready.
[xodr-plot] Loading map…
[xodr-plot] Map loaded, route planner ready.
...
[xodr-plot] Leaving CARLA container 'CARLA-0.9.15' running (it is your container — stop it with `docker stop CARLA-0.9.15` to free the GPU).
```

If the container or server dies between tests, the next test restarts it
transparently (up to 3 attempts — a failed boot often succeeds on a fresh
engine). On failure you get a dialog listing the unroutable segments
(e.g. `no path in the road network between the two waypoints`), the failing
segment is highlighted red on the map, and you can edit the route and test
again. Waypoints that sit more than ~2.5 m off the nearest road are flagged —
CARLA would project them onto the road.

**Headless mode** (no GUI) tests a saved route file:

```bash
xodr-plot path/to/map.xodr --test-route route.xml
# per-segment report on stdout; exit code 0 = all segments routable, 1 = failure
```

## Map legend

| Colour | Lane type |
|--------|-----------|
| dark grey | driving |
| brown | shoulder |
| dark blue | sidewalk |
| dark purple | parking |
| dark green | median |
| teal | biking |
| dark red | restricted |
| … | … |

Junction roads appear as a brighter version of the same colours. Reference
lines are drawn in gold (normal) or dashed orange (junction).

## Development

```bash
uv sync                              # venv with all deps + pytest (dev group)
uv run pytest tests/ -q              # unit tests (no docker needed)

# Live end-to-end tests (start real CARLA containers; a few minutes each):
XODR_PLOT_LIVE_E2E=1 uv run pytest tests/test_e2e_live.py -v
```

## Project structure

```
xodr_plot/
├── __init__.py
├── __main__.py        # CLI entry point (GUI + headless test mode)
├── parser.py          # XML → Road dataclasses
├── geometry.py        # Geometry sampling + lane polygon computation
├── renderer.py        # Matplotlib map rendering
├── route.py           # Route model, XML I/O, coordinate transforms
├── route_editor.py    # Interactive route editor (waypoints, test button)
├── carla_server.py    # CARLA container lifecycle (docker start/stop)
├── planner.py         # Per-segment route checking (GlobalRoutePlanner wrapper)
├── planner_grp.py     # Vendored CARLA leaderboard GlobalRoutePlanner (MIT)
├── planner_dao.py     # Vendored CARLA leaderboard GlobalRoutePlannerDAO (MIT)
├── connectivity.py    # Topology diagnostics
└── fixer.py           # Geometry gap fixing
```
