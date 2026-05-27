# Rise-Carla-Runner

**RiRun** (RISE Carla Runner) is a tool for creating parametrizable driving scenarios in the [Carla](https://carla.org/) simulator from real-world automotive trajectory data. It supports multiple autonomous driving agents via the [PCLA](https://github.com/MasoudJTehrani/PCLA) framework, trajectory replay with different movement modes, and can export simulation results as `.parquet` files which can then be converted to [ASAM OSI](https://www.asam.net/standards/detail/osi/) using the [`osi-gen`](https://github.com/das-rise/osi-gen) tool.

> [!NOTE]
> This open source project is developed by [RISE Research Institutes of Sweden](https://ri.se/). See license file for open source license information.

## Table of Contents

- [Prerequisites](#prerequisites)
- [Setup](#setup)
- [Getting Carla set up in a container](#getting-carla-set-up-in-a-container)
  - [Building, starting and stopping the Carla container](#building-starting-and-stopping-the-carla-container)
- [Example run](#example-run)
- [Processing `.parquet` trajectories](#processing-parquet-trajectories)
- [PCLA agent route generation](#pcla-agent-route-generation)
- [PCLA addendum](#pcla-addendum)
- [Acknowledgments](#acknowledgments)

## Prerequisites

- **Python 3.8** (required — see `requires-python` in `pyproject.toml`)
- **conda** (Miniconda or Miniforge)
- **Carla 0.9.15** — either a local installation (set `CARLA_ROOT` in `.env`) or via Docker with [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html)
- **NVIDIA GPU** with compatible drivers

## Setup

First, copy the example environment file and adjust it to your setup:

```bash
cp .env.example .env
```

Make sure you have `conda` installed. Then, install all required dependencies with

```bash
make rebuild
```

This will create a `conda` environment called `rirun` and install into that all required dependencies.

> [!IMPORTANT]
> Make sure that you activate the environment using `conda activate rirun` before running any experiments.

Alternatively, checkout

```bash
make help
```

for a custom installation.

## Getting Carla set up in a container

You will need a `Carla 0.9.15` server for the simulations. It is most portable to spin one up using Docker, as described in the following:

First, setup `Docker` for your user:

1. create the `docker` group:
   `sudo groupadd docker`

2. add your user to the group:
   `sudo usermod -aG docker $USER`

3. activate changes to groups:
   `newgrp docker`

4. verify you can run docker commands without `sudo`:
   `docker run hello-world`

Great! Now, we pull the `Carla` container (needs to download a few GBs of data):
`docker pull carlasim/carla:0.9.15`

Before running `Carla` in the container, we need to install the `nvidia-docker2` package. Follow the instructions from https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#installation-guide.

### Building, starting and stopping the Carla container

_Build and start:_

```bash
bash docker/start-carla-docker.sh
```

To start the container with a locally built `Carla` package instead of the `carlasim/carla:0.9.15` runtime files, set `CARLA_ROOT` in `.env`. The directory must contain `CarlaUE4.sh` and `PythonAPI/carla/dist/carla-*.whl`. Use `PYTHON_VERSION` to select the matching Python API wheel when the build contains multiple wheels, for example `PYTHON_VERSION="3.8"` selects `PythonAPI/carla/dist/carla-*-cp38-cp38-*.whl`.

```bash
# .env
CARLA_ROOT="/path/to/CARLA"
PYTHON_VERSION="3.8"
```

```bash
bash docker/start-carla-docker.sh
```

_Stop:_
And stop it:

```bash
docker kill ID_HERE
```

_Or:_
Execute:

```bash
bash docker/stop-carla-docker.sh
```

## Example run

Execute:

```bash
python src/rirun/run.py \
    15 \
    Town01 \
    --npc_trajectory_filepaths scenes/town01/npc_test_routes/vehicle1_straight.csv \
    --npc_movement teleport \
    --timestep 0.0333333333333333333 \
    --ego_agent neat_aim2ddepth \
    --ego_route_filepath scenes/town01/agent_routes/agent_turn.xml \
    --record_cameras ego_overhead
```

will use the `PCLA` agent `NEAT_AIM2DDEPTH`, primed to follow the route in agent_turn.xml, with the camera attached to the `ego_vehicle`.

Best to learn about usage of `run.py` by executing

```bash
python src/rirun/run.py --help
```

## Processing `.parquet` trajectories

`make install` (which is run, for example, by `make rebuild`) will also install [`traj-convert`](https://github.com/das-rise/osi-gen), a python package that is used for creating `.parquet` files for every simulation run. These `.parquet` files can then be exported to `OSI` files compatible with the `Omega Prime` file format used in the Synergies project.

> [!NOTE]
> The `traj_convert` command-line utility requires Python 3.10+ and will not work inside the `rirun` conda environment (Python 3.8). Install it in a separate Python 3.10+ environment.

To convert a `.parquet` file to `OSI`, run the following (in a Python 3.10+ environment with `traj-convert` installed):

```bash
python -m traj_convert <<trajectory_file.parquet>> osi <<ISO_country_code>> <<version>> <<projection_string>> <<path_to_opendrive>> -o <<output_file.osi>>
```

For example:

```bash
python -m traj_convert traj.parquet osi 752 0.1.0 "" "Town01.xodr" -o output.osi
```

## PCLA agent route generation

To generate agent route `.xml` files, use the `xodr-plot` tool (installed with the project from `helpers/xodr_plot/`).
```bash
xodr-plot path-to-opendrive.xodr
```
Then, place waypoints by clicking on the map and then adjusting the waypoint angle in the popup. Add as many waypoints as required. Upon closing the plot window, the entire `.xml` route is printed to `stdout` and can be copy-pasted to some `agent-route.xml` file.

## PCLA addendum

This repository contains an edited version of the `PCLA` [repository](https://github.com/MasoudJTehrani/PCLA).

## Acknowledgments

This software was developed as part of the [Synergies](https://synergies-ccam.eu/) project.

![Synergies-logo](media/synergies.png)

Funded by the European Union. Views and opinions expressed are however those of the author(s) only and do not necessarily reflect those of the European Union or European Climate, Infrastructure and Environment Executive Agency (CINEA). Neither the European Union nor the granting authority can be held responsible for them.
