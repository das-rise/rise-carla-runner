# Rise-Carla-Runner

## Setup

Make sure you have `conda` installed. Then, install all required dependencies with
``` bash
make rebuild
```
This will create a `conda` environment called `rirun` and install into that all required dependencies.

> [!IMPORTANT]
> Make sure that you activate the environment using `conda activate rirun` before running any experiments.


Alternatively, checkout
``` bash
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

Before running `Carla` in the container, we need to install the `nvidia-docker2` package. 
**On the `Synergies` server, this is already installed.**
Else, follow the instructions from https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#installation-guide.

### Building and starting the Carla container

*Either:*
Execute:
```bash
docker build -t carla-synergies .
```
Then, start the container and Carla inside it:
```bash
bash start-carla-docker.sh
```

### Stopping Carla

*Either:*
Find the Carla container's id by hand:
```bash
docker ps | grep carla-synergies
```
And stop it:
```bash
docker kill ID_HERE
```

*Or:*
Execute:
```bash
bash stop-carla-docker.sh
```


## Example run

```bash
python run.py 15 PCLA-Town01/Town01.xodr PCLA-Town01/vehicle1_route.csv --movement teleport --timestep 0.0333333333333333333 --pcla_agent neat_aim2ddepth --pcla_route PCLA-Town01/agent_route.xml --ego_camera
```

will use the `PCLA` agent `NEAT_AIM2DDEPTH`, primed to follow the route in PCLA-Town01/agent_route.xml, with the camera attached to the `ego_vehicle`.

Best to learn about usage of `run.py` by executing

```bash
python run.py --help
```

## Procesing `.parquet` trajectories
`make install` (which is run, for example, by `make rebuild`) will also install `traj-convert`, a python package that is used for creating `.parquet` files for every simulation run. These files contain all required information for building an `OSI` file that is compatible with the `Omega Prime` file format used in the Synergies project. 
To convert the `.parquet` file to `OSI`, run the following (with the `rirun` environment activated in `conda`):
```bash
python -m traj_convert <<trajectory_file.parquet>> osi <<ISO_country_code>> <<version>> <<projection_string>> <<path_to_opendrive>> -o <<output_file.osi>>
```

For example:
```bash
python -m traj_convert traj.parquet osi 752 0.1.0 "" "Town01.xodr" -o output.osi
```


## PCLA agent route generation

An example of how the `xml` file defining the waypoints for a PCLA agent can be created is found in `PCLA-Town01/generate_agent_route.py`, there for the example of `Town01.xodr`. However, it is straightforwardly generalizable from there.

## PCLA addendum

This repository contains an edited version of the `PCLA` [repository](https://github.com/MasoudJTehrani/PCLA).