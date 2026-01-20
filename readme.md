# PART 1

# Getting Carla setup in a container

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

# Building and starting the Carla container

*Either:*
Execute:
```
docker build -t carla-synergies .
```
Then, start the container and Carla inside it:
```
docker run -d --privileged --gpus=all --net=host carla-synergies /bin/bash ./CarlaUE4.sh -RenderOffScreen
```

*Or:*
Execute:
```
bash start-carla-docker.sh
```

# Stopping Carla

*Either:*
Find the Carla container's id by hand:
```
docker ps | grep carla-synergies
```
And stop it:
```
docker kill ID_HERE
```

*Or:*
Execute:
```
bash stop-carla-docker.sh
```

# Running Carla clients or Scenic

For running Carla clients that interact with the running Carla docker server, or for running Scenic examples, download and install `Anaconda` first.
After having activated `Anaconda`, execute
```
conda env create -f conda-env.yml
```
From within this environment, you have access to the `scenic` and `carla` python packages.

-----------------------------------------

# Part 2

# Idea

In this directory, we will collect an OpenDrive file
together with scripts to follow waypoints within the map,
as well as scripts for one or more simple agent(s) that follow(s)
 such waypoints.

# Example run

```bash
python run.py 15 PCLA-Town01/Town01.xodr PCLA-Town01/vehicle1_route.csv --movement teleport --timestep 0.0333333333333333333 --pcla_agent neat_aim2ddepth --pcla_route PCLA-Town01/agent_route.xml --ego_camera
```

will use the `PCLA` agent `NEAT_AIM2DDEPTH`, primed to follow the route in PCLA-Town01/agent_route.xml, with the camera attached to the `ego_vehicle`.

Best to learn about usage of `run.py` by executing

```bash
python run.py --help
```

# PCLA agent route generation

An example of how the `xml` file defining the waypoints for a PCLA agent can be created is found in `PCLA-Town01/generate_agent_route.py`, there for the example of `Town01.xodr`. However, it is straightforwardly generalizable from there.

# PCLA addendum

This repository contains an edited version of the `PCLA` [repository](https://github.com/MasoudJTehrani/PCLA).