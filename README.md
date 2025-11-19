# OSI-Gen



## Goal

Create a python functionality that enables to generate `OpenSimulationInterface` (`OSI`) trace files carrying `GroundTruth` information compliant with 
the [OmegaPrime](https://github.com/ika-rwth-aachen/omega-prime/blob/main/docs/omega_prime_specification.md) format developed in the Synergies project. For more details about the requirements, check [`requirements.md`](requirements.md).

## How-to

The relevant `python` code is found in the [`python/`](python/) folder.
First, the trajectories inside a `Carla` simulation are recorded. To do this, a `polars` DataFrame containing annotations for every frame of a `Carla` simulation is generated.
Inside a `carla.Client`, this can be done as such:

```python

import carla2traj
...

traj_recorder = carla2traj.Carla2Traj(world, debug = False) # setup a Trajectory recorder

while True:
    world.tick()
    snapshot = world.get_snapshot()
    traj_recorder.process_world_snapshot(snapshot)
    ...

traj_recorder.save()
```

This saves the trajecory recording to a `parquet` file with a timestamped filename.

Next, the trajectories are converted. For this, `carla2traj.py` is run on the parquet file directly from the command line:

```bash
user:~$ python carla2traj.py --help

Usage: carla2traj.py [-h] [-o OUTPUT] parquet_file {osi,openlabel} arg1 arg2 [arg3] [arg4]

Convert CARLA trajectory data to OmegaPrime-compliant OSI or OpenLabel format

Positional Arguments:
  parquet_file          Path to the input parquet file containing trajectory data
  {osi,openlabel}       Output format: 'osi' or 'openlabel'
  arg1                  For OSI: ISO country_code (int), For OpenLabel: dummy argument A
  arg2                  For OSI: version (X.Y.Z), For OpenLabel: dummy argument B
  arg3                  For OSI: proj_string (str) (optional for OpenLabel)
  arg4                  For OSI: map_reference (str) (optional for OpenLabel)

Options:
  -h, --help            show this help message and exit
  -o, --output OUTPUT   Output file path (default: auto-generated based on format)
```

# Visualization

See [`jupyter/omega_prime_viz.ipynb`](jupyter/omega_prime_viz.ipynb). Activate the development environment before accessing the notebook.


# Development

To further develop the code, use the `conda` environment file in `environment.yml`:
```bash
conda env create -f environment.yml
```

# Acknowledgments

This software was developed as part of the [Synergies](https://synergies-ccam.eu/) project.

![Synergies-logo](media/synergies.png)

Funded by the European Union. Views and opinions expressed are however those of the author(s) only and do not necessarily reflect those of the European Union or European Climate, Infrastructure and Environment Executive Agency (CINEA). Neither the European Union nor the granting authority can be held responsible for them.