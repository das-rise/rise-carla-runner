# omegaprime2rirun

Convert **OmegaPrime** recordings (`.mcap` files with OSI ground-truth
traces, e.g. produced by [COSMO](https://github.com/RI-SE/COSMO)) into
**rirun** trajectory CSVs — one CSV per vehicle.

This is a standalone tool with its own uv environment (Python >= 3.10,
required by `omega-prime`/`betterosi`). rirun itself stays on Python 3.8
and ingests the resulting CSVs exactly like any other trajectory files.

## Usage

```bash
cd helpers/omegaprime2rirun
uv sync                      # creates .venv with Python >= 3.10

uv run omegaprime2csv /path/OmegaPrime_xxx.mcap -o /tmp/omega_csvs
```

Then run rirun on the output as usual:

```bash
python -m rirun.run <duration> <map> -tf /tmp/omega_csvs/*.csv \
    --npc_trajectory_offset <X> <Y>   # same offset you use for your CSV scenes
```

## What it does

- Reads the `ground_truth` topic (one OSI `GroundTruth` per frame) with
  `omega-prime` (`Recording.from_file`).
- Each object with `type == VEHICLE` (default; see `--types`) becomes one
  vehicle; objects with fewer than `--min-frames` frames are skipped.
- The vehicle **size** is the median of the per-frame OSI `length`
  (measured length in meters) and is encoded into the file name
  `vehicle_{id}_{Type}_{size}.csv`, which rirun parses to pick the CARLA
  vehicle model with the closest length.
- Coordinates are written as-is (the recording's frame, e.g. UTM
  easting/northing); use rirun's `--npc_trajectory_offset` as with any
  other trajectory files. Times are absolute trace seconds; use
  `--offset_time` to start mid-scene.
- Heading: with `--heading-mode trace` (default) the trace yaw is written
  as a 4th column, converted to rirun's y-mirrored frame. With
  `--heading-mode straight|spline` a 3-column CSV is written and rirun
  derives the headings at run time.

## Tests

```bash
uv sync --extra dev
uv run pytest
```

The end-to-end test reads the sample recording at the repository root
(`OmegaPrime_*.mcap`) and skips if it is not present.
