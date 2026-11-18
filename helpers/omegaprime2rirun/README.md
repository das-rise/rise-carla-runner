# omegaprime2rirun

Convert OmegaPrime `.mcap` recordings (OSI ground truth, e.g. from [COSMO](https://github.com/RI-SE/COSMO)) into one [rirun](../..) trajectory CSV per vehicle.

## Usage

```bash
uv sync                      # own uv environment, Python >= 3.10 (omega-prime/betterosi)
uv run omegaprime2csv /path/OmegaPrime_xxx.mcap -o /tmp/omega_csvs
```

Then run rirun on the output as usual:

```bash
python src/rirun/run.py <duration> <map> -tf /tmp/omega_csvs/*.csv --npc_trajectory_offset <X> <Y>
```

## Options

- `--types vehicle|vehicle-other|all` — which objects become vehicles (default: `vehicle`)
- `--min-frames N` — skip objects with fewer frames (default: 3)
- `--heading-mode trace|straight|spline` — `trace` (default) writes the trace yaw, converted to rirun's y-mirrored frame, as a 4th CSV column; the other modes write 3-column CSVs and let rirun derive headings at run time
- `--apply-xodr-offset` — transform the coordinates from the recording's geodetic frame (e.g. UTM easting/northing) into the local frame of the OpenDrive map embedded in the recording: the map header's `<offset>` is subtracted (and rotated by the offset's `hdg` if non-zero). Use when the CARLA map was cooked from that OpenDrive file (e.g. DTT maps), which places the roads in the OpenDrive local frame. Fails if the recording contains no OpenDrive map or its header has no `<offset>` element

## Notes

- CSV names: `vehicle_{id}_{type}_{size}.csv`, where size is the median of the per-frame OSI lengths in meters — rirun uses it to pick the closest CARLA vehicle model
- By default, coordinates and times are written as in the recording (geodetic frame, e.g. UTM), so use `--npc_trajectory_offset` / `--offset_time` as for any other trajectory files. For maps cooked from the recording's embedded OpenDrive (e.g. DTT maps), pass `--apply-xodr-offset` instead so the CSVs already lie in the map's local frame
- Tests: `uv sync --extra dev && uv run pytest` (the end-to-end test uses the sample `OmegaPrime_*.mcap` at the repo root and skips if absent)
