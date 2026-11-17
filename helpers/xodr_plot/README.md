# xodr-plot

A simple CLI tool to visualise **OpenDRIVE (.xodr)** road-network files using Matplotlib
and to plot agent routes.

## Requirements

- `uv`

## Installation

```bash
uv sync
```

## Usage

```bash
uv run xodr-plot path/to/map.xodr
```

A Matplotlib window will open showing the visualised road network.
Click inside the window to set waypoints and their orientation. Upon closing the plot
window, the waypoints will be printed as a `xml` route to `stdout`.
The resulting `xml` route has the correct format required for `PCLA` agent routes.

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

Junction roads appear as a brighter version of the same colours. Reference lines are drawn in gold (normal) or dashed orange (junction).

## Project structure

```
xodr_plot/
├── __init__.py
├── __main__.py   # CLI entry point
├── parser.py     # XML → Road dataclasses
├── geometry.py   # Geometry sampling + lane polygon computation
└── renderer.py   # Matplotlib rendering
pyproject.toml
README.md
```
