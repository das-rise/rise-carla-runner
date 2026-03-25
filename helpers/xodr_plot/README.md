# xodr-plot

A simple CLI tool to visualise **OpenDRIVE (.xodr)** road-network files using Matplotlib.

## Features

- **Dark map aesthetic** — dark background with colour-coded filled lanes
- **Lane types** — driving, shoulder, sidewalk, parking, median, biking, restricted, and more, each with a distinct colour
- **Junctions** — highlighted with brighter lane tints and dashed orange reference lines
- **Reference lines** — gold for normal roads, dashed orange for junction connecting roads
- **All geometry primitives** — line, arc, clothoid/spiral, poly3, paramPoly3

## Requirements

- Python ≥ 3.8
- `matplotlib` ≥ 3.5
- `numpy` ≥ 1.21

## Installation

```bash
pip install .
```

This installs the `xodr-plot` command globally.

Alternatively, run directly without installing:

```bash
python -m xodr_plot path/to/map.xodr
```

## Usage

```bash
xodr-plot path/to/map.xodr
```

A Matplotlib window will open showing the visualised road network. Close the window to exit.

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
