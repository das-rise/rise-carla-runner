"""Core conversion: OmegaPrime (.mcap) OSI recordings -> rirun trajectory CSVs.

An OmegaPrime recording produced by COSMO stores one OSI ``GroundTruth``
message per frame on the ``ground_truth`` MCAP topic. Each tracked object in
the recording becomes one rirun vehicle: a CSV with the file name pattern
``vehicle_{vehicle_id}_{vehicle_type}_{size}.csv`` that rirun parses for the
vehicle id, a cosmetic type label, and the overall vehicle length (size in
meters) used to pick the CARLA vehicle model.

The pure helpers below (:func:`heading_for_rirun`, :func:`vehicle_csv_name`,
:func:`write_vehicle_csv`) have no third-party dependencies and work on any
Python. Only :func:`extract_vehicles` and :func:`convert_mcap` require the
omega-prime / betterosi stack (Python >= 3.10, see this project's pyproject).
"""

import logging
import math
import os
from dataclasses import dataclass
from typing import Dict, List, Sequence, Tuple

# osi3.MovingObject.Type values
TYPE_UNKNOWN = 0
TYPE_OTHER = 1
TYPE_VEHICLE = 2
TYPE_PEDESTRIAN = 3
TYPE_ANIMAL = 4

TYPE_NAMES = {
    TYPE_UNKNOWN: "UNKNOWN",
    TYPE_OTHER: "OTHER",
    TYPE_VEHICLE: "VEHICLE",
    TYPE_PEDESTRIAN: "PEDESTRIAN",
    TYPE_ANIMAL: "ANIMAL",
}

# osi3.MovingObject.VehicleClassification.Type values -> file name label.
# The label is cosmetic for rirun (the size in the file name drives the
# vehicle model selection); unknown subtypes fall back to "Vehicle".
SUBTYPE_LABELS = {
    2: "Car",  # SMALL_CAR
    3: "Car",  # COMPACT_CAR
    4: "Car",  # CAR
    5: "Car",  # LUXURY_CAR
    6: "Van",  # DELIVERY_VAN
    7: "Truck",  # HEAVY_TRUCK
    8: "Trailer",  # SEMITRAILER
    9: "Trailer",  # TRAILER
    12: "Bus",  # BUS
    16: "Truck",  # SEMITRACTOR
}

# Heading modes for the output CSVs.
HEADING_MODES = ("trace", "straight", "spline")


@dataclass
class VehicleTrace:
    """One vehicle extracted from an OmegaPrime recording.

    ``rows`` are (x, y, t, heading_deg) tuples in the source coordinate
    frame (e.g. UTM easting/northing), sorted by time. ``heading_deg`` is
    already converted to the frame rirun expects (see
    :func:`heading_for_rirun`). ``t`` is absolute trace time in seconds.
    """

    idx: int
    type: int
    subtype: int
    size_m: float
    t0_s: float
    t1_s: float
    rows: List[Tuple[float, float, float, float]]

    @property
    def n_points(self) -> int:
        return len(self.rows)


def heading_for_rirun(yaw_rad: float) -> float:
    """Convert an OSI yaw (radians, source frame) to a rirun CSV heading (deg).

    rirun mirrors the northing (y -> -y) when converting to Carla
    coordinates, which negates in-plane angles. The heading stored in the
    CSV must therefore already be the negated source yaw, the same
    convention as the savant2rirun CSVs rirun already ingests.
    """
    return (-math.degrees(yaw_rad)) % 360.0


def vehicle_csv_name(idx: int, subtype: int, size_m: float) -> str:
    """File name rirun parses as ``vehicle_{id}_{type}_{size}.csv``.

    The size (overall vehicle length in meters) drives rirun's vehicle
    model selection; the type label is cosmetic.
    """
    label = SUBTYPE_LABELS.get(subtype, "Vehicle")
    return "vehicle_{0}_{1}_{2:.1f}.csv".format(idx, label, size_m)


def write_vehicle_csv(
    path: str, rows: Sequence[Tuple[float, ...]], with_heading: bool = True
) -> None:
    """Write a rirun trajectory CSV (same layout as savant2rirun output).

    Args:
        path: Output file path.
        rows: (x, y, t[, heading_deg]) tuples.
        with_heading: If False, write a 3-column CSV and let rirun derive
            headings from the positions at run time (via its
            --heading_interpolation_mode / --force_heading_interpolation
            flags).
    """
    with open(path, "w") as f:
        f.write("x_loc [m],y_loc [m],Time [s]")
        if with_heading:
            f.write(",Heading [deg]")
        f.write("\n")
        for row in rows:
            if with_heading:
                f.write("{0},{1},{2},{3}\n".format(row[0], row[1], row[2], row[3]))
            else:
                f.write("{0},{1},{2}\n".format(row[0], row[1], row[2]))


def extract_vehicles(
    mcap_path: str,
    types: Sequence[int] = (TYPE_VEHICLE,),
    min_frames: int = 3,
) -> Tuple[List[VehicleTrace], Dict[str, int]]:
    """Extract per-vehicle traces from an OmegaPrime .mcap file.

    Args:
        mcap_path: Path to an OmegaPrime .mcap file (OSI ground truth).
        types: osi3.MovingObject.Type values to include (default: VEHICLE).
        min_frames: Minimum number of frames an object needs to be included.

    Returns:
        ``(traces, skip_counts)`` with ``traces`` sorted by object id and
        ``skip_counts`` mapping a reason (object type name or
        ``"TOO_FEW_FRAMES"``) to the number of skipped objects.

    Raises:
        ImportError: If omega-prime is not available (wrong environment).
    """
    try:
        import polars as pl
        from omega_prime import Recording
    except ImportError as e:
        raise ImportError(
            "omega-prime is required to read OmegaPrime .mcap files. Run "
            "inside the helpers/omegaprime2rirun uv environment (cd "
            "helpers/omegaprime2rirun && uv sync); note that omega-prime "
            "requires Python >= 3.10."
        ) from e

    logging.info("Reading OmegaPrime recording: %s", mcap_path)
    recording = Recording.from_file(str(mcap_path))
    df = recording.df
    if df.height == 0:
        logging.warning("No moving objects found in %s", mcap_path)
        return [], {}

    t0_ns = df.select(pl.col("total_nanos").min()).item()
    t1_ns = df.select(pl.col("total_nanos").max()).item()
    n_objects = df.select(pl.col("idx").n_unique()).item()
    logging.info(
        "%s: %d frames, %d objects, %.3fs - %.3fs",
        mcap_path,
        df.height,
        n_objects,
        t0_ns / 1e9,
        t1_ns / 1e9,
    )

    per_object = (
        df.group_by("idx")
        .agg(
            pl.col("type").first().alias("type"),
            pl.col("subtype").first().alias("subtype"),
            pl.col("length").median().alias("size"),
            pl.col("total_nanos").min().alias("t0"),
            pl.col("total_nanos").max().alias("t1"),
            pl.len().alias("n"),
        )
        .sort("idx")
    )

    traces: List[VehicleTrace] = []
    skip_counts: Dict[str, int] = {}

    for row in per_object.iter_rows(named=True):
        type_name = TYPE_NAMES.get(row["type"], "TYPE_{0}".format(row["type"]))
        if row["type"] not in types:
            skip_counts[type_name] = skip_counts.get(type_name, 0) + 1
            continue
        if row["n"] < min_frames:
            skip_counts["TOO_FEW_FRAMES"] = skip_counts.get("TOO_FEW_FRAMES", 0) + 1
            continue

        sub = df.filter(pl.col("idx") == row["idx"])
        pts = (
            sub.select(["x", "y", "total_nanos", "yaw"]).sort("total_nanos").to_dicts()
        )
        rows = [
            (
                p["x"],
                p["y"],
                p["total_nanos"] / 1e9,
                heading_for_rirun(p["yaw"]),
            )
            for p in pts
        ]
        traces.append(
            VehicleTrace(
                idx=row["idx"],
                type=row["type"],
                subtype=row["subtype"],
                size_m=float(row["size"]),
                t0_s=row["t0"] / 1e9,
                t1_s=row["t1"] / 1e9,
                rows=rows,
            )
        )

    return traces, skip_counts


def convert_mcap(
    mcap_path: str,
    outdir: str,
    types: Sequence[int] = (TYPE_VEHICLE,),
    min_frames: int = 3,
    heading_mode: str = "trace",
) -> List[str]:
    """Convert an OmegaPrime .mcap file to per-vehicle rirun CSVs in ``outdir``.

    Args:
        mcap_path: Path to the OmegaPrime .mcap file.
        outdir: Directory to write the CSVs into (created if needed).
        types: osi3.MovingObject.Type values to include (default: VEHICLE).
        min_frames: Minimum number of frames per object to include.
        heading_mode: "trace" writes the trace yaw as 4th column;
            "straight"/"spline" write 3-column CSVs and let rirun derive
            the headings at run time.

    Returns:
        List of written file paths.
    """
    if heading_mode not in HEADING_MODES:
        raise ValueError(
            "heading_mode must be one of {0}, got {1}".format(
                HEADING_MODES, heading_mode
            )
        )

    traces, skip_counts = extract_vehicles(
        mcap_path, types=types, min_frames=min_frames
    )
    if not traces:
        logging.warning(
            "No vehicles to convert from %s (skipped: %s)", mcap_path, skip_counts
        )
        return []

    os.makedirs(outdir, exist_ok=True)
    with_heading = heading_mode == "trace"
    written: List[str] = []
    for trace in traces:
        name = vehicle_csv_name(trace.idx, trace.subtype, trace.size_m)
        path = os.path.join(outdir, name)
        write_vehicle_csv(path, trace.rows, with_heading=with_heading)
        written.append(path)
        logging.info(
            "  %s: %d frames, size %.2f m, %.3fs - %.3fs",
            name,
            trace.n_points,
            trace.size_m,
            trace.t0_s,
            trace.t1_s,
        )

    logging.info(
        "Wrote %d vehicle CSV(s) to %s (skipped: %s)",
        len(written),
        outdir,
        skip_counts,
    )
    return written
