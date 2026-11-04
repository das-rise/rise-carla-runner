"""Command line interface: omegaprime2csv INPUT.mcap -o OUTDIR."""

import argparse
import logging
import sys

from omegaprime2rirun import convert


def _types_from_arg(value: str):
    """Map a --types choice to a tuple of osi3.MovingObject.Type values."""
    if value == "vehicle":
        return (convert.TYPE_VEHICLE,)
    if value == "vehicle-other":
        return (convert.TYPE_VEHICLE, convert.TYPE_OTHER)
    # "all": every moving object except unknown
    return (
        convert.TYPE_OTHER,
        convert.TYPE_VEHICLE,
        convert.TYPE_PEDESTRIAN,
        convert.TYPE_ANIMAL,
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="omegaprime2csv",
        description=(
            "Convert an OmegaPrime (.mcap) OSI ground-truth recording into "
            "one rirun trajectory CSV per vehicle. The CSVs follow rirun's "
            "vehicle_{id}_{type}_{size}.csv naming so rirun picks the "
            "matching CARLA vehicle model; run rirun on the output directory "
            "as usual (e.g. rirun ... -tf OUTDIR/*.csv)."
        ),
    )
    parser.add_argument(
        "mcap",
        type=str,
        help="Path to the OmegaPrime .mcap file.",
    )
    parser.add_argument(
        "-o",
        "--outdir",
        type=str,
        required=True,
        help="Directory to write the vehicle CSVs into.",
    )
    parser.add_argument(
        "--types",
        choices=["vehicle", "vehicle-other", "all"],
        default="vehicle",
        help=(
            "Which objects become vehicles: 'vehicle' (default, only "
            "objects classified VEHICLE), 'vehicle-other' (also "
            "unclassified 'other' moving objects), 'all' (also "
            "pedestrians and animals)."
        ),
    )
    parser.add_argument(
        "--min-frames",
        type=int,
        default=3,
        help="Minimum number of frames per object (default: 3).",
    )
    parser.add_argument(
        "--heading-mode",
        choices=list(convert.HEADING_MODES),
        default="trace",
        help=(
            "'trace' (default) writes the trace yaw as a 4th CSV column; "
            "'straight'/'spline' write 3-column CSVs and let rirun derive "
            "headings at run time via --heading_interpolation_mode."
        ),
    )
    parser.add_argument(
        "-v",
        "--verbose",
        action="store_true",
        help="Enable debug logging.",
    )
    return parser


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s [%(levelname)s] - %(message)s",
    )

    try:
        convert.convert_mcap(
            args.mcap,
            outdir=args.outdir,
            types=_types_from_arg(args.types),
            min_frames=args.min_frames,
            heading_mode=args.heading_mode,
        )
    except ImportError as e:
        logging.error(str(e))
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
