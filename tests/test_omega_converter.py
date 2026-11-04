"""Tests for the OmegaPrime -> CSV converter (helpers/omegaprime2rirun).

The converter is a standalone uv project (Python >= 3.10, needs
omega-prime/betterosi). Its dependency-free helpers are imported from the
source tree here and tested on any Python, including the bridge through
rirun's ``process_trajectory_file`` (no CARLA server needed).

The end-to-end test on the sample recording skips when the omega-prime
stack or the sample file is unavailable.
"""

import glob
import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))
sys.path.insert(
    0,
    os.path.join(os.path.dirname(__file__), "..", "helpers", "omegaprime2rirun"),
)

from omegaprime2rirun import convert  # noqa: E402

from rirun.kinetics.trajectory_utils import process_trajectory_file  # noqa: E402
from rirun.kinetics.vehicle_models import VEHICLE_FILENAME_RE  # noqa: E402

SAMPLE_PATHS = sorted(
    glob.glob(os.path.join(os.path.dirname(__file__), "..", "OmegaPrime_*.mcap"))
)


def _sample_path():
    return SAMPLE_PATHS[0] if SAMPLE_PATHS else None


# ---------------------------------------------------------------------------
# Pure helpers
# ---------------------------------------------------------------------------


def test_heading_for_rirun() -> None:
    # rirun mirrors y -> -y, which negates in-plane angles
    assert convert.heading_for_rirun(0.0) == 0.0
    assert convert.heading_for_rirun(math.pi / 2) == pytest.approx(270.0)
    assert convert.heading_for_rirun(math.pi) == pytest.approx(180.0)
    assert convert.heading_for_rirun(3 * math.pi / 2) == pytest.approx(90.0)
    assert convert.heading_for_rirun(-math.pi / 2) == pytest.approx(90.0)
    # wraps to [0, 360)
    assert 0.0 <= convert.heading_for_rirun(7.0) < 360.0


def test_vehicle_csv_name_parses_with_rirun_regex() -> None:
    name = convert.vehicle_csv_name(184, 4, 5.02)
    assert name == "vehicle_184_Car_5.0.csv"
    match = VEHICLE_FILENAME_RE.match(name)
    assert match is not None
    assert match.group("vehicle_id") == "184"
    assert match.group("vehicle_type") == "Car"
    assert match.group("size") == "5.0"

    # unknown subtype falls back to "Vehicle"
    assert convert.vehicle_csv_name(7, 99, 3.8) == "vehicle_7_Vehicle_3.8.csv"

    # all labels must be parseable by rirun ([A-Za-z]+)
    for subtype in (2, 3, 4, 5, 6, 7, 8, 9, 12, 16, 0, 1, 10, 11, 13, 14):
        assert VEHICLE_FILENAME_RE.match(convert.vehicle_csv_name(1, subtype, 4.5))


def test_write_vehicle_csv(tmp_path) -> None:
    rows = [(1.0, 2.0, 0.0, 30.0), (1.5, 2.5, 0.5, 31.0)]
    path = str(tmp_path / "with_heading.csv")
    convert.write_vehicle_csv(path, rows, with_heading=True)
    with open(path) as f:
        lines = f.read().strip().split("\n")
    assert lines[0] == "x_loc [m],y_loc [m],Time [s],Heading [deg]"
    assert lines[1] == "1.0,2.0,0.0,30.0"

    path3 = str(tmp_path / "no_heading.csv")
    convert.write_vehicle_csv(path3, rows, with_heading=False)
    with open(path3) as f:
        lines3 = f.read().strip().split("\n")
    assert lines3[0] == "x_loc [m],y_loc [m],Time [s]"
    assert lines3[1] == "1.0,2.0,0.0"


# ---------------------------------------------------------------------------
# Bridge: generated CSVs through rirun's trajectory pipeline
# ---------------------------------------------------------------------------


def _write_trace_csv(tmp_path, name, rows, with_heading=True):
    path = os.path.join(str(tmp_path), name)
    convert.write_vehicle_csv(path, rows, with_heading=with_heading)
    return path


def test_bridge_4_column_csv_through_process_trajectory_file(tmp_path) -> None:
    # constant heading 30 deg in the (already y-mirrored) rirun frame
    rows = [
        (0.0, 0.0, 0.0, 30.0),
        (10.0, 0.0, 1.0, 30.0),
        (20.0, 0.0, 2.0, 30.0),
    ]
    path = _write_trace_csv(tmp_path, "vehicle_1_Car_5.0.csv", rows)
    traj = process_trajectory_file(path, offset=(100.0, 200.0))
    pts = list(traj.get_trajectory())
    assert len(pts) == 3
    # offset applied, northing mirrored
    assert (pts[0].x, pts[0].y) == pytest.approx((100.0, -200.0))
    assert (pts[1].x, pts[1].y) == pytest.approx((110.0, -200.0))
    # file heading is kept as-is (same as savant2rirun 4-column CSVs)
    assert pts[0].heading == pytest.approx(30.0)
    assert pts[2].heading == pytest.approx(30.0)
    # speeds derived from positions: 10 m/s = 36 km/h
    assert pts[1].speed == pytest.approx(36.0)


def test_bridge_3_column_csv_gets_generated_heading(tmp_path) -> None:
    # heading-mode "straight"/"spline": 3-column CSV, rirun derives headings
    rows = [(0.0, 0.0, 0.0), (0.0, 10.0, 1.0)]
    path = _write_trace_csv(tmp_path, "vehicle_2_Van_4.5.csv", rows, with_heading=False)
    traj = process_trajectory_file(path, heading_interpolation_mode="straight")
    pts = list(traj.get_trajectory())
    # after the y -> -y flip the +y direction becomes -y: heading 270 deg
    assert pts[1].heading == pytest.approx(270.0)
    assert pts[1].speed == pytest.approx(36.0)


# ---------------------------------------------------------------------------
# End-to-end on the sample recording (needs the omega-prime stack)
# ---------------------------------------------------------------------------


@pytest.mark.skipif(
    _sample_path() is None, reason="sample OmegaPrime recording not present"
)
def test_e2e_sample_mcap_produces_rirun_csvs(tmp_path) -> None:
    pytest.importorskip("omega_prime")
    written = convert.convert_mcap(_sample_path(), str(tmp_path))
    # 148 VEHICLE objects in the sample, 9 skipped (<3 frames)
    assert len(written) == 139

    # every generated file name must be parseable by rirun
    for path in written:
        assert VEHICLE_FILENAME_RE.match(os.path.basename(path)), path

    # and every CSV must flow through rirun's pipeline
    for path in written[:3]:
        traj = process_trajectory_file(path)
        assert traj._len_trajectory >= 3
        pts = list(traj.get_trajectory())
        assert all(p.speed is not None for p in pts[1:])
