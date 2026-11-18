"""End-to-end tests for the OmegaPrime -> rirun CSV conversion.

Requires the omega-prime/betterosi stack (this project's uv environment,
Python >= 3.10) and the sample recording at the repository root. Skips if
either is unavailable.
"""

import glob
import math
import os
import re

import pytest
from omegaprime2rirun import convert

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
SAMPLE_PATHS = sorted(glob.glob(os.path.join(REPO_ROOT, "OmegaPrime_*.mcap")))
SAMPLE_PATH = SAMPLE_PATHS[0] if SAMPLE_PATHS else None
SAMPLE_NAME = os.path.basename(SAMPLE_PATH) if SAMPLE_PATH else None

# Expected conversion results per known sample recording (the first match in
# sorted order is used as THE sample).
SAMPLE_EXPECTATIONS = {
    "OmegaPrime_An_VIDa_2025-11-19T143750CET_D.mcap": dict(
        n_vehicles=102,
        skips={},
        longest_idx=4199,
        longest_frames=23866,
        size_range=(2.6, 13.7),
    ),
    "OmegaPrime_pAn_VIDa_2026-02-06T163827CET_V.mcap": dict(
        n_vehicles=139,
        skips={"OTHER": 108, "TOO_FEW_FRAMES": 9},
        longest_idx=184,
        longest_frames=1422,
        size_range=(3.5, 8.9),
    ),
}
SAMPLE_EXP = SAMPLE_EXPECTATIONS.get(SAMPLE_NAME)

try:
    import omega_prime  # noqa: F401

    HAS_STACK = True
except ImportError:
    HAS_STACK = False

requires_sample = pytest.mark.skipif(
    SAMPLE_PATH is None, reason="sample OmegaPrime recording not present"
)
requires_stack = pytest.mark.skipif(not HAS_STACK, reason="omega-prime not installed")
requires_known_sample = pytest.mark.skipif(
    SAMPLE_EXP is None,
    reason="no expectations recorded for sample {0}".format(SAMPLE_NAME),
)

NAME_RE = re.compile(r"^vehicle_(\d+)_([A-Za-z]+)_(\d+(?:\.\d+)?)\.csv$")


@requires_sample
@requires_stack
@requires_known_sample
def test_convert_sample_mcap(tmp_path) -> None:
    exp = SAMPLE_EXP
    written = convert.convert_mcap(SAMPLE_PATH, str(tmp_path))

    assert len(written) == exp["n_vehicles"]

    # every file name parses as vehicle_{id}_{type}_{size}.csv
    for path in written:
        assert NAME_RE.match(os.path.basename(path)), path

    # unique names, sizes within the sample's VEHICLE length range
    bases = [os.path.basename(p) for p in written]
    assert len(set(bases)) == len(bases)
    sizes = sorted(float(NAME_RE.match(b).group(3)) for b in bases)
    assert exp["size_range"][0] <= sizes[0] and sizes[-1] <= exp["size_range"][1]

    # spot check: the longest-lived sample object
    plongest = next(
        p
        for p in written
        if os.path.basename(p).startswith("vehicle_{0}_".format(exp["longest_idx"]))
    )
    with open(plongest) as f:
        lines = f.read().strip().split("\n")
    assert lines[0] == "x_loc [m],y_loc [m],Time [s],Heading [deg]"
    assert len(lines) == exp["longest_frames"] + 1  # header + frames
    ts = [float(line.split(",")[2]) for line in lines[1:]]
    assert ts == sorted(ts)


@requires_sample
@requires_stack
@requires_known_sample
def test_skip_counts_and_heading_modes(tmp_path) -> None:
    exp = SAMPLE_EXP
    traces, skips = convert.extract_vehicles(SAMPLE_PATH)
    assert len(traces) == exp["n_vehicles"]
    assert skips == exp["skips"]
    assert "VEHICLE" not in skips

    convert.convert_mcap(SAMPLE_PATH, str(tmp_path), heading_mode="straight")
    first = sorted(os.listdir(tmp_path))[0]
    header = open(os.path.join(tmp_path, first)).readline().strip()
    assert header == "x_loc [m],y_loc [m],Time [s]"


NS = "http://code.asam.net/simulation/standard/opendrive_schema"
XODR_WITH_OFFSET = (
    '<?xml version="1.0" encoding="UTF-8"?>\n'
    '<OpenDRIVE xmlns="{ns}">\n'
    '  <header revMajor="1" revMinor="7" name="exp" version="1.00">\n'
    "    <geoReference>+proj=utm +zone=33 +north</geoReference>\n"
    '    <offset x="374703.9227" y="6401996.8046" z="0" hdg="0"/>\n'
    "  </header>\n"
    "</OpenDRIVE>\n"
).format(ns=NS)
XODR_WITHOUT_OFFSET = (
    '<?xml version="1.0" encoding="UTF-8"?>\n'
    '<OpenDRIVE xmlns="{ns}">\n'
    '  <header revMajor="1" revMinor="7" name="exp" version="1.00">\n'
    "    <geoReference>+proj=utm +zone=33 +north</geoReference>\n"
    "  </header>\n"
    "</OpenDRIVE>\n"
).format(ns=NS)


def test_parse_xodr_offset() -> None:
    # namespace-qualified header, as found in COSMO recordings
    ox, oy, oz, hdg = convert.parse_xodr_offset(XODR_WITH_OFFSET)
    assert ox == pytest.approx(374703.9227)
    assert oy == pytest.approx(6401996.8046)
    assert oz == pytest.approx(0.0)
    assert hdg == pytest.approx(0.0)
    # no <offset> element
    assert convert.parse_xodr_offset(XODR_WITHOUT_OFFSET) is None
    # unnamespaced documents work too; hdg is converted to radians;
    # missing attributes default to zero
    plain = '<OpenDRIVE><header><offset x="1.5" hdg="90"/></header></OpenDRIVE>'
    ox, oy, oz, hdg = convert.parse_xodr_offset(plain)
    assert ox == pytest.approx(1.5)
    assert oy == pytest.approx(0.0)
    assert oz == pytest.approx(0.0)
    assert hdg == pytest.approx(math.pi / 2)
    # malformed attribute value
    bad = '<OpenDRIVE><header><offset x="abc"/></header></OpenDRIVE>'
    with pytest.raises(RuntimeError):
        convert.parse_xodr_offset(bad)


def test_apply_xodr_offset_to_rows() -> None:
    rows = [(10.0, 20.0, 0.0, 30.0)]
    # hdg = 0: pure translation, z ignored, time and heading untouched
    out = convert.apply_xodr_offset_to_rows(rows, (5.0, 7.0, 1.0, 0.0))
    assert len(out) == 1
    assert out[0][0] == pytest.approx(5.0)
    assert out[0][1] == pytest.approx(13.0)
    assert out[0][2] == pytest.approx(0.0)
    assert out[0][3] == pytest.approx(30.0)
    # hdg = 90 deg: geodetic = offset + R(hdg) * local, so
    # local = R(-90) * (10, 20) = (20, -10) and the heading rotates by +90
    out = convert.apply_xodr_offset_to_rows(rows, (0.0, 0.0, 0.0, math.pi / 2))
    assert len(out) == 1
    assert out[0][0] == pytest.approx(20.0)
    assert out[0][1] == pytest.approx(-10.0)
    assert out[0][2] == pytest.approx(0.0)
    assert out[0][3] == pytest.approx(120.0)


def test_xodr_offset_from_recording_errors() -> None:
    from types import SimpleNamespace

    def fake_recording(odr_xml):
        if odr_xml is None:
            return SimpleNamespace(map=None)
        return SimpleNamespace(map=SimpleNamespace(odr_xml=odr_xml))

    # no map at all
    with pytest.raises(RuntimeError, match="No OpenDrive map"):
        convert.xodr_offset_from_recording(fake_recording(None), "rec.mcap")
    # map without an <offset> element
    with pytest.raises(RuntimeError, match="no <offset> element"):
        convert.xodr_offset_from_recording(
            fake_recording(XODR_WITHOUT_OFFSET), "rec.mcap"
        )
    # map with an <offset> element
    offset = convert.xodr_offset_from_recording(
        fake_recording(XODR_WITH_OFFSET), "rec.mcap"
    )
    assert offset[0] == pytest.approx(374703.9227)
    assert offset[1] == pytest.approx(6401996.8046)
    assert offset[2] == pytest.approx(0.0)
    assert offset[3] == pytest.approx(0.0)


def _read_csv_rows(path: str):
    with open(path) as f:
        lines = f.read().strip().split("\n")[1:]
    return [
        (
            float(parts[0]),
            float(parts[1]),
            float(parts[2]),
            float(parts[3]) if len(parts) > 3 else None,
        )
        for parts in (line.split(",") for line in lines)
    ]


@requires_sample
@requires_stack
def test_apply_xodr_offset_end_to_end(tmp_path) -> None:
    from omega_prime import Recording

    # expected offset, read from the sample's own embedded map
    xodr = Recording.from_file(SAMPLE_PATH).map.odr_xml
    ox, oy, _oz, hdg = convert.parse_xodr_offset(xodr)
    assert hdg == 0.0  # the sample's offset is a pure translation

    dir_raw = tmp_path / "raw"
    dir_offset = tmp_path / "offset"
    written_raw = convert.convert_mcap(SAMPLE_PATH, str(dir_raw))
    written_offset = convert.convert_mcap(
        SAMPLE_PATH, str(dir_offset), apply_xodr_offset=True
    )
    assert 0 < len(written_raw) == len(written_offset)

    by_name = {os.path.basename(p): p for p in written_raw}
    for path in written_offset:
        raw_rows = _read_csv_rows(by_name[os.path.basename(path)])
        off_rows = _read_csv_rows(path)
        assert len(raw_rows) == len(off_rows)
        for (rx, ry, rt, rh), (oxv, oyv, ot, oh) in zip(raw_rows, off_rows):
            assert oxv == pytest.approx(rx - ox, abs=1e-6)
            assert oyv == pytest.approx(ry - oy, abs=1e-6)
            assert ot == rt
            assert oh == rh

    # sanity: after the transform at least one point per axis lies close to
    # the local frame's origin
    min_abs_x = min_abs_y = float("inf")
    for path in written_offset:
        for x, y, _t, _h in _read_csv_rows(path):
            min_abs_x = min(min_abs_x, abs(x))
            min_abs_y = min(min_abs_y, abs(y))
    assert min_abs_x < 150.0
    assert min_abs_y < 150.0
