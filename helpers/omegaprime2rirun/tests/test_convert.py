"""End-to-end tests for the OmegaPrime -> rirun CSV conversion.

Requires the omega-prime/betterosi stack (this project's uv environment,
Python >= 3.10) and the sample recording at the repository root. Skips if
either is unavailable.
"""

import glob
import os
import re

import pytest
from omegaprime2rirun import convert

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
SAMPLE_PATHS = sorted(glob.glob(os.path.join(REPO_ROOT, "OmegaPrime_*.mcap")))
SAMPLE_PATH = SAMPLE_PATHS[0] if SAMPLE_PATHS else None

try:
    import omega_prime  # noqa: F401

    HAS_STACK = True
except ImportError:
    HAS_STACK = False

requires_sample = pytest.mark.skipif(
    SAMPLE_PATH is None, reason="sample OmegaPrime recording not present"
)
requires_stack = pytest.mark.skipif(not HAS_STACK, reason="omega-prime not installed")

NAME_RE = re.compile(r"^vehicle_(\d+)_([A-Za-z]+)_(\d+(?:\.\d+)?)\.csv$")


@requires_sample
@requires_stack
def test_convert_sample_mcap(tmp_path) -> None:
    written = convert.convert_mcap(SAMPLE_PATH, str(tmp_path))

    # the sample has 148 VEHICLE objects, 9 of them with <3 frames (skipped)
    assert len(written) == 139

    # every file name parses as vehicle_{id}_{type}_{size}.csv
    for path in written:
        assert NAME_RE.match(os.path.basename(path)), path

    # unique names, sizes within the sample's VEHICLE length range
    bases = [os.path.basename(p) for p in written]
    assert len(set(bases)) == len(bases)
    sizes = sorted(float(NAME_RE.match(b).group(3)) for b in bases)
    assert 3.5 <= sizes[0] and sizes[-1] <= 8.9

    # spot check: idx 184 is the longest-lived sample object (1422 frames)
    p184 = next(p for p in written if os.path.basename(p).startswith("vehicle_184_"))
    with open(p184) as f:
        lines = f.read().strip().split("\n")
    assert lines[0] == "x_loc [m],y_loc [m],Time [s],Heading [deg]"
    assert len(lines) == 1423  # header + 1422 frames
    ts = [float(line.split(",")[2]) for line in lines[1:]]
    assert ts == sorted(ts)


@requires_sample
@requires_stack
def test_skip_counts_and_heading_modes(tmp_path) -> None:
    traces, skips = convert.extract_vehicles(SAMPLE_PATH)
    assert len(traces) == 139
    # the sample's 108 OTHER objects are skipped by default, plus the 9
    # VEHICLE objects that have fewer than the default 3 frames
    assert skips.get("OTHER") == 108
    assert skips.get("TOO_FEW_FRAMES") == 9
    assert "VEHICLE" not in skips

    convert.convert_mcap(SAMPLE_PATH, str(tmp_path), heading_mode="straight")
    first = sorted(os.listdir(tmp_path))[0]
    header = open(os.path.join(tmp_path, first)).readline().strip()
    assert header == "x_loc [m],y_loc [m],Time [s]"
