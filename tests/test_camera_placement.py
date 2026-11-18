"""Tests for rirun.utils.camera_placement (no CARLA server required).

Run with pytest, or standalone:  python tests/test_camera_placement.py
"""

import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from rirun.utils.camera_placement import (  # noqa: E402
    STATIONARY_CAMERA_MAX_DISTANCE_M,
    euclidean_distance,
    point_centroid,
    stationary_camera_warning,
)

# ---------------------------------------------------------------------------
# point_centroid
# ---------------------------------------------------------------------------


def test_centroid_empty() -> None:
    assert point_centroid([]) is None
    assert point_centroid(()) is None


def test_centroid_single_point() -> None:
    assert point_centroid([(10.0, 20.0, 30.0)]) == (10.0, 20.0, 30.0)


def test_centroid_multiple_points() -> None:
    pts = [(0.0, 0.0, 0.0), (10.0, 0.0, 0.0), (0.0, 10.0, 0.0), (0.0, 0.0, 10.0)]
    assert point_centroid(pts) == (2.5, 2.5, 2.5)


def test_centroid_accepts_iterable_of_locations() -> None:
    # carla.Location-like objects (x/y/z attributes) are not used here; the
    # function consumes plain sequences, as run.py builds them.
    pts = (
        (374626.0, 6401934.0, 0.0),
        (374752.0, 6402076.0, 0.0),
    )
    cx, cy, cz = point_centroid(pts)
    assert cx == 374689.0
    assert cy == 6402005.0
    assert cz == 0.0


# ---------------------------------------------------------------------------
# euclidean_distance
# ---------------------------------------------------------------------------


def test_distance_zero() -> None:
    assert euclidean_distance((1.0, 2.0, 3.0), (1.0, 2.0, 3.0)) == 0.0


def test_distance_3d() -> None:
    assert euclidean_distance((0.0, 0.0, 0.0), (3.0, 4.0, 0.0)) == 5.0
    assert math.isclose(euclidean_distance((0.0, 0.0, 0.0), (3.0, 4.0, 12.0)), 13.0)


# ---------------------------------------------------------------------------
# stationary_camera_warning
# ---------------------------------------------------------------------------

# Vehicle cloud at UTM-scale coordinates, as in the BorasEkas DTT scene.
CLOUD = (374689.0, 6402005.0, 0.0)
CLOUD_LABEL = "vehicle cloud centroid"


def test_no_warning_inside_threshold() -> None:
    cam = (374800.0, 6402100.0, 150.0)  # ~150 m away
    assert stationary_camera_warning(cam, CLOUD, CLOUD_LABEL) is None


def test_no_warning_at_exact_threshold() -> None:
    cam = (374689.0 + 2000.0, 6402005.0, 0.0)
    assert STATIONARY_CAMERA_MAX_DISTANCE_M == 2000.0
    assert stationary_camera_warning(cam, CLOUD, CLOUD_LABEL) is None


def test_warning_beyond_threshold() -> None:
    cam = (0.0, 0.0, 150.0)  # world origin, ~6400 km from the cloud
    msg = stationary_camera_warning(cam, CLOUD, CLOUD_LABEL)
    assert msg is not None
    # reports the distance, the reference point, and a fix
    assert "6413.0 km" in msg
    assert "(374689, 6402005, 0)" in msg
    assert "--overhead_camera_position 374689 6402005 150" in msg
    # mentions the failure mode
    assert "SIGSEGV" in msg
    assert "LargeMapManager" in msg


def test_no_warning_without_reference_point() -> None:
    assert stationary_camera_warning((0.0, 0.0, 150.0), None, CLOUD_LABEL) is None


def test_custom_threshold() -> None:
    cam = (1000.0, 0.0, 0.0)
    assert stationary_camera_warning(cam, (0.0, 0.0, 0.0), "map center", 500.0)
    assert stationary_camera_warning(cam, (0.0, 0.0, 0.0), "map center", 5000.0) is None


if __name__ == "__main__":
    import traceback

    failures = 0
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            try:
                fn()
                print(f"PASS {name}")
            except Exception:
                failures += 1
                print(f"FAIL {name}")
                traceback.print_exc()
    print(f"\n{'all tests passed' if failures == 0 else f'{failures} test(s) failed'}")
    raise SystemExit(1 if failures else 0)
