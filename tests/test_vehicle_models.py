"""Tests for rirun.kinetics.vehicle_models (no CARLA server required).

Run with pytest, or standalone:  python tests/test_vehicle_models.py
"""

import colorsys
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

import carla  # noqa: E402

from rirun.kinetics.vehicle_models import (  # noqa: E402
    _RED_HUE_BAND_DEG,
    PCLA_RESERVED_COLOR,
    VehicleModel,
    color_from_filename,
    parse_vehicle_filename,
)

# ---------------------------------------------------------------------------
# Filename parsing
# ---------------------------------------------------------------------------


def test_parse_valid_filenames() -> None:
    assert parse_vehicle_filename("vehicle_12_car_4.5.csv") == ("12", "car", 4.5)
    # mixed-case vehicle type occurs in real file names
    assert parse_vehicle_filename("vehicle_0_Car_5.7.csv") == ("0", "Car", 5.7)
    assert parse_vehicle_filename("vehicle_7_truck_6.0.csv") == ("7", "truck", 6.0)
    assert parse_vehicle_filename("vehicle_7_bus_10.csv") == ("7", "bus", 10.0)
    # full paths and ids containing underscores work
    assert parse_vehicle_filename("/some/dir/vehicle_abc_12_car_4.5.csv") == (
        "abc_12",
        "car",
        4.5,
    )


def test_parse_invalid_filenames() -> None:
    assert parse_vehicle_filename("vehicle1_straight.csv") is None  # legacy name
    assert parse_vehicle_filename("vehicle_1_car.csv") is None  # no size
    assert parse_vehicle_filename("vehicle_1_car_abc.csv") is None  # bad size
    assert parse_vehicle_filename("vehicle_1_car_4.5") is None  # no .csv
    assert parse_vehicle_filename("car_4.5.csv") is None  # no vehicle_ prefix


# ---------------------------------------------------------------------------
# VehicleModel enum
# ---------------------------------------------------------------------------


def test_enum_sanity() -> None:
    assert len(VehicleModel) >= 20
    blueprints = [m.blueprint for m in VehicleModel]
    assert len(set(blueprints)) == len(blueprints), "blueprint ids must be unique"
    for m in VehicleModel:
        assert m.blueprint.startswith("vehicle.")
        assert m.length > 0
        # overall heights: passenger cars to buses/trucks
        assert 1.0 <= m.height <= 3.0


def test_best_match_known() -> None:
    assert VehicleModel.best_match(4.5).length == 4.54  # Toyota Prius, delta 0.06
    assert VehicleModel.best_match(3.83).length == 3.83  # Mini Cooper S, exact
    assert VehicleModel.best_match(5.68).length == 5.68  # Cybertruck, exact


def test_best_match_clamps_out_of_range() -> None:
    shortest = min(m.length for m in VehicleModel)
    longest = max(m.length for m in VehicleModel)
    assert VehicleModel.best_match(0.0).length == shortest
    assert VehicleModel.best_match(100.0).length == longest
    # result is always an enum member with a valid blueprint
    assert isinstance(VehicleModel.best_match(4.7), VehicleModel)


def test_best_match_is_closest() -> None:
    # for a grid of lengths the returned model must be the true minimum
    lengths = [m.length for m in VehicleModel]
    for probe in (x / 10.0 for x in range(20, 81)):
        expected = min(lengths, key=lambda length: abs(length - probe))
        assert abs(VehicleModel.best_match(probe).length - expected) < 1e-9


# ---------------------------------------------------------------------------
# Color assignment
# ---------------------------------------------------------------------------

_SYNTHETIC_FILES = [
    f"vehicle_{i}_{kind}_{size}.csv"
    for i in range(60)
    for kind in ("car", "truck", "bus")
    for size in ("4.0", "4.5", "5.0")
]


def test_color_deterministic() -> None:
    assert color_from_filename("vehicle_1_car_4.5.csv") == color_from_filename(
        "vehicle_1_car_4.5.csv"
    )
    # only the basename matters
    assert color_from_filename("vehicle_1_car_4.5.csv") == color_from_filename(
        "/a/b/c/vehicle_1_car_4.5.csv"
    )


def test_color_distinct_per_file() -> None:
    colors = {(c.r, c.g, c.b) for c in map(color_from_filename, _SYNTHETIC_FILES)}
    assert len(colors) >= 90, f"expected mostly distinct colors, got {len(colors)}"


def _hue_deg(c: carla.Color) -> float:
    h, _, _ = colorsys.rgb_to_hsv(c.r / 255, c.g / 255, c.b / 255)
    return h * 360.0


def test_color_never_in_red_band() -> None:
    lo = _RED_HUE_BAND_DEG + 0.5
    hi = 360.0 - _RED_HUE_BAND_DEG - 0.5
    for f in _SYNTHETIC_FILES:
        hue = _hue_deg(color_from_filename(f))
        assert lo <= hue <= hi, f"{f}: hue {hue} inside red band"
        assert color_from_filename(f) != PCLA_RESERVED_COLOR


def test_color_is_carla_color() -> None:
    c = color_from_filename("vehicle_1_car_4.5.csv")
    assert isinstance(c, carla.Color)
    assert all(0 <= v <= 255 for v in (c.r, c.g, c.b))
    assert (PCLA_RESERVED_COLOR.r, PCLA_RESERVED_COLOR.g, PCLA_RESERVED_COLOR.b) == (
        255,
        0,
        0,
    )


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
