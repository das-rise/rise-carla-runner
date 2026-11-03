"""Vehicle model selection and color assignment for NPC vehicles.

NPC trajectory files are named ``vehicle_{vehicle_id}_{vehicle_type}_{size}.csv``
where ``size`` is the length of the vehicle in meters.  This module maps a
vehicle length to the CARLA vehicle model whose overall length is closest,
and derives a deterministic per-vehicle color from the file name.

The color red (the hue band around 0/360 deg) is reserved for PCLA agents,
which always spawn in red (see :data:`PCLA_RESERVED_COLOR`).
"""

import colorsys
import hashlib
import os
import re
from enum import Enum
from typing import Optional, Tuple

import carla


class VehicleModel(Enum):
    """Relevant 4-wheeled CARLA vehicles: blueprint id -> (length, height).

    Lengths and heights are real-world overall dimensions (approximate,
    +/- 0.3 m / +/- 0.1 m), which is good enough for "closest length"
    matching and for placing the vehicle on the road at spawn time.  Sizes
    outside the listed range clamp to the closest listed vehicle.
    """

    AUDI_A2 = ("vehicle.audi.a2", 3.85, 1.42)
    AUDI_TT = ("vehicle.audi.tt", 4.18, 1.36)
    AUDI_ETRON = ("vehicle.audi.etron", 4.90, 1.73)
    BMW_GRANDTOURER = ("vehicle.bmw.grandtourer", 4.94, 1.70)
    CHEVROLET_IMPALA = ("vehicle.chevrolet.impala", 5.18, 1.43)
    CITROEN_C3 = ("vehicle.citroen.c3", 4.05, 1.45)
    DODGE_CHARGER_2020 = ("vehicle.dodge.charger_2020", 5.06, 1.45)
    DODGE_CHARGER_POLICE = ("vehicle.dodge.charger_police", 5.06, 1.36)
    DODGE_CHARGER_POLICE_2020 = ("vehicle.dodge.charger_police_2020", 5.06, 1.45)
    FORD_AMBULANCE = ("vehicle.ford.ambulance", 5.90, 2.60)
    FORD_CROWN = ("vehicle.ford.crown", 5.07, 1.47)
    FORD_MUSTANG = ("vehicle.ford.mustang", 4.84, 1.33)
    JEEP_WRANGLER_RUBICON = ("vehicle.jeep.wrangler_rubicon", 4.90, 1.90)
    LINCOLN_MKZ_2017 = ("vehicle.lincoln.mkz_2017", 5.00, 1.49)
    LINCOLN_MKZ_2020 = ("vehicle.lincoln.mkz_2020", 5.04, 1.49)
    MERCEDES_COUPE = ("vehicle.mercedes.coupe", 4.59, 1.41)
    MERCEDES_COUPE_2020 = ("vehicle.mercedes.coupe_2020", 4.97, 1.40)
    MERCEDES_SPRINTER = ("vehicle.mercedes.sprinter", 5.26, 2.00)
    MINI_COOPER_S = ("vehicle.mini.cooper_s", 3.83, 1.42)
    MINI_COOPER_S_2021 = ("vehicle.mini.cooper_s_2021", 3.83, 1.43)
    MITSUBISHI_FUSOROSA = ("vehicle.mitsubishi.fusorosa", 7.00, 2.50)
    NISSAN_MICRA = ("vehicle.nissan.micra", 3.99, 1.45)
    NISSAN_PATROL = ("vehicle.nissan.patrol", 4.86, 1.88)
    NISSAN_PATROL_2021 = ("vehicle.nissan.patrol_2021", 4.86, 1.88)
    SEAT_LEON = ("vehicle.seat.leon", 4.57, 1.45)
    TESLA_MODEL3 = ("vehicle.tesla.model3", 4.69, 1.44)
    TESLA_CYBERTRUCK = ("vehicle.tesla.cybertruck", 5.68, 1.79)
    TOYOTA_PRIUS = ("vehicle.toyota.prius", 4.54, 1.50)
    VOLKSWAGEN_T2 = ("vehicle.volkswagen.t2", 4.28, 1.94)
    CARLA_COLA = ("vehicle.carlamotors.carlacola", 4.20, 2.00)

    @property
    def blueprint(self) -> str:
        """CARLA blueprint id of this vehicle model."""
        return self.value[0]

    @property
    def length(self) -> float:
        """Overall vehicle length in meters."""
        return self.value[1]

    @property
    def height(self) -> float:
        """Overall vehicle height in meters."""
        return self.value[2]

    @classmethod
    def best_match(cls, length_m: float) -> "VehicleModel":
        """Return the model whose overall length is closest to ``length_m``.

        Args:
            length_m: Vehicle length in meters.
        """
        return min(cls, key=lambda m: abs(m.length - length_m))


#: File name convention for NPC trajectory files (size in meters).
#: The vehicle type may be mixed case (e.g. ``vehicle_0_Car_5.7.csv``).
VEHICLE_FILENAME_RE = re.compile(
    r"^vehicle_(?P<vehicle_id>.+?)_(?P<vehicle_type>[A-Za-z]+)_(?P<size>\d+(?:\.\d+)?)\.csv$",
)


def parse_vehicle_filename(path: str) -> Optional[Tuple[str, str, float]]:
    """Parse a file name of the form
    ``vehicle_{vehicle_id}_{vehicle_type}_{size}.csv``.

    Args:
        path: Path (or plain file name) of the trajectory CSV.

    Returns:
        Tuple of (vehicle_id, vehicle_type, size in meters), or ``None`` if
        the file name does not follow the convention (e.g. legacy names).
    """
    match = VEHICLE_FILENAME_RE.match(os.path.basename(path))
    if match is None:
        return None
    return (
        match["vehicle_id"],
        match["vehicle_type"],
        float(match["size"]),
    )


#: Color reserved for PCLA agents, which are always red.
PCLA_RESERVED_COLOR = carla.Color(255, 0, 0)

#: Hue band (in degrees) around red (0/360 deg) reserved for PCLA agents.
#: The band is excluded from NPC colors; quantizing to 8-bit RGB can shift a
#: hue by up to ~2 deg, so the excluded band is a bit wider than the
#: "looks red" band of +/- 12 deg.
_RED_HUE_BAND_DEG = 15.0


def _rgb_from_filename(path: str) -> Tuple[int, int, int]:
    """Deterministic (r, g, b) in 0-255 derived from the file name.

    The hue is taken from a stable sha256 digest (Python's built-in ``hash``
    is salted per process and must not be used) and kept out of the red band
    reserved for PCLA agents.  Saturation and value are derived from further
    digest bytes so that colors stay vivid and distinguishable.
    """
    digest = hashlib.sha256(os.path.basename(path).encode("utf-8")).digest()
    hue = int.from_bytes(digest[:2], "big") % 360
    if hue < _RED_HUE_BAND_DEG or hue >= 360.0 - _RED_HUE_BAND_DEG:
        hue = (hue + 180) % 360
    saturation = 0.45 + 0.35 * (digest[2] / 255)
    value = 0.45 + 0.35 * (digest[3] / 255)
    r, g, b = colorsys.hsv_to_rgb(hue / 360.0, saturation, value)
    return int(r * 255), int(g * 255), int(b * 255)


def color_from_filename(path: str) -> carla.Color:
    """Deterministic, non-red NPC color derived from the trajectory file name.

    The same file name always yields the same color, and the result is never
    inside the red hue band reserved for PCLA agents (see
    :data:`PCLA_RESERVED_COLOR`).

    Args:
        path: Path (or plain file name) of the trajectory CSV.
    """
    r, g, b = _rgb_from_filename(path)
    return carla.Color(r, g, b)
