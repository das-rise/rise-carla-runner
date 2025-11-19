from carla2traj import Carla2Traj
from traj2osi import Traj2OSI

traj = Carla2Traj.from_file(
    "/THISREPO/RiRun/traj_20251112_101757.parquet"
)  # Load existing trajectory data

traj.convert(
    Traj2OSI,
    "output.osi",
    converter_args={
        "country_code": 752,
        "version": "0.1.0",
        "proj_string": "",
        "map_reference": "Town01.xodr",
    },
)  # Convert and save to OSI format
