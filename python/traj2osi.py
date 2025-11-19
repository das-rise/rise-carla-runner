from typing import Any
import betterosi
from traj2x import Traj2X
from traj import TrajDF

NANOS_PER_SECOND = 1_000_000_000


class Traj2OSI(Traj2X):
    """Converter to OSI format as specified in SYNERGIES OmegaPrime, https://github.com/ika-rwth-aachen/omega-prime/blob/main/docs/omega_prime_specification.md."""

    def __init__(
        self,
        traj_df: TrajDF,
        country_code: int,
        version: str,
        proj_string: str = "",
        map_reference: str = "",
    ):
        super().__init__(traj_df)
        self._country_code = country_code
        self._version_major, self._version_minor, self._version_patch = map(
            int, version.split(".")
        )
        self._proj_string = proj_string
        self._map_reference = map_reference

    def __call__(self, output_path: str):
        # Implement conversion to OSI format as defined in https://github.com/ika-rwth-aachen/omega-prime/blob/main/docs/omega_prime_specification.md

        host_vehicle_id = None

        with betterosi.Writer(output_path) as writer_osi:
            for frame_data in self._traverse_frames():
                # for now, we only register moving objects
                moving_objects = []
                for object in frame_data.iter_rows(named=True):

                    # For now, we assume the first object is the host vehicle
                    host_vehicle_id = (
                        betterosi.Identifier(object["movingobject_id"])
                        if host_vehicle_id is None
                        else host_vehicle_id
                    )

                    base = self._get_base_moving(object)

                    # TODO: vehicle classification mapping
                    moving_objects.append(
                        betterosi.MovingObject(
                            id=betterosi.Identifier(object["movingobject_id"]),
                            type=self._get_movingobject_type(object),
                            vehicle_classification=betterosi.MovingObjectVehicleClassification(
                                type=betterosi.MovingObjectVehicleClassificationType.UNKNOWN,
                                role=betterosi.MovingObjectVehicleClassificationRole.UNKNOWN,
                            ),
                            base=base,
                        )
                    )
                # Create frame
                timestamp = frame_data["timestamp"][0]
                timestamp_seconds = int(timestamp)
                timestamp_nanos = int(
                    (timestamp - timestamp_seconds) * NANOS_PER_SECOND
                )

                ground_truth_frame = betterosi.GroundTruth(
                    version=betterosi.InterfaceVersion(
                        version_major=self._version_major,
                        version_minor=self._version_minor,
                        version_patch=self._version_patch,
                    ),
                    country_code=self._country_code,
                    map_reference=self._map_reference,
                    proj_string=self._proj_string,
                    proj_frame_offset=betterosi.GroundTruthProjFrameOffset(
                        position=betterosi.Vector3D(x=0.0, y=0.0, z=0.0), yaw=0.0
                    ),
                    timestamp=betterosi.Timestamp(
                        seconds=timestamp_seconds, nanos=timestamp_nanos
                    ),
                    moving_object=moving_objects,
                    host_vehicle_id=host_vehicle_id,
                )

                writer_osi.add(ground_truth_frame)

    def _get_movingobject_type(
        self, object: dict[str, Any]
    ) -> betterosi.MovingObjectType:
        type_str = object["type"]
        if type_str == "Vehicle":
            return betterosi.MovingObjectType.VEHICLE
        elif type_str == "Pedestrian":
            return betterosi.MovingObjectType.PEDESTRIAN
        elif type_str == "Animal":
            return betterosi.MovingObjectType.ANIMAL
        elif type_str == "Other":
            return betterosi.MovingObjectType.OTHER
        else:
            return betterosi.MovingObjectType.UNKNOWN

    def _get_base_moving(self, object: dict[str, Any]) -> betterosi.BaseMoving:
        return betterosi.BaseMoving(
            dimension=betterosi.Dimension3D(
                length=object["dimension_x"],
                width=object["dimension_y"],
                height=object["dimension_z"],
            ),
            position=betterosi.Vector3D(
                x=object["position_x"],
                y=object["position_y"],
                z=object["position_z"],
            ),
            orientation=betterosi.Orientation3D(
                roll=object["orientation_x"],
                pitch=object["orientation_y"],
                yaw=object["orientation_z"],
            ),
            velocity=betterosi.Vector3D(
                x=object["velocity"][0],
                y=object["velocity"][1],
                z=object["velocity"][2],
            ),
            acceleration=betterosi.Vector3D(
                x=object["acceleration"][0],
                y=object["acceleration"][1],
                z=object["acceleration"][2],
            ),
        )
