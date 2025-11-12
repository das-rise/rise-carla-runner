# Requirements for OSI in Synergies

According to the [OmegaPrime](https://github.com/ika-rwth-aachen/omega-prime/blob/main/docs/omega_prime_specification.md) specification, the requirements are as follows:

## Content

The following information is to be repeated for every frame / every `GroundTruthMessage`:

|**Signal hierarchy**|**Data model and type**|**Minimal Accuracy**|*implemented* | *tested* |
|---|---|---|---|---|
|map_reference|str<br><br>Content depends on the chosen map association option (see 'Map reference' below).|| X |
|country_code|int [3 digit ISO country code (e.g. germany=276,usa=840)].|| X |
|version|InterfaceVersion| | X |
|. version_major|int| | X |
|. version_minor|int| | X |
|. version_patch|int| | X |
|proj_frame_offset|GroundTruthProjFrameOffset|0,2 m| X |
|. position|Vector3D|0,2 m|  X |
|. yaw|float|0,035 rad (2°)|  X |
|proj_string|str [PROJ coordinate transformation software library]<br><br>Mandatory for real world data; Can be omitted for simulation data.|| X |
|timestamp|Timestamp (total time is combination of seconds and nanos)|| X |
|. nanos|int|| X |
|. seconds|int|| X |
|host_vehicle_id|Identifier||
|. value|int||
|moving_object|list[MovingObject]||
|. id|Identifier||
|. . value|int||
|. base|BaseMoving||
|. . dimension|Dimension3D||
|. . . x|float [m]|0,2 m|
|. . . y|float [m]|0,2 m|
|. . . z|float [m]|0,2 m|
|. . position|Vector3D||
|. . . x|float [m]|0,2 m|
|. . . y|float [m]|0,2 m|
|. . . z|float [m]|0,2 m|
|. . orientation|Orientation3D||
|. . . roll|float [rad]|0,035 rad (2°)|
|. . . pitch|float [rad]|0,035 rad (2°)|
|. . . yaw|float [rad]|0,035 rad (2°)|
|. . velocity|Vector3D|0,1 m/s|
|. . acceleration|Vector3D|0,1 m/s^2|
|. type|MovingObjectType<br><br>(Other, Vehicle, Pedestrian, Animal)||
|. vehicle_classification|MovingObjectVehicleClassification||
|. . type|MovingObjectVehicleClassificationType<br><br>(Other, car, delivery van, semitrailer, trailer, motorbike, bicycle, bus, tram, train, wheelchair, standup scooter)||
|. . role|MovingObjectVehicleClassificationRole<br><br>(Other, civil, ambulance, fire, police, public transport, road assistance, garbage collectin, road construction, military)||
|traffic_light|list[TrafficLight]||
|. id|Identifier||
|. . value|int||
|. base (optional)|BaseStationary<br><br>The position of traffic lights is given in the associated OpenDRIVE map file. If optionally given in the OSI message, it must match the corresponding data in the map file. The association between OSI and OpenDRIVE objects is established using the 'source_reference' field.||
|. classification|TrafficLightClassification||
|. . color|TrafficLightClassificationColor<br><br>(Other, red, yellow, green, blue, white)||
|. . icon|TrafficLightClassificationIcon<br><br>(Other, none, arrow_straight_ahead, arrow_left, arrow_diag_left, arrow_straight_ahead_left, arrow_right, ...)||
|. . mode|TrafficLightClassificationMode<br><br>(Other, off, constant, flashing, counting)||
|. . counter|float||
|. . is_out_of_service|bool||
|. source_reference|list[ExternalReference]<br><br>The source reference maps the dynamic OSI traffic light information to the static traffic light information in the OpenDRIVE map.||

## Format specification

The following rules apply to OMEGA-PRIME multi-channel trace files:

- The OMEGA-PRIME OSI GroundTruth message stream must be stored as a compliant OSI channel as specified by the OSI MCAP format specification.
- The channel name of the OMEGA-PRIME OSI GroundTruth data must be \ground_truth.
- The OMEGA-PRIME OSI GroundTruth message interface must comply with the required subset as defined in the table in section 'Dynamic Information'.
- The version of the OMEGA-PRIME OSI GroundTruth messages must be >=3.7.0.
- The message frequency of consecutive OMEGA-PRIME OSI GroundTruth messages must be 10Hz or higher.

## Map reference

![MCAP file with embedded OpenDRIVE map](mcap_file.png)

We make the design choice to store the `OpenDrive` map in the MCAP topic `ground_truth_map`, ensuring that the association between map and ground truth can not be lost. Map references must be ensured to match!