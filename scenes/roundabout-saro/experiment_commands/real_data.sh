export DIRECT=0
python src/rirun/run.py \
  40 \
  scenes/roundabout-saro/maps/saro_fixed_bidirectional_WIDER_LANES.xodr \
  saro_roundabout_converted/vehicle_*Car.csv \
  --movement teleport \
  --timestep 0.0333333333333333333 \
  --offset 1013 \
  --camera_mode overhead \
  --overhead_camera_position 1700 -700 75