export DIRECT=0
python src/rirun/run.py \
  40 \
  scenes/roundabout-saro/maps/SaroRound_2lane2_shifted_wider.xodr \
  scenes/roundabout-saro/npc_test_routes/vehicle_*Car.csv \
  --movement teleport \
  --timestep 0.0333333333333333333 \
  --offset_time 1013 \
  --camera_mode overhead \
  --overhead_camera_position 100 -49 150