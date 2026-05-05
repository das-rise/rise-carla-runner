export DIRECT=0
python src/rirun/run.py \
  40 \
  scenes/roundabout-saro/maps/SaroRound_2lane2_shifted_wider.xodr \
  --npc_trajectory_filepaths scenes/roundabout-saro/npc_test_routes/at_big_roundabout.csv \
  scenes/roundabout-saro/npc_test_routes/vehicle_*Car.csv \
  --npc_movement teleport \
  --timestep 0.0333333333333333333 \
  --offset_time 1013 \
  --record_cameras stationary_overhead \
  --overhead_camera_position 100 -49 150