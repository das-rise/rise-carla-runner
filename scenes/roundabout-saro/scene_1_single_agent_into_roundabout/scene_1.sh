export DIRECT=0
python src/rirun/run.py \
  45 \
  scenes/roundabout-saro/maps/SaroRound_2lane2_shifted.xodr \
  saro_roundabout_converted/vehicle_*_Car.csv \
  --movement pid \
  --timestep 0.0333333333333333333 \
  --offset_time 1013 \
  --camera_mode overhead \
  --overhead_camera_position 100 -49 150 \
  --trajectory_statistics average_distance_true \
  # --pcla_agent tfpp_wp_1 \
  # --pcla_route scenes/roundabout-saro/scene_1_single_agent_into_roundabout/agent.xml