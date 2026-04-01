export DIRECT=0
python src/rirun/run.py \
  45 \
  scenes/roundabout-saro/scene_1_single_agent_into_roundabout/SaroRound_2lane_u_adj_14.xodr \
  --trajectory-filepaths saro_roundabout_converted4/*Car.csv \
  --movement teleport \
  --timestep 0.0333333333333333333 \
  --offset_time 1013 \
  --pcla_agent tfpp_wp_1 \
  --pcla_route scenes/roundabout-saro/scene_1_single_agent_into_roundabout/scene_1_3_single_agent_into_roundabout.xml \
  --camera_mode overhead \
  --overhead_camera_position 0 0 150