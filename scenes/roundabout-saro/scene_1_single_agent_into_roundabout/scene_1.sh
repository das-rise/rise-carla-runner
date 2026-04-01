python src/rirun/run.py \
  45 \
  scenes/roundabout-saro/maps/SaroRound_2lane_u_adj.xodr \
  --trajectory-filepaths saro_roundabout_converted3/*.csv \
  --movement pid \
  --timestep 0.0333333333333333333 \
  --offset_time 1013 \
  --camera_mode overhead \
  --overhead_camera_position 0 0 150 \
  --trajectory_statistics average_distance_true \
  --pcla_agent neat_aim2dsem \
  --pcla_route scenes/roundabout-saro/scene_1_single_agent_into_roundabout/agent.xml