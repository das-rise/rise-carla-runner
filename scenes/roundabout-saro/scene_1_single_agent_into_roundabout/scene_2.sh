export DIRECT=0
python src/rirun/run.py \
  45 \
  scenes/roundabout-saro/maps/saro_fixed_bidirectional_wider_2lane_gemini.xodr \
  --movement teleport \
  --timestep 0.0333333333333333333 \
  --offset_time 1013 \
  --pcla_agent tfpp_wp_1 \
  --pcla_route scenes/roundabout-saro/scene_1_single_agent_into_roundabout/agent_2.xml \
  --camera_mode overhead \
  --overhead_camera_position 1651 -715 150