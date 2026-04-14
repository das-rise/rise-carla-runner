export DIRECT=0
python src/rirun/run.py \
  45 \
  scenes/roundabout-saro/maps/saro_fixed_bidirectional_WIDER_LANES.xodr \
  --movement teleport \
  --timestep 0.0333333333333333333 \
  --pcla_agent tfpp_wp_1 \
  --pcla_route scenes/roundabout-saro/agent_routes/across_big_roundabout.xml \
  --camera_mode overhead \
  --overhead_camera_position 1651 -715 150