export DIRECT=0
python run.py \
  45 \
  ../../scenes/roundabout-saro/maps/saro_fixed_bidirectional_WIDER_LANES.xodr \
  ../../scenes/roundabout-saro/npc_test_routes/at_big_roundabout.csv \
  --movement teleport \
  --timestep 0.0333333333333333333 \
  --pcla_agent tfpp_wp_1 \
  --pcla_route ../../scenes/roundabout-saro/agent_routes/across_big_roundabout.xml \
  --ego_camera