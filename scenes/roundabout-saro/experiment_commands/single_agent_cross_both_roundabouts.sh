python run.py 
    100 \
    ../../scenes/roundabout-saro/maps/saro_fixed_bidirectional_WIDER_LANES.xodr \
    ../../scenes/roundabout-saro/npc_test_routes/at_big_roundabout.csv \
    --movement teleport \
    --timestep 0.0333333333333333333 \
    --pcla_agent neat_aim2dsem \
    --pcla_route ../../scenes/roundabout-saro/agent_routes/double_roundabout.xml \
    --ego_camera