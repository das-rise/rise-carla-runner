python src/rirun/run.py \
    100 \
    scenes/roundabout-saro/maps/saro_fixed_bidirectional_WIDER_LANES.xodr \
    --movement teleport \
    --timestep 0.0333333333333333333 \
    --pcla_agent neat_aim2dsem \
    --pcla_route scenes/roundabout-saro/agent_routes/double_roundabout.xml \
    --camera_mode ego