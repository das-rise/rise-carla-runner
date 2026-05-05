python src/rirun/run.py \
    60 \
    scenes/roundabout-saro/maps/saro_fixed_bidirectional_WIDER_LANES.xodr \
    --npc_movement teleport \
    --timestep 0.0333333333333333333 \
    --ego_agent neat_aim2dsem \
    --ego_route_filepath scenes/roundabout-saro/agent_routes/double_roundabout.xml \
    --record_cameras ego_overhead
