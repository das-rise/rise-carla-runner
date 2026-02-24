python src/rirun/run.py \
    15 \
    Town01 \
    scenes/town01/npc_test_routes/vehicle1_straight.csv \
    --movement teleport \
    --timestep 0.0333333333333333333 \
    --pcla_agent neat_aim2dsem \
    --pcla_route scenes/town01/agent_routes/agent_turn.xml \
    --camera_mode ego