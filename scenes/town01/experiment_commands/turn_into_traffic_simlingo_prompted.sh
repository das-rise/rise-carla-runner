export SIMLINGO_CUSTOM_PROMPT="When there is a vehicle in front of you, overtake it by passing it on the left side."
python run.py \
    15 \
    Town01 \
    ../../scenes/town01/npc_test_routes/vehicle1_straight.csv \
    --movement teleport \
    --timestep 0.0333333333333333333 \
    --pcla_agent simlingo_rirun \
    --pcla_route ../../scenes/town01/agent_routes/agent_turn.xml \
    --ego_camera

unset SIMLINGO_CUSTOM_PROMPT