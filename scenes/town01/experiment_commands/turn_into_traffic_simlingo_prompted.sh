export SIMLINGO_CUSTOM_PROMPT="Do not overtake the blue vehicle in front of you. What should the ego do next?"
export SIMLINGO_USER_FLAG=1
python src/rirun/run.py \
    15 \
    Town01 \
    scenes/town01/npc_test_routes/vehicle1_straight.csv \
    --movement teleport \
    --timestep 0.0333333333333333333 \
    --pcla_agent simlingo_rirun \
    --pcla_route scenes/town01/agent_routes/agent_turn.xml \
    --ego_camera

unset SIMLINGO_CUSTOM_PROMPT
unset SIMLINGO_USER_FLAG