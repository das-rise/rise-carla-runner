export SIMLINGO_CUSTOM_PROMPT="Do not overtake the blue vehicle in front of you. What should the ego do next?"
export SIMLINGO_USER_FLAG=1

. helpers/docker-quick.sh

carla_docker_restart

python src/rirun/run.py \
    15 \
    Town01 \
    --npc_trajectory_filepaths scenes/town01/npc_test_routes/vehicle1_straight.csv \
    --npc_movement pid \
    --timestep 0.0333333333333333333 \
    --ego_agent simlingo_rirun \
    --ego_route_filepath scenes/town01/agent_routes/agent_turn.xml \
    --record_cameras ego_overhead

unset SIMLINGO_CUSTOM_PROMPT
unset SIMLINGO_USER_FLAG

# Examples of additional run.py CLI flags that are currently unused here:
#   --output_dir output/turn_into_traffic
#   --display_camera stationary_overhead
#   --mapmatch
#   --offset_time 2.0
#   --seed 42
