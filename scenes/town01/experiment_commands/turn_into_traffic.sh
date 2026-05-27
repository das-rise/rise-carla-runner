. helpers/docker-quick.sh

carla_docker_restart

python src/rirun/run.py \
    15 \
    Town01 \
    --npc_trajectory_filepaths scenes/town01/npc_test_routes/vehicle1_straight.csv \
    --npc_movement teleport \
    --timestep 0.0333333333333333333 \
    --ego_agent neat_aim2dsem \
    --ego_route_filepath scenes/town01/agent_routes/agent_turn.xml \
    --record_cameras ego_overhead

# Examples of additional run.py CLI flags that are currently unused here:
#   --output_dir output/turn_into_traffic
#   --display_camera stationary_overhead
#   --mapmatch
#   --offset_time 2.0
#   --seed 42