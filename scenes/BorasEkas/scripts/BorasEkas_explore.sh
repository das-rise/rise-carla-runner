. helpers/docker-quick.sh

TODAY=$(date '+%F_%H-%M-%S')

carla_docker_restart

export SLOWER=0

python src/rirun/run.py \
    45 \
    scenes/BorasEkas/maps/BorasEkas_small_14_fixed_wider.xodr \
    --npc_trajectory_filepaths input_trajectories/BorasEkas/*Car.csv \
    --npc_movement teleport \
    --trajectory_statistics average_distance_true \
    --timestep 0.0333333333333333333 \
    --force_heading_interpolation \
    --heading_interpolation_mode spline \
    --offset_time 167 \
    --record_cameras stationary_overhead \
    --overhead_camera_position 0 0 150 \
    --use-dataprov \
    --output_dir output_ssd/boras-ekas__"$TODAY" \
    --ego_agent behavior_agent \
    --ego_behavior aggressive \
    --ego_route_filepath scenes/BorasEkas/agent_route.xml \
    --ego_spawn_time 170 \

