. helpers/docker-quick.sh

TODAY=$(date '+%F_%H-%M-%S')

for i in {1013..1023}; do

  carla_docker_restart

  python src/rirun/run.py \
    45 \
    scenes/roundabout-saro/scene_1_single_agent_into_roundabout/saro_fixed_bidirectional_WIDER_LANES_modified.xodr \
    --npc_trajectory_filepaths input_trajectories/saro_roundabout_converted4/*Car.csv \
    --npc_movement teleport \
    --trajectory_statistics average_distance_true \
    --mapmatch \
    --timestep 0.0333333333333333333 \
    --offset_time 1013 \
    --ego_agent tfpp_wp_1 \
    --ego_route_filepath scenes/roundabout-saro/scene_1_single_agent_into_roundabout/scene_1_1_single_agent_into_roundabout.xml \
    --ego_spawn_time $i \
    --record_cameras stationary_overhead ego_dashcam \
    --overhead_camera_position 1651 -715 150 \
    --use-dataprov \
    --output_dir output_ssd/saro_"$i"__"$TODAY"

    echo "---"
    echo "---"
    echo "---"
    echo " Finished run with ego_spawn_time $i. Sleeping for 15 seconds before next run to ensure all files are written and avoid potential issues with Docker or file system."
    sleep 15

done
