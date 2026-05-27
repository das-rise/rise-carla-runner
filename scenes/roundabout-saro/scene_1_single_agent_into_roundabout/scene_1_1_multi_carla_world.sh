. helpers/docker-quick.sh

TODAY=$(date '+%F_%H-%M-%S')

for i in {1013..1023}; do

  carla_docker_restart

  python src/rirun/run.py \
    45 \
    GbgSaroRound \
    --npc_trajectory_filepaths input_trajectories/saro_round_converted_carla_world/*Car.csv \
    --npc_movement behavior_agent \
    --npc_trajectory_offset 678710.40 6374600.42 \
    --on_npc_behavior_agent_route_done destroy \
    --mapmatch \
    --timestep 0.0333333333333333333 \
    --offset_time 1013 \
    --ego_agent tfpp_wp_1 \
    --ego_route_filepath scenes/roundabout-saro/agent_routes/GbgSaroRound.xml \
    --ego_spawn_time $i \
    --record_cameras stationary_overhead ego_dashcam \
    --overhead_camera_position 1505 -1145 100 \
    --use-dataprov \
    --output_dir output_ssd/saro_"$i"_3d__"$TODAY"

    echo "---"
    echo "---"
    echo "---"
    echo " Finished run with ego_spawn_time $i. Sleeping for 15 seconds before next run to ensure all files are written and avoid potential issues with Docker or file system."
    sleep 15

done
