. helpers/docker-quick.sh

carla_docker_restart

python src/rirun/run.py \
  45 \
  scenes/roundabout-saro/scene_1_single_agent_into_roundabout/saro_fixed_bidirectional_WIDER_LANES_modified.xodr \
  --npc_trajectory_filepaths saro_roundabout_converted4/*Car.csv \
  --npc_movement teleport \
  --timestep 0.0333333333333333333 \
  --offset_time 1013 \
  --ego_agent tfpp_wp_1 \
  --ego_route_filepath scenes/roundabout-saro/scene_1_single_agent_into_roundabout/scene_1_1_single_agent_into_roundabout.xml \
  --ego_spawn_time 1020 \
  --record_cameras stationary_overhead \
  --overhead_camera_position 1651 -715 150 \
  --use-dataprov
