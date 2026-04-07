. helpers/docker-quick.sh

carla_docker_restart

python src/rirun/run.py \
  45 \
  scenes/roundabout-saro/scene_1_single_agent_into_roundabout/saro_fixed_bidirectional_WIDER_LANES_modified.xodr \
  --trajectory-filepaths saro_roundabout_converted4/*Car.csv \
  --movement teleport \
  --timestep 0.0333333333333333333 \
  --offset_time 1013 \
  --pcla_agent tfpp_wp_1 \
  --pcla_route scenes/roundabout-saro/scene_1_single_agent_into_roundabout/scene_1_1_single_agent_into_roundabout.xml \
  --pcla_spawn_time -1020 \
  --camera_mode overhead \
  --overhead_camera_position 1651 -715 150