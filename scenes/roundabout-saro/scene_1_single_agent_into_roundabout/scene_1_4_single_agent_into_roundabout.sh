. helpers/docker-quick.sh

carla_docker_restart

python src/rirun/run.py \
  100 \
  scenes/roundabout-saro/scene_1_single_agent_into_roundabout/SaroRound_2laneU_2026-04-21_trajectory-matched.xodr \
  --trajectory-filepaths input_trajectories/saro_round_converted_2026-04-21_twolane/*csv \
  --movement teleport \
  --mapmatch \
  --force_heading_interpolation \
  --heading_interpolation_mode spline \
  --timestep 0.0333333333333333333 \
  --offset_time 0 \
  --camera_mode overhead \
  --overhead_camera_position 0 0 150 \
  --use-dataprov 
  # --pcla_agent tfpp_wp_1 \
  # --pcla_route scenes/roundabout-saro/scene_1_single_agent_into_roundabout/scene_1_3_single_agent_into_roundabout.xml \
  # --pcla_spawn_time 0 \

