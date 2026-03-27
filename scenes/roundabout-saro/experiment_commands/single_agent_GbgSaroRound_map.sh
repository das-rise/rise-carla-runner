export DIRECT=0
python src/rirun/run.py \
  45 \
  GbgSaroRound \
  --movement teleport \
  --timestep 0.0333333333333333333 \
  --pcla_agent neat_aim2dsem \
  --pcla_route scenes/roundabout-saro/agent_routes/GbgSaroRound_map.xml \
  --camera_mode ego