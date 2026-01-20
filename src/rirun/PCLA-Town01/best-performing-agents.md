`SIMLINGO_RIRUN`:

```bash
python run.py 5 PCLA-Town01/Town01.xodr PCLA-Town01/vehicle1_route.csv --movement teleport --timestep 0.0333333333333333333 --pcla_agent simlingo_rirun --pcla_route PCLA-Town01/agent_route.xml --ego_camera
```

- slow simulation
- **promptable!** can be, within borders, prompted to behave differently. Currently only succesful for prompts relating to driving speed

---

`SIMLINGO_SIMLINGO`:

```bash
python run.py 5 PCLA-Town01/Town01.xodr PCLA-Town01/vehicle1_route.csv --movement teleport --timestep 0.0333333333333333333 --pcla_agent simlingo --pcla_route PCLA-Town01/agent_route.xml --ego_camera
```

- slow simulation
- medium-well lanekeeping

---

`TFPP_LAV_0`:

```bash
STOP_CONTROL=1 python run.py 5 PCLA-Town01/Town01.xodr PCLA-Town01/vehicle1_route.csv --movement teleport --timestep 0.0333333333333333333 --pcla_agent tfpp_lav_0 --pcla_route PCLA-Town01/agent_route.xml --ego_camera
```

- very fast simulation
- good lane-keeping

---

`TFPP_WP_0`:

```bash
DIRECT=0 python run.py 10 PCLA-Town01/Town01.xodr PCLA-Town01/vehicle1_route.csv --movement teleport --timestep 0.0333333333333333333 --pcla_agent tfpp_wp_0 --pcla_route PCLA-Town01/agent_route.xml --ego_camera
```

- fast simulation
- sticks to middle of road, not really to its lane

---

`NEAT_[NEAT/AIMBEV/AIM2DD/AIM2DDEPTH]`:

```bash
python run.py 10 PCLA-Town01/Town01.xodr PCLA-Town01/vehicle1_route.csv --movement teleport --timestep 0.0333333333333333333 --pcla_agent neat_[neat/aimbev/aim2dd/aim2ddepth] --pcla_route PCLA-Town01/agent_route.xml --ego_camera
```

- very fast
