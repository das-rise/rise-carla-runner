from enum import Enum


class PCLA_Agent(Enum):
    """
    Enum listing all PCLA autonomous driving agents currently available.
    Grouped by their source repositories or families.
    PCLA repo: https://github.com/MasoudJTehrani/PCLA
    """

    # SimLingo (CarLLava)
    SIMLINGO_SIMLINGO = "simlingo_simlingo"
    SIMLINGO_RIRUN = "simlingo_rirun"

    # Transfuser++ (3 seeds for each agent)
    TFPP_L6_0 = "tfpp_l6_0"  # Best performing Transfuser++ agent
    TFPP_L6_1 = "tfpp_l6_1"
    TFPP_L6_2 = "tfpp_l6_2"

    TFPP_LAV_0 = "tfpp_lav_0"  # Transfuser++ not trained on Town02/05
    TFPP_LAV_1 = "tfpp_lav_1"
    TFPP_LAV_2 = "tfpp_lav_2"

    TFPP_WP_0 = "tfpp_wp_0"  # Transfuser++ WP from appendix
    TFPP_WP_1 = "tfpp_wp_1"
    TFPP_WP_2 = "tfpp_wp_2"

    TFPP_AIM_0 = "tfpp_aim_0"  # Reproduction of AIM method
    TFPP_AIM_1 = "tfpp_aim_1"
    TFPP_AIM_2 = "tfpp_aim_2"

    # Learning from All Vehicles (LAV)
    LAV_LAV = "lav_lav"  # Original LAV agent
    LAV_FAST = "lav_fast"  # Optimized leaderboard submission

    # Learning By Cheating (LBC)
    LBC_NC = "lbc_nc"  # NoCrash model
    LBC_LB = "lbc_lb"  # Leaderboard model

    # World on Rails (WoR)
    WOR_NC = "wor_nc"  # NoCrash model
    WOR_LB = "wor_lb"  # Leaderboard model

    # NEAT
    NEAT_NEAT = "neat_neat"
    NEAT_AIMBEV = "neat_aimbev"
    NEAT_AIM2DSEM = "neat_aim2dsem"
    NEAT_AIM2DDEPTH = "neat_aim2ddepth"

    # Interfuser
    IF_IF = "if_if"  # Second best performing CARLA Leaderboard 1 agent


def check_agent_env(agent: PCLA_Agent) -> None:
    """
    Chheck whether environment variables required by specific PCLA agents are set.

    Args:
        agent (PCLA_Agent): The PCLA agent to check environment variables for.
    """

    import os

    if agent in [PCLA_Agent.TFPP_L6_0, PCLA_Agent.TFPP_L6_1, PCLA_Agent.TFPP_L6_2]:
        if not os.environ.get("UNCERTAINTY_THRESHOLD"):
            raise RuntimeError(
                "UNCERTAINTY_THRESHOLD=0.33 environment variable not found for tfpp_l6 agents"
            )
    elif agent in [PCLA_Agent.TFPP_LAV_0, PCLA_Agent.TFPP_LAV_1, PCLA_Agent.TFPP_LAV_2]:
        if not os.environ.get("STOP_CONTROL"):
            raise RuntimeError(
                "STOP_CONTROL=1 environment variable not found for tfpp_lav agents"
            )
    elif agent in [PCLA_Agent.TFPP_AIM_0, PCLA_Agent.TFPP_AIM_1, PCLA_Agent.TFPP_AIM_2]:
        if not os.environ.get("DIRECT"):
            raise RuntimeError(
                "DIRECT=0 environment variable not found for tfpp_aim agents"
            )
    elif agent in [PCLA_Agent.TFPP_WP_0, PCLA_Agent.TFPP_WP_1, PCLA_Agent.TFPP_WP_2]:
        if not os.environ.get("DIRECT"):
            raise RuntimeError(
                "DIRECT=0 environment variable not found for tfpp_wp agents"
            )
    elif agent == PCLA_Agent.IF_IF:
        if not os.environ.get("ROUTES"):
            raise RuntimeError(
                "ROUTES=<<path_to_agent_route.xml>> environment variable not found for if_if agent"
            )
    elif agent == PCLA_Agent.SIMLINGO_RIRUN:
        if not os.environ.get("SIMLINGO_CUSTOM_PROMPT") or not os.environ.get("SIMLINGO_USER_FLAG"):
            raise RuntimeError(
                "SIMLINGO_CUSTOM_PROMPT=<<your_prompt>> and/or SIMLINGO_USER_FLAG=[0|1|2|3|4] environment variables not found for simlingo_rirun agent"
            )
