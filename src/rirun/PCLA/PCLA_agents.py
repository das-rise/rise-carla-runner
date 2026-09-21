from enum import Enum


class PCLA_Agent(Enum):
    """
    Enum listing all PCLA autonomous driving agents currently available.
    Grouped by their source repositories or families.
    PCLA repo: https://github.com/MasoudJTehrani/PCLA
    """

    # SimLingo (CarLLaVA)
    SIMLINGO_SIMLINGO = "simlingo_simlingo"
    SIMLINGO_RIRUN = "simlingo_rirun"

    # LMDrive
    LMDRIVE_LLAVA = "lmdrive_llava"
    LMDRIVE_VICUNA = "lmdrive_vicuna"
    LMDRIVE_LLAMA = "lmdrive_llama"

    # Transfuser v3
    TFV3_TF = "tfv3_tf"
    TFV3_LTF = "tfv3_ltf"
    TFV3_LF = "tfv3_lf"
    TFV3_GF = "tfv3_gf"

    # Transfuser v4 (Transfuser++ checkpoints, 3 seeds for each variant)
    TFV4_L6_0 = "tfv4_l6_0"  # Best performing Transfuser v4 agent
    TFV4_L6_1 = "tfv4_l6_1"
    TFV4_L6_2 = "tfv4_l6_2"

    TFV4_LAV_0 = "tfv4_lav_0"  # Transfuser v4 not trained on Town02/05
    TFV4_LAV_1 = "tfv4_lav_1"
    TFV4_LAV_2 = "tfv4_lav_2"

    TFV4_WP_0 = "tfv4_wp_0"  # Transfuser v4 WP from appendix
    TFV4_WP_1 = "tfv4_wp_1"
    TFV4_WP_2 = "tfv4_wp_2"

    TFV4_AIM_0 = "tfv4_aim_0"  # Reproduction of AIM method
    TFV4_AIM_1 = "tfv4_aim_1"
    TFV4_AIM_2 = "tfv4_aim_2"

    # Transfuser v5
    TFV5_ALLTOWNS = "tfv5_alltowns"
    TFV5_NOTOWN13 = "tfv5_notown13"

    # Transfuser v6
    TFV6_REGNET = "tfv6_regnet"
    TFV6_RESNET = "tfv6_resnet"
    TFV6_4CAMERAS = "tfv6_4cameras"
    TFV6_NORADAR = "tfv6_noradar"
    TFV6_VISIONONLY = "tfv6_visiononly"
    TFV6_NOTOWN13 = "tfv6_notown13"

    # CaRL / Roach / PlanT (RL agents)
    CARL_CARL_0 = "carl_carl_0"
    CARL_CARL_1 = "carl_carl_1"
    CARL_CARLV11 = "carl_carlv11"
    CARL_ROACH_0 = "carl_roach_0"
    CARL_ROACH_1 = "carl_roach_1"
    CARL_ROACH_2 = "carl_roach_2"
    CARL_ROACH_3 = "carl_roach_3"
    CARL_ROACH_4 = "carl_roach_4"
    CARL_PLANT_0 = "carl_plant_0"
    CARL_PLANT_1 = "carl_plant_1"
    CARL_PLANT_2 = "carl_plant_2"
    CARL_PLANT_3 = "carl_plant_3"
    CARL_PLANT_4 = "carl_plant_4"

    # PlanT2
    PLANT2_PLANT2_0 = "plant2_plant2_0"
    PLANT2_PLANT2_1 = "plant2_plant2_1"
    PLANT2_PLANT2_2 = "plant2_plant2_2"

    # Tactile Transformer (TT)
    TT_TT = "tt_tt"

    # Orion
    ORION_BASE = "orion_base"

    # MindDrive (Qwen2)
    MINDDRIVE_05B = "minddrive_05b"
    MINDDRIVE_3B = "minddrive_3b"

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
    Check whether environment variables required by specific PCLA agents are set.

    Note: agent-specific environment variables that are part of the upstream
    PCLA setup (e.g. UNCERTAINTY_THRESHOLD / STOP_CONTROL / DIRECT for the
    Transfuser v4 family and ROUTES for if_if / lmdrive) are set automatically
    by pcla_functions.give_path at agent load time.

    Args:
        agent (PCLA_Agent): The PCLA agent to check environment variables for.
    """

    import os

    if agent == PCLA_Agent.SIMLINGO_RIRUN:
        if not os.environ.get("SIMLINGO_CUSTOM_PROMPT") or not os.environ.get("SIMLINGO_USER_FLAG"):
            raise RuntimeError(
                "SIMLINGO_CUSTOM_PROMPT=<<your_prompt>> and/or SIMLINGO_USER_FLAG=[0|1|2|3|4] environment variables not found for simlingo_rirun agent"
            )
