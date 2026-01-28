from agent_simlingo import LingoAgent, bicycle_model_forward, measurement_function_hx, measurement_mean, residual_measurement_h, residual_state_x, state_mean
from rirun_config_simlingo import RiRunGlobalConfig
import logging

import os
import pathlib
import time
from collections import deque
from pathlib import Path

import hydra
import numpy as np
import torch
import ujson
from filterpy.kalman import MerweScaledSigmaPoints
from filterpy.kalman import UnscentedKalmanFilter as UKF
from leaderboardcodes import autonomous_agent2
from omegaconf import OmegaConf
from transformers import AutoProcessor

import transfuser_utils as t_u
from scenario_logger import ScenarioLogger
from nav_planner import LateralPIDController

import os

DEBUG = False # saves images during evaluation
HD_VIZ = False
USE_UKF = True

# user_flag
# 0 : "<SAFETY> {speed_info} {targeting_prompt}"
# 1 : "<INSTRUCTION_FOLLOWING> {speed_info} {targeting_prompt} {custom_prompt}"
# 2 : "<INSTRUCTION_FOLLOWING> {speed_info} {custom_prompt}"
# 3 : "{speed_info} {custom_prompt}"
# 4 : "{speed_info} {targeting_prompt} {custom_prompt}"

try:
    CUSTOM_PROMPT = os.environ["SIMLINGO_CUSTOM_PROMPT"]
except KeyError:
    raise KeyError("SIMLINGO_CUSTOM_PROMPT environment variable not set")
USER_FLAG = 1

class RirunLingoAgent(LingoAgent):

    def setup(self, path_to_conf_file, route_index=None):
        """Sets up the agent. route_index is for logging purposes"""

        torch.cuda.empty_cache()
        self.track = autonomous_agent2.Track.SENSORS
        if "+" in path_to_conf_file:
            print(f"path to conf file: {path_to_conf_file}")
            self.config_path = path_to_conf_file.split("+")[0]
            print(f"Config path: {self.config_path}")
            self.save_path_root = path_to_conf_file.split("+")[1]
            print(f"Save path root: {self.save_path_root}")
        else:
            self.config_path = path_to_conf_file
            print(f"Config path: {self.config_path}")
            self.save_path_root = route_index
            print(f"Save path root: {self.save_path_root}")
        self.step = -1
        self.initialized = False
        self.device = torch.device("cuda")
        self.DrivingInput = {}
        self.config = RiRunGlobalConfig()

        if self.config.eval_route_as == -1:
            self.config.eval_route_as = self.model.route_as

        self.last_command = -1
        self.last_command_tmp = -1
        self.user_command = None
        self.user_flag = None
        self.running = True
        self.custom_prompt = None

        self.LMDRIVE_AUGM = False
        if self.LMDRIVE_AUGM:
            command_templates_file = f"data/augmented_templates/lmdrive.json"
            with open(command_templates_file, "r") as f:
                self.command_templates = ujson.load(f)

        # used for interactive eval of instruction following
        # thread = threading.Thread(target=self.input_thread)
        # thread.daemon = True  # This makes the thread exit when the main program exits
        # thread.start()

        self.route_path = os.environ.get("ROUTES", "")
        route_type = self.route_path.split("data/benchmarks/")[-1].split("/")[0]
        route_number = str(pathlib.Path(self.route_path).stem)

        # PID controller for turning - used in earlier versions of the agent
        # self.turn_controller = t_u.PIDController(k_p=self.config.turn_kp,
        #                                          k_i=self.config.turn_ki,
        #                                          k_d=self.config.turn_kd,
        #                                          n=self.config.turn_n)
        self.speed_controller = t_u.PIDController(
            k_p=self.config.speed_kp,
            k_i=self.config.speed_ki,
            k_d=self.config.speed_kd,
            n=self.config.speed_n,
        )

        self.turn_controller = LateralPIDController(inference_mode=False)

        image_fps = 5
        image_history_length = 1

        self.image_buffer = deque(maxlen=image_fps * image_history_length)

        # config
        self.carla_frame_rate = 1.0 / 20.0  # CARLA frame rate in milliseconds
        self.data_save_freq = 5
        self.lidar_seq_len = 1
        self.logging_freq = 10  # Log every 10 th frame
        self.logger_region_of_interest = (
            30.0  # Meters around the car that will be logged.
        )
        self.dense_route_planner_min_distance = 1.0
        self.dense_route_planner_max_distance = 50.0
        self.log_route_planner_min_distance = 4.0
        self.route_planner_max_distance = 50.0
        self.route_planner_min_distance = 7.5

        # load config from .hydra folder
        self.config_load_path = (
            Path(self.config_path).parent.parent.parent / ".hydra" / "config.yaml"
        )
        with open(self.config_load_path, "r") as file:
            cfg = OmegaConf.load(file)
        self.cfg = cfg
        self.cfg.model.vision_model.use_global_img = cfg.data_module.use_global_img

        processor = AutoProcessor.from_pretrained(
            cfg.model.vision_model.variant, trust_remote_code=True
        )
        if "tokenizer" in processor.__dict__:
            self.tokenizer = processor.tokenizer
        else:
            self.tokenizer = processor
        self.tokenizer.add_special_tokens(
            {
                "additional_special_tokens": [
                    "<WAYPOINTS>",
                    "<WAYPOINTS_DIFF>",
                    "<ORG_WAYPOINTS_DIFF>",
                    "<ORG_WAYPOINTS>",
                    "<WAYPOINT_LAST>",
                    "<ROUTE>",
                    "<ROUTE_DIFF>",
                    "<TARGET_POINT>",
                ]
            }
        )
        self.tokenizer.padding_side = "left"
        # llm_tokenizer = AutoTokenizer.from_pretrained(cfg.model.language_model.variant)
        cache_dir = f"pretrained/{(cfg.model.vision_model.variant.split('/')[1])}"
        default_dtype = torch.get_default_dtype()
        torch.set_default_dtype(torch.bfloat16)
        self.model = hydra.utils.instantiate(
            cfg.model,
            cfg_data_module=cfg.data_module,
            processor=processor,
            cache_dir=cache_dir,
            _recursive_=False,
        ).to(self.device)
        torch.set_default_dtype(default_dtype)
        self.model.load_state_dict(torch.load(self.config_path))
        self.iter = self.config_path.split("epoch=")[-1].split("/")[0]
        self.session = self.config_path.split("/")[-4]

        self.T = 1
        self.stuck_detector = 0
        self.force_move = 0

        self.commands = deque(maxlen=2)
        self.commands.append(4)
        self.commands.append(4)
        self.target_point_prev = [1e5, 1e5, 1e5]

        # Filtering
        if USE_UKF:
            self.points = MerweScaledSigmaPoints(
                n=4, alpha=0.00001, beta=2, kappa=0, subtract=residual_state_x
            )
            self.ukf = UKF(
                dim_x=4,
                dim_z=4,
                fx=bicycle_model_forward,
                hx=measurement_function_hx,
                dt=self.carla_frame_rate,
                points=self.points,
                x_mean_fn=state_mean,
                z_mean_fn=measurement_mean,
                residual_x=residual_state_x,
                residual_z=residual_measurement_h,
            )

            # State noise, same as measurement because we
            # initialize with the first measurement later
            self.ukf.P = np.diag([0.5, 0.5, 0.000001, 0.000001])
            # Measurement noise
            self.ukf.R = np.diag([0.5, 0.5, 0.000000000000001, 0.000000000000001])
            self.ukf.Q = np.diag([0.0001, 0.0001, 0.001, 0.001])  # Model noise
            # Used to set the filter state equal the first measurement
            self.filter_initialized = False
        # Stores the last filtered positions of the ego vehicle. Need at least 2 for LiDAR 10 Hz realignment
        self.state_log = deque(
            maxlen=max((self.lidar_seq_len * self.data_save_freq), 2)
        )

        # Path to where visualizations and other debug output gets stored
        if self.save_path_root is not None:
            self.save_path = os.environ.get("SAVE_PATH", "") + self.save_path_root
        else:
            self.save_path = None
        # self.checkpoint_path = os.environ.get('CHECKPOINT_ENDPOINT').

        # Logger that generates logs used for infraction replay in the results_parser.
        if self.save_path is not None and route_index is not None:
            self.save_path = pathlib.Path(self.save_path) / route_index
            pathlib.Path(self.save_path).mkdir(parents=True, exist_ok=True)

            self.lon_logger = ScenarioLogger(
                save_path=self.save_path,
                route_index=route_index,
                logging_freq=self.logging_freq,
                log_only=True,
                route_only=False,  # with vehicles
                roi=self.logger_region_of_interest,
            )

        if self.save_path is not None:
            self.debug_save_path = (
                self.save_path
                + "/debug_viz"
                + f'/{self.session}/iter_{self.iter}/{route_type}/{route_number}_{time.strftime("%Y_%m_%d_%H_%M_%S")}'
            )
            Path(self.debug_save_path).mkdir(parents=True, exist_ok=True)
            self.save_path_metric = self.debug_save_path + "/metric"
            Path(self.save_path_metric).mkdir(parents=True, exist_ok=True)

        if DEBUG:
            self.save_path_img = self.debug_save_path + "/images"
            Path(self.save_path_img).mkdir(parents=True, exist_ok=True)

    def tick(self, input_data):
        if hasattr(self, "_has_logged_custom_prompt") is False:
            self._has_logged_custom_prompt = False
        if hasattr(self, "_has_logged_full_prompt") is False:
            self._has_logged_full_prompt = False
        # user_flag
        # 0 : "<SAFETY> {speed_info} {targeting_prompt}"
        # 1 : "<INSTRUCTION_FOLLOWING> {speed_info} {targeting_prompt} {custom_prompt}"
        # 2 : "<INSTRUCTION_FOLLOWING> {speed_info} {custom_prompt}"
        # 3 : "{speed_info} {custom_prompt}"
        # 4 : "{speed_info} {targeting_prompt} {custom_prompt}"
        self.user_flag = USER_FLAG
        self.custom_prompt = CUSTOM_PROMPT

        if not self._has_logged_custom_prompt:
            logging.info(
                f"[SIMLINGO] Custom prompt set to: '{self.custom_prompt}'; user_flag: {self.user_flag}"
            )
            self._has_logged_custom_prompt = True

        if hasattr(self, "prompt") and self._has_logged_full_prompt is False:
            logging.info(f"[SIMLINGO] Full prompt: '{self.prompt}'")
            self._has_logged_full_prompt = True

        return super().tick(input_data)


def get_entry_point():
    return "RirunLingoAgent"
