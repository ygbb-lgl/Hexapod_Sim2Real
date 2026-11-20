from pathlib import Path

import numpy as np
import yaml

ROOT_DIR = str(Path(__file__).resolve().parents[2])

LEG_ACTION_DIM = 18
ACTION_DIM = 19
OBSERVATION_DIM = 69


class Config:
    def __init__(self, file_path) -> None:
        with open(file_path, "r", encoding="utf-8") as f:
            config = yaml.load(f, Loader=yaml.FullLoader)

        self.control_dt = float(config["control_dt"])

        self.ang_vel = float(config["ang_vel_scale"])
        self.dof_pos = float(config["dof_pos_scale"])
        self.dof_vel = float(config["dof_vel_scale"])
        self.lin_vel = float(config["lin_vel_scale"])

        self.joint2motor_idx = config["joint2motor_idx"]
        self.policy_path = config["policy_path"].replace("{ROOT_DIR}", ROOT_DIR)
        self.default_angles = np.asarray(config["default_angles"], dtype=np.float32)
        self.command_scale = np.asarray(config["command_scale"], dtype=np.float32)
        self.action_scale = float(config["action_scale"])

        self.num_leggeds_actions = int(config["num_leggeds_actions"])
        self.num_actions = int(config["num_actions"])
        self.num_obs = int(config["num_obs"])

        estimate = config["estimate"]
        self.policy_device = str(estimate["policy_device"])

        # Legacy name retained because the yaw-sensor helper expects radians.
        self.offset_deg = float(config["offset_deg"])

        self.winch_radius_m = float(config["winch_radius_m"])
        self.winch_gear_ratio = float(config["winch_gear_ratio"])
        self.winch_initial_length_m = float(config["winch_initial_length_m"])
        self.winch_min_length_m = float(config["winch_min_length_m"])
        self.winch_max_length_m = float(config["winch_max_length_m"])
        self.winch_max_line_speed_mps = float(config["winch_max_line_speed_mps"])
        self.winch_response_time_s = float(config["winch_response_time_s"])
        self.winch_acceleration_limit_mps2 = float(
            config["winch_acceleration_limit_mps2"]
        )
        self.winch_payout_sign_from_motor = float(
            config["winch_payout_sign_from_motor"]
        )
        self.winch_max_motor_rpm = float(config["winch_max_motor_rpm"])

        self._validate()

    def _validate(self) -> None:
        if self.num_obs != OBSERVATION_DIM:
            raise ValueError(f"num_obs must be {OBSERVATION_DIM}, got {self.num_obs}")
        if self.num_actions != ACTION_DIM:
            raise ValueError(
                f"num_actions must be {ACTION_DIM}, got {self.num_actions}"
            )
        if self.num_leggeds_actions != LEG_ACTION_DIM:
            raise ValueError(
                "num_leggeds_actions must be "
                f"{LEG_ACTION_DIM}, got {self.num_leggeds_actions}"
            )
        if len(self.joint2motor_idx) != LEG_ACTION_DIM:
            raise ValueError("joint2motor_idx must contain 18 motor IDs")
        if self.default_angles.shape != (LEG_ACTION_DIM,):
            raise ValueError("default_angles must contain 18 values")
        if self.command_scale.shape != (3,):
            raise ValueError("command_scale must contain [vx, vy, wz]")
        if (
            not np.isfinite(self.default_angles).all()
            or not np.isfinite(self.command_scale).all()
        ):
            raise ValueError("default_angles and command_scale must be finite")

        positive = (
            self.control_dt,
            self.action_scale,
            self.winch_radius_m,
            self.winch_gear_ratio,
            self.winch_max_line_speed_mps,
            self.winch_response_time_s,
            self.winch_acceleration_limit_mps2,
            self.winch_max_motor_rpm,
        )
        if not all(np.isfinite(value) and value > 0.0 for value in positive):
            raise ValueError("control, joint and winch positive parameters are invalid")
        if not (
            0.0 < self.winch_min_length_m < self.winch_max_length_m
            and self.winch_min_length_m
            <= self.winch_initial_length_m
            <= self.winch_max_length_m
        ):
            raise ValueError("winch_initial_length_m must lie inside min/max length")
        if self.winch_payout_sign_from_motor not in (-1.0, 1.0):
            raise ValueError("winch_payout_sign_from_motor must be -1 or +1")
