from pathlib import Path

import numpy as np
import yaml
from hexapod_tethered_estimate_utils import ACTION_DIM, JOINT_DIM, OBSERVATION_DIM

ROOT_DIR = str(Path(__file__).resolve().parents[2])


class Config:
    def __init__(self, file_path) -> None:
        with open(file_path, "r") as f:
            config = yaml.load(f, Loader=yaml.FullLoader)

            self.control_dt = config["control_dt"]

            self.ang_vel = config["ang_vel_scale"]
            self.dof_pos = config["dof_pos_scale"]
            self.dof_vel = config["dof_vel_scale"]
            self.lin_vel = config["lin_vel_scale"]

            self.joint2motor_idx = config["joint2motor_idx"]

            self.policy_path = config["policy_path"].replace("{ROOT_DIR}", ROOT_DIR)

            self.pre_tension_model_path = config["pre_tension_model_path"].replace(
                "{ROOT_DIR}", ROOT_DIR
            )

            self.default_angles = np.array(config["default_angles"], dtype=np.float32)

            self.command_scale = config["command_scale"]
            self.action_scale = config["action_scale"]
            self.tension_action_scale = config["tension_action_scale"]

            self.num_leggeds_actions = config["num_leggeds_actions"]
            self.num_actions = config["num_actions"]
            self.num_obs = config["num_obs"] 

            estimate = config["estimate"]
            self.policy_device = str(estimate.get("policy_device", "cpu"))
            self.policy_action_clip = float(estimate["policy_action_clip"])
            self.cable_action_range = np.asarray(
                estimate["cable_action_range"], dtype=np.float32
            )

            if self.num_obs != OBSERVATION_DIM:
                raise ValueError(
                    f"num_obs must be {OBSERVATION_DIM}, got {self.num_obs}"
                )
            if self.num_actions != ACTION_DIM:
                raise ValueError(
                    f"num_actions must be {ACTION_DIM}, got {self.num_actions}"
                )
            if self.num_leggeds_actions != JOINT_DIM:
                raise ValueError(
                    "num_leggeds_actions must be "
                    f"{JOINT_DIM}, got {self.num_leggeds_actions}"
                )
            if (
                not np.isfinite(self.policy_action_clip)
                or self.policy_action_clip <= 0.0
            ):
                raise ValueError("estimate.policy_action_clip must be positive")
            if (
                self.cable_action_range.shape != (2,)
                or not np.isfinite(self.cable_action_range).all()
                or self.cable_action_range[0] >= self.cable_action_range[1]
            ):
                raise ValueError(
                    "estimate.cable_action_range must contain [minimum, maximum]"
                )

            self.offset_deg = config["offset_deg"]

            self.tsc_speed_sign = config["speed_sign"]
            # Backward compatible: if only k_p_tension_rpm_per_unit is provided, use it for both forward/backward.
            self.tsc_k_p_forward_rpm_per_unit = config["k_p_forward_rpm_per_unit"]
            self.tsc_k_p_backward_rpm_per_unit = config["k_p_backward_rpm_per_unit"]
            
            self.tsc_k_d_forward_rpm_per_unit = config["k_d_forward_rpm_per_unit"]
            self.tsc_k_d_backward_rpm_per_unit = config["k_d_backward_rpm_per_unit"]

            self.tsc_k_i_forward_rpm_per_unit = config["k_i_forward_rpm_per_unit"]
            self.tsc_k_i_backward_rpm_per_unit = config["k_i_backward_rpm_per_unit"]
            # Feedforward constants (paper-style)
            self.tsc_ff_enabled = config["ff_enabled"]
            self.tsc_ff_max_rpm = config["ff_max_rpm"]
            self.tsc_ff_max_speed_mps = config["ff_max_speed_mps"]
            self.tsc_ff_radius_m = config["ff_radius_m"]

            self.tsc_Kff_forward = config["Kff_forward"]
            self.tsc_Kff_backward = config["Kff_backward"]

            self.tsc_tension_deadband = config["tension_deadband"]
            self.tsc_speed_deadband_rpm = config["speed_deadband_rpm"]
            self.tsc_tension_lpf_alpha = config["tension_lpf_alpha"]
            self.tsc_speed_limit_rpm = config["speed_limit_rpm"]


            # For torque controller:
            self.tsc_k_p_forward_rpm_per_unit_torque = config["k_p_forward_rpm_per_unit_torque"]
            self.tsc_k_p_backward_rpm_per_unit_torque = config["k_p_backward_rpm_per_unit_torque"]

            self.tsc_k_d_forward_rpm_per_unit_torque = config["k_d_forward_rpm_per_unit_torque"]
            self.tsc_k_d_backward_rpm_per_unit_torque = config["k_d_backward_rpm_per_unit_torque"]

            self.tsc_k_i_forward_rpm_per_unit_torque = config["k_i_forward_rpm_per_unit_torque"]
            self.tsc_k_i_backward_rpm_per_unit_torque = config["k_i_backward_rpm_per_unit_torque"]           

            self.tsc_Kff_forward_torque = config["Kff_forward_torque"]
            self.tsc_Kff_backward_torque = config["Kff_backward_torque"]

            self.tsc_torque_limit = config["torque_limit"]
            self.tsc_torque_deadband = config["torque_deadband"]
