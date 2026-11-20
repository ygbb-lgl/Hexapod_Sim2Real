"""Deploy the 69-D MJLab Real-Cable policy on the tethered Hexapod.

The first 18 policy outputs are leg-position actions.  Output 18 is a signed
winch line-speed command in m/s: positive pays cable out and negative reels it
in.  It is never interpreted as a desired tension in this deployment.
"""

from __future__ import annotations

import csv
import math
import os
import sys
import time
from datetime import datetime
from pathlib import Path

import numpy as np
import torch

# This file lives one directory below all deployment packages.  Insert that
# parent before importing local modules so both of these work:
#   python real_cable_experiment/deploy_real_hexapod_tethered_speed.py
#   python /absolute/path/to/deploy_real_hexapod_tethered_speed.py
SCRIPT_DIR = Path(__file__).resolve().parent
DEPLOY_REAL_DIR = SCRIPT_DIR.parent
if str(DEPLOY_REAL_DIR) not in sys.path:
    sys.path.insert(0, str(DEPLOY_REAL_DIR))

import config_hexapod_tethered_real_cable
from common.command_helper_hexapod import (
    create_damping_cmd,
    create_zero_cmd,
    create_zero_velocity_cmd,
)
from config_hexapod_tethered_real_cable import (
    ACTION_DIM,
    LEG_ACTION_DIM,
    OBSERVATION_DIM,
    Config,
)
from hexapod_tethered_estimate_utils.observation import (
    tether_direction_from_yaw_pitch,
)
from hexapod_tethered_utils.cable_arm_yaw_sensor import CableArmYawSensor
from hexapod_tethered_utils.cable_end_pitch_sensor import CableEndPitchSensor
from hexapod_tethered_utils.cable_tension_sensor import CableTensionSensor
from hexapod_tethered_utils.joystick_reader import Gamepad
from imu_sdk_deta40.imu_sdk import IMUSDK
from motor_igh_sdk.deploy_real_el4090_pysoem_spool_speed import (
    RL_Real_PySOEM_WithSpoolSpeed,
)


class WinchSpeedCommandFilter:
    """Training-matched finite-response filter for desired payout speed."""

    def __init__(self, response_time: float, acceleration_limit: float) -> None:
        if response_time <= 0.0 or acceleration_limit <= 0.0:
            raise ValueError(
                "winch response time and acceleration limit must be positive"
            )
        self.response_time = float(response_time)
        self.acceleration_limit = float(acceleration_limit)
        self.speed = 0.0

    def reset(self) -> None:
        self.speed = 0.0

    def step(self, target_speed: float, dt: float) -> float:
        alpha = 1.0 - math.exp(-float(dt) / self.response_time)
        delta = (float(target_speed) - self.speed) * alpha
        max_delta = self.acceleration_limit * float(dt)
        self.speed += float(np.clip(delta, -max_delta, max_delta))
        return self.speed


class RealCableCsvLogger:
    """Write the complete deployment state into one CSV row per control step."""

    def __init__(
        self,
        directory: str,
        trajectory_id: str,
        control_dt: float,
        flush_every_n: int = 50,
    ) -> None:
        os.makedirs(directory, exist_ok=True)
        self.path = os.path.join(directory, f"{trajectory_id}.csv")
        self.trajectory_id = trajectory_id
        self.control_dt = float(control_dt)
        self.flush_every_n = max(int(flush_every_n), 1)
        self._rows = 0
        self._file = open(self.path, "w", newline="", buffering=1024 * 1024)
        sudo_uid = os.environ.get("SUDO_UID")
        sudo_gid = os.environ.get("SUDO_GID")
        if sudo_uid is not None and sudo_gid is not None:
            try:
                os.chown(directory, int(sudo_uid), int(sudo_gid))
                os.chown(self.path, int(sudo_uid), int(sudo_gid))
            except (OSError, ValueError) as exc:
                print(f"[RealCableCSV][WARN] cannot restore log ownership: {exc}")
        self._fieldnames = self._make_fieldnames()
        self._writer = csv.DictWriter(self._file, fieldnames=self._fieldnames)
        self._writer.writeheader()
        print(f"[RealCableCSV] logging enabled: {self.path}")

    @staticmethod
    def _names(prefix: str, size: int):
        return [f"{prefix}_{i}" for i in range(size)]

    @classmethod
    def _make_fieldnames(cls):
        scalar = [
            "timestamp_ns",
            "trajectory_id",
            "sample_index",
            "control_dt_s",
            "force_raw_n",
            "torque_actual_nm",
            "torque_command_prev_nm",
            "torque_command_issued_nm",
            "motor_position_rad",
            "motor_speed_command_rpm",
            "motor_speed_actual_rpm",
            "policy_line_speed_target_mps",
            "line_speed_command_mps",
            "line_speed_actual_mps",
            "winch_natural_length_m",
            "winch_encoder_valid",
            "imu_valid",
            "tension_sensor_valid",
            "saturation_flag",
            "emergency_flag",
            "data_valid_flag",
        ]
        vectors = []
        for name, size in (
            ("velocity_command", 3),
            ("policy_action", ACTION_DIM),
            ("joint_position", 18),
            ("joint_velocity", 18),
            ("body_gravity_vector", 3),
            ("body_angular_velocity", 3),
            ("body_linear_velocity", 3),
            ("body_linear_acceleration", 3),
            ("tether_direction_body", 3),
        ):
            vectors.extend(cls._names(name, size))
        return scalar + vectors

    @staticmethod
    def _put_vector(row, prefix: str, value, size: int) -> None:
        array = np.asarray(value, dtype=np.float64).reshape(-1)
        if array.size != size:
            raise ValueError(f"{prefix} must contain {size} values, got {array.size}")
        for i, item in enumerate(array):
            row[f"{prefix}_{i}"] = float(item)

    def write(
        self,
        *,
        timestamp_ns: int,
        force_raw_n: float,
        torque_actual_nm: float,
        torque_command_prev_nm: float,
        torque_command_issued_nm: float,
        motor_position_rad: float,
        motor_speed_command_rpm: float,
        motor_speed_actual_rpm: float,
        policy_line_speed_target_mps: float,
        line_speed_command_mps: float,
        line_speed_actual_mps: float,
        winch_natural_length_m: float,
        winch_encoder_valid: bool,
        velocity_command,
        policy_action,
        joint_position,
        joint_velocity,
        body_gravity_vector,
        body_angular_velocity,
        body_linear_velocity,
        body_linear_acceleration,
        tether_direction_body,
        imu_valid: bool,
        tension_sensor_valid: bool,
        saturation_flag: bool,
        emergency_flag: bool = False,
    ) -> None:
        row = {
            "timestamp_ns": int(timestamp_ns),
            "trajectory_id": self.trajectory_id,
            "sample_index": self._rows,
            "control_dt_s": self.control_dt,
            "force_raw_n": float(force_raw_n),
            "torque_actual_nm": float(torque_actual_nm),
            "torque_command_prev_nm": float(torque_command_prev_nm),
            "torque_command_issued_nm": float(torque_command_issued_nm),
            "motor_position_rad": float(motor_position_rad),
            "motor_speed_command_rpm": float(motor_speed_command_rpm),
            "motor_speed_actual_rpm": float(motor_speed_actual_rpm),
            "policy_line_speed_target_mps": float(policy_line_speed_target_mps),
            "line_speed_command_mps": float(line_speed_command_mps),
            "line_speed_actual_mps": float(line_speed_actual_mps),
            "winch_natural_length_m": float(winch_natural_length_m),
            "winch_encoder_valid": int(winch_encoder_valid),
            "imu_valid": int(imu_valid),
            "tension_sensor_valid": int(tension_sensor_valid),
            "saturation_flag": int(saturation_flag),
            "emergency_flag": int(emergency_flag),
            "data_valid_flag": int(
                imu_valid and tension_sensor_valid and winch_encoder_valid
            ),
        }
        self._put_vector(row, "velocity_command", velocity_command, 3)
        self._put_vector(row, "policy_action", policy_action, ACTION_DIM)
        self._put_vector(row, "joint_position", joint_position, 18)
        self._put_vector(row, "joint_velocity", joint_velocity, 18)
        self._put_vector(row, "body_gravity_vector", body_gravity_vector, 3)
        self._put_vector(row, "body_angular_velocity", body_angular_velocity, 3)
        self._put_vector(row, "body_linear_velocity", body_linear_velocity, 3)
        self._put_vector(row, "body_linear_acceleration", body_linear_acceleration, 3)
        self._put_vector(row, "tether_direction_body", tether_direction_body, 3)
        self._writer.writerow(row)
        self._rows += 1
        if self._rows % self.flush_every_n == 0:
            self._file.flush()

    def close(self) -> None:
        if self._file is not None:
            self._file.flush()
            self._file.close()
            self._file = None


class Controller:
    def __init__(self, config: Config) -> None:
        self.config = config
        if int(config.num_obs) != OBSERVATION_DIM:
            raise ValueError(
                f"Real-Cable policy requires num_obs={OBSERVATION_DIM}, "
                f"got {config.num_obs}"
            )
        if int(config.num_actions) != ACTION_DIM:
            raise ValueError(
                f"Real-Cable policy requires num_actions={ACTION_DIM}, "
                f"got {config.num_actions}"
            )

        # 手柄（pygame）用于生成 cmd；遥控器仍用于按钮状态机（start/A/select）
        self.gamepad = None
        if Gamepad is not None:
            try:
                self.gamepad = Gamepad()
            except Exception as e:
                self.gamepad = None
                print(f"[gamepad] disabled due to error: {e}")

        self.ang_vel_scale = config.ang_vel
        self.dof_pos_scale = config.dof_pos
        self.dof_vel_scale = config.dof_vel
        self.lin_vel_scale = config.lin_vel

        # Policy-side per-joint gains (indexed by policy_idx 0..17)
        self._kp_policy, self._kd_policy = self._build_policy_joint_gains()

        policy_path = Path(config.policy_path).expanduser().resolve()
        if not policy_path.is_file():
            raise FileNotFoundError(
                "Real-Cable TorchScript policy not found: "
                f"{policy_path}. Export/copy policy.pt or pass --policy."
            )
        try:
            self.policy = torch.jit.load(
                str(policy_path), map_location=self.config.policy_device
            )
        except Exception as exc:
            raise RuntimeError(
                f"Cannot load {policy_path} as a stateful TorchScript policy. "
                "A raw model_*.pt checkpoint is not deployable directly."
            ) from exc
        self.policy.eval()
        # Warm-up is followed by reset so artificial samples do not pollute GRU memory.
        self._warm_up()

        # 初始化状态和命令
        self.qj = np.zeros(config.num_leggeds_actions, dtype=np.float32)
        self.dqj = np.zeros(config.num_leggeds_actions, dtype=np.float32)
        self.spool_q = np.zeros(1, dtype=np.float32)
        self.action = np.zeros(config.num_actions, dtype=np.float32)
        self.target_dof_pos = config.default_angles.copy()
        self.obs = np.zeros(config.num_obs, dtype=np.float32)
        self.cmd = np.array([0, 0, 0], dtype=np.float32)
        self.counter = 0
        self._winch_encoder_origin_rad: float | None = None
        self._winch_natural_length_m = float(config.winch_initial_length_m)
        self._winch_actual_line_speed_mps = 0.0
        self._winch_command_filter = WinchSpeedCommandFilter(
            response_time=float(config.winch_response_time_s),
            acceleration_limit=float(config.winch_acceleration_limit_mps2),
        )

        # Keep each script and each process invocation in its own log directory.
        self.script_name = os.path.splitext(os.path.basename(__file__))[0]
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        script_dir = os.path.dirname(os.path.abspath(__file__))
        self.log_dir = os.path.join(
            script_dir,
            "data_logs",
            self.script_name,
            f"{self.script_name}_{timestamp}",
        )
        os.makedirs(self.log_dir, exist_ok=True)
        self.trajectory_id = f"{self.script_name}_trajectory_{timestamp}"
        self.real_cable_logger = RealCableCsvLogger(
            directory=self.log_dir,
            trajectory_id=self.trajectory_id,
            control_dt=float(self.config.control_dt),
            flush_every_n=50,
        )
        # Speed mode has no direct torque command; these columns stay at zero.
        self._previous_spool_torque_command_nm = 0.0

        # One EtherCAT master drives 18 joint motors plus one speed-mode winch.
        self.robot = RL_Real_PySOEM_WithSpoolSpeed("enp109s0")
        self.robot_start = self.robot.start()
        if not self.robot_start:
            raise RuntimeError("EtherCAT robot start failed; refusing to run policy")

        # imu init
        self.imu = IMUSDK(port="/dev/ttyUSB_imu", baudrate=921600)
        self.imu_started = self.imu.start()
        if not self.imu_started:
            print("[WARNING] IMU start failed. Will use zero data.")

        # 六位力传感器初始化 RS232
        self.tension_sensor = CableTensionSensor(
            port="/dev/ttyUSB_cable_tension", baudrate=115200
        )
        self.tension_started = self.tension_sensor.start()
        if not self.tension_started:
            print("[WARNING] Tension sensor start failed. Will use zero data.")

        # 末端绝对角度传感器初始化 RS485
        self.pitch_sensor = CableEndPitchSensor(
            port="/dev/ttyUSB_pitch", baudrate=115200
        )
        self.pitch_started = self.pitch_sensor.start()
        if not self.pitch_started:
            print("[WARNING] Pitch sensor start failed. Will use zero data.")

        # 磁栅编码器初始化 RS485
        self.yaw_sensor = CableArmYawSensor(port="/dev/ttyUSB_yaw", baudrate=115200)
        self.yaw_started = self.yaw_sensor.start()
        if not self.yaw_started:
            print("[WARNING] Yaw sensor start failed. Will use zero data.")

    def __del__(self):
        csv_logger = getattr(self, "real_cable_logger", None)
        if csv_logger is not None:
            try:
                csv_logger.close()
            except Exception:
                pass
        log_dir = getattr(self, "log_dir", None)
        if log_dir is not None:
            print(f"[Logging] CSV file saved to {log_dir}")

    def _build_policy_joint_gains(self):
        """Build kp/kd arrays aligned with policy joint indices.

        User-defined groups are specified in motor_id space, but the runtime
        buffers are indexed by policy_idx (0..17). We map via
        config.joint2motor_idx.
        """

        group1 = {13, 7, 1, 16, 10, 4}
        group2 = {18, 9, 3, 15, 11, 6}
        group3 = {17, 8, 2, 14, 12, 5}

        kp = np.zeros(18, dtype=np.float32)
        kd = np.zeros(18, dtype=np.float32)

        unknown_motor_ids = []
        for policy_idx in range(18):
            motor_id = int(self.config.joint2motor_idx[policy_idx])
            if motor_id in group1:
                kp[policy_idx] = 100.0
                kd[policy_idx] = 3.0
            elif motor_id in group2:
                kp[policy_idx] = 100.0
                kd[policy_idx] = 3.0
            elif motor_id in group3:
                kp[policy_idx] = 100.0
                kd[policy_idx] = 3.0
            else:
                unknown_motor_ids.append(motor_id)

        if unknown_motor_ids:
            # 不阻塞运行，但提示配置/分组可能不完整
            uniq = sorted(set(unknown_motor_ids))
            print(f"[WARNING] Some motor_ids are not in any gain group: {uniq}")

        return kp, kd

    def _warm_up(self):
        obs = torch.zeros(
            (1, OBSERVATION_DIM),
            dtype=torch.float32,
            device=self.config.policy_device,
        )
        with torch.inference_mode():
            for _ in range(10):
                output = self.policy(obs)
        if not isinstance(output, torch.Tensor) or tuple(output.shape) != (
            1,
            ACTION_DIM,
        ):
            shape = getattr(output, "shape", None)
            raise ValueError(
                f"Real-Cable policy must map (1, {OBSERVATION_DIM}) to "
                f"(1, {ACTION_DIM}), got {shape}"
            )
        reset = getattr(self.policy, "reset", None)
        if not callable(reset):
            raise ValueError(
                "Real-Cable deployment requires the stateful TorchScript export "
                "with a reset() method"
            )
        reset()
        print("Real-Cable policy warmed up; GRU state reset.")

    def _reset_policy_memory(self) -> None:
        self.policy.reset()
        self.action.fill(0.0)
        self.obs.fill(0.0)

    def _calibrate_winch_encoder(self) -> None:
        """Bind motor angle to a configured internal length estimate.

        This reference supports relative-travel logging and soft limits only;
        it is not an absolute rope-length measurement or an Actor observation.
        """
        if not bool(getattr(self.robot, "_spool_enabled", False)):
            raise RuntimeError(
                "cannot calibrate winch: EtherCAT spool channel is disabled"
            )
        motor_position = float(self.robot.spool_state_buffer.position[0])
        if not np.isfinite(motor_position):
            raise RuntimeError("cannot calibrate winch: motor position is not finite")
        self._winch_encoder_origin_rad = motor_position
        self._winch_natural_length_m = float(self.config.winch_initial_length_m)
        self._winch_actual_line_speed_mps = 0.0
        print(
            "[RealCable] internal winch travel reference set: "
            f"theta0={motor_position:+.6f} rad, "
            f"L0_est={self._winch_natural_length_m:.3f} m"
        )

    def _update_winch_encoder_state(self) -> bool:
        motor_position = float(self.robot.spool_state_buffer.position[0])
        motor_velocity = float(self.robot.spool_state_buffer.velocity[0])
        valid = bool(np.isfinite(motor_position) and np.isfinite(motor_velocity))
        if not valid:
            self._winch_actual_line_speed_mps = 0.0
            return False
        if self._winch_encoder_origin_rad is None:
            self._calibrate_winch_encoder()

        drum_delta = (motor_position - float(self._winch_encoder_origin_rad)) / float(
            self.config.winch_gear_ratio
        )
        length = (
            float(self.config.winch_initial_length_m)
            + float(self.config.winch_payout_sign_from_motor)
            * float(self.config.winch_radius_m)
            * drum_delta
        )
        self._winch_natural_length_m = float(
            np.clip(
                length,
                float(self.config.winch_min_length_m),
                float(self.config.winch_max_length_m),
            )
        )
        self._winch_actual_line_speed_mps = float(
            float(self.config.winch_payout_sign_from_motor)
            * float(self.config.winch_radius_m)
            * motor_velocity
            / float(self.config.winch_gear_ratio)
        )
        return True

    def _line_speed_to_motor_rpm(self, line_speed_mps: float) -> float:
        return float(
            float(self.config.winch_payout_sign_from_motor)
            * float(line_speed_mps)
            * float(self.config.winch_gear_ratio)
            / float(self.config.winch_radius_m)
            * 60.0
            / (2.0 * math.pi)
        )

    def _motor_rpm_to_line_speed(self, motor_rpm: float) -> float:
        return float(
            float(self.config.winch_payout_sign_from_motor)
            * float(motor_rpm)
            * (2.0 * math.pi / 60.0)
            * float(self.config.winch_radius_m)
            / float(self.config.winch_gear_ratio)
        )

    def prepare_policy_session(self) -> None:
        """Stop the drum, calibrate once, and clear policy/filter state."""
        create_zero_velocity_cmd(self.robot)
        if self._winch_encoder_origin_rad is None:
            self._calibrate_winch_encoder()
        self._update_winch_encoder_state()
        self._winch_command_filter.reset()
        self._reset_policy_memory()
        print(
            "[RealCable] policy ready: positive action=payout, "
            f"payout motor sign={self.config.winch_payout_sign_from_motor:+.0f}, "
            f"radius={self.config.winch_radius_m:.4f} m"
        )

    # 零力矩模式
    def zero_torque_state(self):
        print("Enter zero torque state.")
        print("Waiting for the Y signal to default pos...")
        if self.gamepad is None:
            raise RuntimeError(
                "Gamepad is not available; cannot enter zero_torque_state"
            )

        while self.gamepad.get_button_y() != 1:
            create_zero_cmd(self.robot)
            # 零速度
            create_zero_velocity_cmd(self.robot)
            time.sleep(self.config.control_dt)

    # 移动到默认位置
    def move_to_default_pos(self):
        print("Moving to default pos.")
        # Safety: ensure spool speed motor (id19) is stopped while repositioning joints.
        create_zero_velocity_cmd(self.robot)
        total_time = 2.0
        num_step = int(total_time / self.config.control_dt)
        default_pos = self.config.default_angles

        init_dof_pos = np.zeros(18, dtype=np.float32)
        # 读取当前 DOF 位置作为起点
        # NOTE: RL_Real_PySOEM buffers are indexed by policy_idx (0..17).
        # SDK internally maps policy_idx -> motor_id and applies direction/offset.
        for i in range(18):
            init_dof_pos[i] = self.robot.motor_state_buffer.position[i]

        for i in range(num_step):
            if self.gamepad.get_button_lb() == 1:
                return False

            alpha = i / num_step

            for j in range(18):
                target_pos = default_pos[j]
                q = init_dof_pos[j] * (1 - alpha) + target_pos * alpha
                self.robot.motor_command_buffer.kp[j] = 150.0
                self.robot.motor_command_buffer.kd[j] = 11.0
                self.robot.motor_command_buffer.target_position[j] = q
                self.robot.motor_command_buffer.target_velocity[j] = 0.0
                self.robot.motor_command_buffer.feedforward_torque[j] = 0.0

            # Keep stopping spool continuously (avoid stale speed cmd).
            create_zero_velocity_cmd(self.robot)

            time.sleep(self.config.control_dt)

        return True

    # 在默认位置等待，直到按下按钮A，或按下LB进入阻尼
    def default_pos_state(self):
        print("Enter default pos state.")
        print("Waiting for Button A to start policy, or LB for damping...")

        if self.gamepad is None:
            raise RuntimeError(
                "Gamepad is not available; cannot enter default_pos_state"
            )

        # init_dof_pos = np.zeros(18, dtype=np.float32)
        while True:
            if self.gamepad.get_button_lb() == 1:
                return False
            if self.gamepad.get_button_a() == 1:
                return True

            for i in range(18):
                default_pos = self.config.default_angles[i]
                self.robot.motor_command_buffer.kp[i] = 150.0
                self.robot.motor_command_buffer.kd[i] = 11.0
                self.robot.motor_command_buffer.target_position[i] = default_pos
                self.robot.motor_command_buffer.target_velocity[i] = 0.0
                self.robot.motor_command_buffer.feedforward_torque[i] = 0.0
                # init_dof_pos[i] = self.robot.motor_state_buffer.position[i]
                # print(init_dof_pos[i])

            # Safety: keep spool speed at 0 while holding default pose.
            create_zero_velocity_cmd(self.robot)
            time.sleep(self.config.control_dt)

    # 打印关节初始位置
    def print_initial_joint_positions(self):
        print("Initial joint positions:")
        for i in range(18):
            motor_id = self.config.joint2motor_idx[i]
            pos = self.robot.motor_state_buffer.position[i]
            print(f"policy_idx {i:02d} (motor_id {motor_id:02d}), Position: {pos:.3f}")

    # 打印imu数据
    def print_imu_data(self):
        vel = self.imu.get_linear_velocity()
        grav = self.imu.get_gravity_acceleration()
        if vel is not None and grav is not None:
            for i in range(10):
                print(
                    f"Vel: [{vel[0]:6.3f}, {vel[1]:6.3f}, {vel[2]:6.3f}] | Grav: [{grav[0]:6.3f}, {grav[1]:6.3f}, {grav[2]:6.3f}]"
                )
                time.sleep(0.1)
        else:
            print("No IMU data available.")

    def print_pitch_angle(self):
        pitch_value = self.pitch_sensor.get_angle()
        if pitch_value is not None:
            print(f"Pitch Angle: {pitch_value:.3f} rad")
        else:
            print("No pitch angle data available.")

    def print_yaw_angle(self):
        yaw_value = self.yaw_sensor.get_angle()
        tether_yaw = self.yaw_sensor.get_body_fixed_yaw(
            yaw_value=yaw_value,
            offset_rad=self.config.offset_deg,
        )
        if yaw_value is not None and tether_yaw is not None:
            print(f"Yaw Sensor: {float(yaw_value):.3f} deg")
            print(f"Policy Tether Yaw: {tether_yaw:.3f} rad")
        else:
            print("No yaw angle data available.")

    def run(self):
        if self.gamepad is None:
            raise RuntimeError("Gamepad is not available; cannot run control loop")

        observation_timestamp_ns = time.monotonic_ns()
        self.counter += 1
        for i in range(18):
            self.qj[i] = self.robot.motor_state_buffer.position[i]
            self.dqj[i] = self.robot.motor_state_buffer.velocity[i]

        self.spool_q[0] = self.robot.spool_state_buffer.position[0]
        spool_torque_actual_pre_action = float(self.robot.spool_state_buffer.torque[0])
        winch_encoder_valid = self._update_winch_encoder_state()
        if not winch_encoder_valid:
            create_zero_velocity_cmd(self.robot)
            raise RuntimeError("winch encoder state is invalid; winch stopped")
        # The SDK reports motor velocity in rad/s.
        spool_speed_actual_rpm = (
            float(self.robot.spool_state_buffer.velocity[0]) * 60.0 / (2.0 * math.pi)
        )
        vel = self.imu.get_linear_velocity()
        imu_data = self.imu.get_imu_data()
        grav = self.imu.get_gravity_acceleration()

        tension_value = self.tension_sensor.get_cable_tension()
        tension_sensor_valid = tension_value is not None and np.isfinite(tension_value)
        if not tension_sensor_valid:
            tension_value = 0.0

        pitch_radians = self.pitch_sensor.get_angle()
        if pitch_radians is None or not np.isfinite(pitch_radians):
            pitch_radians = 0.0

        yaw_value = self.yaw_sensor.get_angle()
        tether_yaw = self.yaw_sensor.get_body_fixed_yaw(
            yaw_value=yaw_value,
            offset_rad=self.config.offset_deg,
        )
        if tether_yaw is None or not np.isfinite(tether_yaw):
            tether_yaw = 0.0
        tether_direction_body = tether_direction_from_yaw_pitch(
            float(tether_yaw), float(pitch_radians)
        )

        # Linear velocity is for logging only.  The Real-Cable Actor's GRU
        # estimates it and the 69-D policy input intentionally excludes it.
        if vel is None:
            linvel = np.zeros(3, dtype=np.float32)
            velocity_valid = False
        else:
            linvel = np.asarray(vel, dtype=np.float32)
            velocity_valid = bool(linvel.shape == (3,) and np.all(np.isfinite(linvel)))
            if not velocity_valid:
                linvel = np.zeros(3, dtype=np.float32)

        if imu_data is None or grav is None:
            ang_vel = np.zeros(3, dtype=np.float32)
            gravity_orientation = np.array([0.0, 0.0, -1.0], dtype=np.float32)
            body_linear_acceleration = np.zeros(3, dtype=np.float32)
            imu_signal_valid = False
        else:
            ang_vel = np.asarray(
                [imu_data["gyro_x"], imu_data["gyro_y"], imu_data["gyro_z"]],
                dtype=np.float32,
            )
            gravity_orientation = np.asarray(grav, dtype=np.float32)
            body_linear_acceleration = np.asarray(
                [imu_data["acc_x"], imu_data["acc_y"], imu_data["acc_z"]],
                dtype=np.float32,
            )
            imu_signal_valid = bool(
                np.all(np.isfinite(ang_vel))
                and np.all(np.isfinite(gravity_orientation))
                and np.all(np.isfinite(body_linear_acceleration))
            )
            g_norm = float(np.linalg.norm(gravity_orientation))
            if g_norm > 1e-6:
                gravity_orientation = gravity_orientation / g_norm
            else:
                gravity_orientation = np.array([0.0, 0.0, -1.0], dtype=np.float32)
                imu_signal_valid = False
        imu_valid = bool(imu_signal_valid and velocity_valid)

        cmd = self.gamepad.get_command()
        self.cmd[0] = np.float32(np.clip(cmd[0], -1.0, 1.0))
        self.cmd[1] = np.float32(np.clip(cmd[1], -1.0, 1.0))
        self.cmd[2] = np.float32(np.clip(cmd[2], -1.0, 1.0))
        physical_command = self.cmd * np.asarray(
            self.config.command_scale, dtype=np.float32
        )

        qj_obs = self.qj - self.config.default_angles
        dqj_obs = self.dqj.copy()

        # Exact 69-D training order. Linear velocity is estimated by the GRU.
        self.obs[0:3] = ang_vel * self.ang_vel_scale
        self.obs[3:6] = gravity_orientation
        self.obs[6:24] = qj_obs * self.dof_pos_scale
        self.obs[24:42] = dqj_obs * self.dof_vel_scale
        self.obs[42:61] = self.action
        self.obs[61:64] = physical_command
        self.obs[64] = np.float32(max(float(tension_value), 0.0))
        self.obs[65:68] = tether_direction_body
        self.obs[68] = np.float32(self._winch_actual_line_speed_mps)

        obs_tensor = (
            torch.from_numpy(self.obs).unsqueeze(0).to(device=self.config.policy_device)
        )
        with torch.inference_mode():
            action_tensor = self.policy(obs_tensor)
        if not isinstance(action_tensor, torch.Tensor) or tuple(
            action_tensor.shape
        ) != (1, ACTION_DIM):
            raise RuntimeError(
                f"policy output must have shape (1, {ACTION_DIM}), "
                f"got {getattr(action_tensor, 'shape', None)}"
            )
        next_action = action_tensor.detach().cpu().numpy()[0].astype(np.float32)
        if not np.all(np.isfinite(next_action)):
            create_zero_velocity_cmd(self.robot)
            raise RuntimeError("policy produced NaN or infinity; winch stopped")
        self.action = next_action.copy()

        # Keep the raw output for the next last_action observation; preserve
        # this script's tighter joint safety limit for hardware commands.
        joint_action = np.clip(self.action[0:18], -2.0, 2.0)
        policy_line_speed_target = float(
            np.clip(
                self.action[LEG_ACTION_DIM],
                -float(self.config.winch_max_line_speed_mps),
                float(self.config.winch_max_line_speed_mps),
            )
        )

        target_dof_pos = (
            self.config.default_angles + joint_action * self.config.action_scale
        )

        for i in range(18):
            q = target_dof_pos[i]
            self.robot.motor_command_buffer.kp[i] = float(self._kp_policy[i])
            self.robot.motor_command_buffer.kd[i] = float(self._kd_policy[i])
            self.robot.motor_command_buffer.target_position[i] = q
            self.robot.motor_command_buffer.target_velocity[i] = 0.0
            self.robot.motor_command_buffer.feedforward_torque[i] = 0.0

        # The policy directly controls signed payout speed.  The only dynamics
        # added here are the same finite response and acceleration bound used
        # in training; there is no tension PD or velocity feedforward loop.
        at_min = (
            self._winch_natural_length_m <= float(self.config.winch_min_length_m) + 1e-6
        )
        at_max = (
            self._winch_natural_length_m >= float(self.config.winch_max_length_m) - 1e-6
        )
        length_limited_target = policy_line_speed_target
        if (at_min and length_limited_target < 0.0) or (
            at_max and length_limited_target > 0.0
        ):
            length_limited_target = 0.0
        filtered_line_speed = self._winch_command_filter.step(
            length_limited_target,
            float(self.config.control_dt),
        )
        if (at_min and filtered_line_speed < 0.0) or (
            at_max and filtered_line_speed > 0.0
        ):
            filtered_line_speed = 0.0
            self._winch_command_filter.speed = 0.0

        rpm_unclipped = self._line_speed_to_motor_rpm(filtered_line_speed)
        rpm_limit = abs(float(self.config.winch_max_motor_rpm))
        spool_speed_rpm = float(np.clip(rpm_unclipped, -rpm_limit, rpm_limit))
        issued_line_speed = self._motor_rpm_to_line_speed(spool_speed_rpm)
        # Keep the software model synchronized if the RPM safety bound clips it.
        self._winch_command_filter.speed = issued_line_speed
        self.robot.spool_command_buffer.target_speed_rpm[0] = float(spool_speed_rpm)
        saturation_flag = bool(
            abs(float(self.action[LEG_ACTION_DIM]))
            > float(self.config.winch_max_line_speed_mps) + 1e-6
            or abs(rpm_unclipped) > rpm_limit + 1e-6
            or length_limited_target != policy_line_speed_target
        )
        self.real_cable_logger.write(
            timestamp_ns=observation_timestamp_ns,
            force_raw_n=float(tension_value),
            torque_actual_nm=spool_torque_actual_pre_action,
            torque_command_prev_nm=0.0,
            torque_command_issued_nm=0.0,
            motor_position_rad=float(self.spool_q[0]),
            motor_speed_command_rpm=float(spool_speed_rpm),
            motor_speed_actual_rpm=float(spool_speed_actual_rpm),
            policy_line_speed_target_mps=policy_line_speed_target,
            line_speed_command_mps=issued_line_speed,
            line_speed_actual_mps=self._winch_actual_line_speed_mps,
            winch_natural_length_m=self._winch_natural_length_m,
            winch_encoder_valid=winch_encoder_valid,
            velocity_command=physical_command,
            policy_action=self.action,
            joint_position=self.qj,
            joint_velocity=self.dqj,
            body_gravity_vector=gravity_orientation,
            body_angular_velocity=ang_vel,
            body_linear_velocity=linvel,
            body_linear_acceleration=body_linear_acceleration,
            tether_direction_body=tether_direction_body,
            imu_valid=imu_valid,
            tension_sensor_valid=tension_sensor_valid,
            saturation_flag=saturation_flag,
            emergency_flag=False,
        )
        time.sleep(self.config.control_dt)


if __name__ == "__main__":
    config_path = (
        f"{config_hexapod_tethered_real_cable.ROOT_DIR}"
        "/deploy/deploy_real/configs/hexapod_tethered_real_cable.yaml"
    )
    config = Config(config_path)
    print(
        "[RealCable] 69-D observation, action[18]=signed payout speed (m/s)\n"
        f"[RealCable] policy={config.policy_path}\n"
        f"[RealCable] internal L0 estimate={config.winch_initial_length_m:.3f} m, "
        f"radius={config.winch_radius_m:.4f} m, "
        f"payout motor sign={config.winch_payout_sign_from_motor:+.0f}"
    )
    controller = Controller(config)

    time.sleep(1.0)
    controller.print_initial_joint_positions()
    controller.print_imu_data()
    controller.print_pitch_angle()
    controller.print_yaw_angle()

    controller.zero_torque_state()
    enter_policy = controller.move_to_default_pos()
    if enter_policy:
        enter_policy = controller.default_pos_state()

    if enter_policy:
        controller.prepare_policy_session()
        print("Start main control loop. Press LB to exit.")

    while enter_policy:
        try:
            if controller.gamepad.get_button_lb() == 1:
                break

            if controller.gamepad.get_button_y() == 1:
                create_zero_velocity_cmd(controller.robot)
                if not controller.move_to_default_pos():
                    break
                if not controller.default_pos_state():
                    break
                controller.prepare_policy_session()
                continue

            controller.run()
        except KeyboardInterrupt:
            break

    if getattr(controller, "gamepad", None) is not None:
        try:
            controller.gamepad.stop()
        except Exception:
            pass

    print(
        "Entering damping mode to lower the robot safely. Press Ctrl+C to exit completely."
    )
    try:
        while True:
            create_damping_cmd(controller.robot)
            create_zero_velocity_cmd(controller.robot)
            time.sleep(config.control_dt)
    except KeyboardInterrupt:
        pass

    print("Exit")
