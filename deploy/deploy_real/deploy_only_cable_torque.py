"""Standalone cable tension tracking debug (spool speed control only).

目的：把“绳子拉力跟踪”从足式机器人整体控制中拆出来，单独调参。

- 只使用：IMU + 拉力传感器 + yaw/pitch 传感器 + spool 速度电机
- 腿部 18 关节保持阻尼（不跑策略/不下发位置），避免互相耦合
- 你用手推机器人模拟运动：IMU 速度用于前馈，yaw 用于投影修正

运行示例：
  python deploy/deploy_real/deploy_only_cable.py \
	--config deploy/deploy_real/configs/hexapod_tethered.yaml \
	--tension-ref 80 \
	--plot
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import time
from dataclasses import dataclass
from datetime import datetime
from typing import Optional

import numpy as np

from imu_sdk_deta40.imu_sdk import IMUSDK


from hexapod_tethered_utils.tension_torque_controller import (
	TensionTorqueController,
	TensionTorqueControllerConfig,
)


from motor_igh_sdk.deploy_real_el4090_pysoem_spool_torque import (
	RL_Real_PySOEM_WithSpoolTorque,
)


from hexapod_tethered_utils.joystick_reader import Gamepad
from hexapod_tethered_utils.cable_tension_sensor import CableTensionSensor
from hexapod_tethered_utils.cable_arm_yaw_sensor import CableArmYawSensor

from common.command_helper_hexapod import create_damping_cmd, create_zero_velocity_cmd

import config_hexapod_tethered
from config_hexapod_tethered import Config


class RealTimePlotter:
	def __init__(
		self,
		history_seconds: float = 20.0,
		control_dt: float = 0.02,
		plot_every_n: int = 5,
		enabled: bool = True,
	) -> None:
		self.enabled = bool(enabled)
		self.control_dt = float(control_dt)
		self.plot_every_n = max(int(plot_every_n), 1)

		self._step = 0
		self._ok = False

		self._maxlen = max(int(history_seconds / max(self.control_dt, 1e-6)), 10)
		self._t = []
		self._tension_meas = []
		self._tension_ref = []
		self._yaw = []
		self._imu_vx = []
		self._spool_cmd_rpm = []
		self._spool_vel_rpm = []
		self._feedback_rpm = []
		self._rff_rpm = []

		if not self.enabled:
			return

		try:
			import matplotlib.pyplot as plt  # type: ignore

			self._plt = plt
			self._plt.ion()

			self._fig, self._axs = self._plt.subplots(
				5, 1, sharex=True, figsize=(10, 10), constrained_layout=True
			)
			self._fig.canvas.manager.set_window_title("Cable-only Debug")

			ax = self._axs[0]
			(self._ln_tension_meas,) = ax.plot([], [], label="tension_meas")
			(self._ln_tension_ref,) = ax.plot([], [], label="tension_ref")
			ax.set_ylabel("Tension")
			ax.grid(True)
			ax.legend(loc="upper right")

			ax = self._axs[1]
			(self._ln_yaw,) = ax.plot([], [], label="yaw_differ(rad)")
			ax.set_ylabel("Yaw")
			ax.grid(True)
			ax.legend(loc="upper right")

			ax = self._axs[2]
			(self._ln_imu_vx,) = ax.plot([], [], label="imu_vx(m/s)")
			ax.set_ylabel("IMU vx")
			ax.grid(True)
			ax.legend(loc="upper right")

			ax = self._axs[3]
			(self._ln_spool_cmd,) = ax.plot([], [], label="spool_cmd_rpm")
			(self._ln_spool_vel,) = ax.plot([], [], label="spool_vel_rpm")
			ax.set_ylabel("Spool")
			ax.set_xlabel("Time (s)")
			ax.grid(True)
			ax.legend(loc="upper right")

			ax = self._axs[4]
			(self._ln_feedback_rpm,) = ax.plot([], [], 'r-', label="feedback_rpm", linewidth=1.5)
			(self._ln_rff_rpm,) = ax.plot([], [], 'b--', label="rff_rpm", linewidth=1.5)
			ax.set_ylabel("RPM")
			ax.set_xlabel("Time (s)")
			ax.grid(True)
			ax.legend(loc="upper right")

			self._ok = True
		except Exception as e:
			self._ok = False
			self.enabled = False
			print(f"[Plot] disabled due to error: {e}")

	def update(
		self,
		t_s: float,
		tension_meas: float,
		tension_ref: float,
		yaw_differ_rad: float,
		imu_vx: float,
		spool_cmd_rpm: float,
		spool_vel_rpm: float,
		feedback_rpm: float,
		rff_rpm: float,

	) -> None:
		if not self.enabled or not self._ok:
			return

		self._step += 1
		self._t.append(float(t_s))
		self._tension_meas.append(float(tension_meas))
		self._tension_ref.append(float(tension_ref))
		self._yaw.append(float(yaw_differ_rad))
		self._imu_vx.append(float(imu_vx))
		self._spool_cmd_rpm.append(float(spool_cmd_rpm))
		self._spool_vel_rpm.append(float(spool_vel_rpm))
		self._feedback_rpm.append(float(feedback_rpm))
		self._rff_rpm.append(float(rff_rpm))

		if len(self._t) > self._maxlen:
			self._t = self._t[-self._maxlen :]
			self._tension_meas = self._tension_meas[-self._maxlen :]
			self._tension_ref = self._tension_ref[-self._maxlen :]
			self._yaw = self._yaw[-self._maxlen :]
			self._imu_vx = self._imu_vx[-self._maxlen :]
			self._spool_cmd_rpm = self._spool_cmd_rpm[-self._maxlen :]
			self._spool_vel_rpm = self._spool_vel_rpm[-self._maxlen :]
			self._feedback_rpm = self._feedback_rpm[-self._maxlen :]
			self._rff_rpm = self._rff_rpm[-self._maxlen :]

		if (self._step % self.plot_every_n) != 0:
			return

		t = np.asarray(self._t, dtype=np.float32)
		if t.size < 2:
			return

		self._ln_tension_meas.set_data(t, np.asarray(self._tension_meas, dtype=np.float32))
		self._ln_tension_ref.set_data(t, np.asarray(self._tension_ref, dtype=np.float32))
		self._ln_yaw.set_data(t, np.asarray(self._yaw, dtype=np.float32))
		self._ln_imu_vx.set_data(t, np.asarray(self._imu_vx, dtype=np.float32))
		self._ln_spool_cmd.set_data(t, np.asarray(self._spool_cmd_rpm, dtype=np.float32))
		self._ln_spool_vel.set_data(t, np.asarray(self._spool_vel_rpm, dtype=np.float32))
		self._ln_feedback_rpm.set_data(t, np.asarray(self._feedback_rpm, dtype=np.float32))
		self._ln_rff_rpm.set_data(t, np.asarray(self._rff_rpm, dtype=np.float32))

		for ax in self._axs:
			ax.relim()
			ax.autoscale_view(scalex=True, scaley=True)

		try:
			self._fig.canvas.draw_idle()
			self._fig.canvas.flush_events()
			self._plt.pause(0.001)
		except Exception:
			self.enabled = False
			print("[Plot] disabled (runtime GUI error)")


@dataclass
class RuntimeState:
	armed: bool = False
	last_tension: float = float("nan")
	last_yaw_differ: float = 0.0
	last_imu_vx: float = 0.0


def _safe_float(x: Optional[float], default: float) -> float:
	if x is None:
		return float(default)
	try:
		return float(x)
	except Exception:
		return float(default)




def build_argparser() -> argparse.ArgumentParser:
	p = argparse.ArgumentParser(description="Cable-only tension tracking debug")
	p.add_argument(
		"--config",
		type=str,
		default=f"{config_hexapod_tethered.ROOT_DIR}/deploy/deploy_real/configs/hexapod_tethered.yaml",
		help="YAML config path (uses speed_sign/k_p/ff/etc)",
	)
	p.add_argument("--ifname", type=str, default="enp86s0", help="EtherCAT NIC name")
	p.add_argument(
		"--tension-ref",
		type=float,
		default=0.0,
		help="Reference tension for tracking (same unit as tension sensor output)",
	)
	p.add_argument(
		"--spool-current-limit-01a",
		type=int,
		default=550,
		help="Spool motor current limit (0.1A unit)",
	)
	p.add_argument(
		"--plot",
		action="store_true",
		help="Enable best-effort matplotlib realtime plot",
	)
	p.add_argument(
		"--arm-with-gamepad",
		action="store_true",
		help="If gamepad exists, require A to arm and B to disarm",
	)
	return p


def main() -> int:
	args = build_argparser().parse_args()
	cfg = Config(args.config)

	print("[CableOnly] Starting...")
	print(f"[CableOnly] config: {args.config}")
	print(
		"[CableOnly] TSC params: "
		f"sign={cfg.tsc_speed_sign}, "
		f"kp_fwd={cfg.tsc_k_p_forward_rpm_per_unit}, "
		f"kp_bwd={cfg.tsc_k_p_backward_rpm_per_unit}, "
		f"ff_enabled={cfg.tsc_ff_enabled}, "
		f"radius={cfg.tsc_ff_radius_m}, "
		f"limit_rpm={cfg.tsc_speed_limit_rpm}"
	)

	# Optional gamepad (for arming / emergency disarm)
	gamepad = None
	if Gamepad is not None:
		try:
			gamepad = Gamepad()
		except Exception as e:
			gamepad = None
			print(f"[CableOnly] gamepad disabled: {e}")

	state = RuntimeState(armed=(not args.arm_with_gamepad))
	if args.arm_with_gamepad and gamepad is not None:
		print("[CableOnly] Gamepad present: press A to ARM, B to DISARM, LB to EXIT")
	else:
		print("[CableOnly] Control is ARMED by default; Ctrl+C to stop")

	# Logging
	timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
	log_dir = os.path.join(os.getcwd(), f"cable_only_logs_{timestamp}")
	os.makedirs(log_dir, exist_ok=True)
	log_path = os.path.join(log_dir, "cable_only.csv")
	log_f = open(log_path, "w", newline="")
	log_w = csv.writer(log_f)
	log_w.writerow(
		[
			"t_s",
			"armed",
			"tension_meas",
			"tension_ref",
			"yaw_differ_rad",
			"imu_vx",
			"spool_cmd_rpm",
			"spool_vel_rpm",
			"spool_pos_rad",
			"yaw_raw_deg",
		]
	)
	print(f"[CableOnly][Logging] {log_path}")

	plotter = RealTimePlotter(
		history_seconds=20.0,
		control_dt=float(cfg.control_dt),
		plot_every_n=5,
		enabled=bool(args.plot),
	)

	# Controllers
	tsc = TensionTorqueController(
		TensionTorqueControllerConfig(
			speed_sign=float(cfg.tsc_speed_sign),
			k_p_forward_rpm_per_unit=float(cfg.tsc_k_p_forward_rpm_per_unit_torque),
			k_p_backward_rpm_per_unit=float(cfg.tsc_k_p_backward_rpm_per_unit_torque),
			k_d_forward_rpm_per_unit=float(cfg.tsc_k_d_forward_rpm_per_unit_torque),
			k_d_backward_rpm_per_unit=float(cfg.tsc_k_d_backward_rpm_per_unit_torque),
			k_i_forward_rpm_per_unit=float(cfg.tsc_k_i_forward_rpm_per_unit_torque),
			k_i_backward_rpm_per_unit=float(cfg.tsc_k_i_backward_rpm_per_unit_torque),
			ff_enabled=bool(cfg.tsc_ff_enabled),
			ff_radius_m=float(cfg.tsc_ff_radius_m),
			tension_deadband=float(cfg.tsc_tension_deadband),
			tension_lpf_alpha=float(cfg.tsc_tension_lpf_alpha),
            Kff_forward=float(cfg.tsc_Kff_forward_torque),
            Kff_backward=float(cfg.tsc_Kff_backward_torque),
			torque_limit=float(cfg.tsc_torque_limit),
			torque_deadband=float(cfg.tsc_torque_deadband),
		)
	)

	# Hardware init
	robot = RL_Real_PySOEM_WithSpoolTorque(args.ifname)
	robot.spool_command_buffer.ctrl_status[0] = 1
	robot.spool_command_buffer.ack_status[0] = 1
	create_damping_cmd(robot)
	create_zero_velocity_cmd(robot)

	if not robot.start():
		print("[CableOnly][ERROR] EtherCAT start failed.")
		log_f.close()
		return 1

	imu = IMUSDK(port="/dev/ttyUSB_imu", baudrate=921600)
	imu_started = bool(imu.start())
	if not imu_started:
		print("[CableOnly][WARN] IMU start failed; feedforward will be 0")

	tension_sensor = CableTensionSensor(port="/dev/ttyUSB_cable_tension", baudrate=115200)
	tension_started = bool(tension_sensor.start())
	if not tension_started:
		print("[CableOnly][WARN] tension sensor start failed; spool will hold 0 RPM")

	yaw_sensor = CableArmYawSensor(port="/dev/ttyUSB_yaw", baudrate=115200)
	yaw_started = bool(yaw_sensor.start())
	if not yaw_started:
		print("[CableOnly][WARN] yaw sensor start failed; yaw will be treated as 0")

	t0 = time.time()
	next_tick = time.perf_counter()
	dt = float(cfg.control_dt)

	def _cleanup():
		try:
			robot.spool_command_buffer.target_torque_nm[0] = 0.0
			create_zero_velocity_cmd(robot)
			time.sleep(0.05)
		except Exception:
			pass
		try:
			robot.stop()
		except Exception:
			pass
		for dev in (tension_sensor, yaw_sensor):
			try:
				if hasattr(dev, "stop"):
					dev.stop()
			except Exception:
				pass
		try:
			if hasattr(imu, "stop"):
				imu.stop()
		except Exception:
			pass
		try:
			log_f.close()
		except Exception:
			pass

	print("[CableOnly] Loop running...")
	try:
		while True:
			# Gamepad buttons (best-effort)
			if gamepad is not None:
				if gamepad.consume_a_click():
					state.armed = True
					print("press A")
				if gamepad.consume_b_click():
					state.armed = False
				# Continuous LB to exit
				if int(gamepad.get_button_lb()) == 1:
					print("[CableOnly] LB pressed -> exit")
					break

			# Read sensors
			vel = imu.get_linear_velocity() if imu_started else None
			imu_vx = 0.0
			if vel is not None:
				try:
					imu_vx = float(vel[0])
				except Exception:
					imu_vx = 0.0
			state.last_imu_vx = imu_vx

			tension_meas = tension_sensor.get_cable_tension() if tension_started else None
			if tension_meas is not None:
				state.last_tension = float(tension_meas)

			yaw_raw_deg = yaw_sensor.get_angle() if yaw_started else None
			spool_pos_rad = float(robot.spool_state_buffer.position[0])
			yaw_differ = yaw_sensor.get_yaw_angle(
				motor_angle_deg=spool_pos_rad,
				yaw_value=yaw_raw_deg,
				offset_deg=float(cfg.offset_deg),
			)
			if yaw_differ is not None:
				state.last_yaw_differ = float(yaw_differ)
			yaw_differ_rad = float(state.last_yaw_differ)

			# Decide command
			t=time.time()
			tension_ref = float(args.tension_ref)
			spool_cmd_torque = 0.0
			feedback_torque = 0.0
			rff_torque = 0.0

			# Safety: if no valid tension measurement, hold 0 rpm
			if state.armed and (tension_meas is not None):
				spool_cmd_torque,feedback_torque, rff_torque= tsc.step(
					speed_input=float(0),
					#t=time.time(),
					#speed_input=float(imu_vx),
					yaw=float(yaw_differ_rad),
					#yaw=float(0),
					tension_ref=float(tension_ref),
					tension_meas=float(tension_meas),
				)
			else:
				spool_cmd_torque = 0.0
				feedback_torque = 0.0
				rff_torque = 0.0

			robot.spool_command_buffer.target_torque_nm[0] = float(spool_cmd_torque)
			spool_torque = float(robot.spool_state_buffer.torque[0])
			# print(imu_vx)
			# Log + plot
			t_s = time.time() - t0
			log_w.writerow(
				[
					f"{t_s:.6f}",
					int(state.armed),
					_safe_float(tension_meas, float("nan")),
					float(tension_ref),
					float(yaw_differ_rad),
					float(imu_vx),
					float(spool_cmd_torque),
					float(spool_torque),
					float(spool_pos_rad),
					_safe_float(yaw_raw_deg, float("nan")),
				]
			)

			plotter.update(
				t_s=float(t_s),
				tension_meas=_safe_float(tension_meas, 0.0),
				tension_ref=float(tension_ref),
				yaw_differ_rad=float(yaw_differ_rad),
				imu_vx=float(imu_vx),
				spool_cmd_torque=float(spool_cmd_torque),
				spool_torque=float(spool_torque),
				feedback_torque=float(feedback_torque), 
    			rff_torque=float(rff_torque), 
			)

			# Control timing
			next_tick += dt
			now = time.perf_counter()
			sleep_s = next_tick - now
			if sleep_s > 0:
				time.sleep(sleep_s)
			else:
				# If we overrun, resync (avoid drift).
				next_tick = now

	except KeyboardInterrupt:
		print("[CableOnly] KeyboardInterrupt -> exit")
	finally:
		_cleanup()
		print(f"[CableOnly] Logs saved to: {log_dir}")

	return 0


if __name__ == "__main__":
	raise SystemExit(main())

