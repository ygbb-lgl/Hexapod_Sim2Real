
"""验证 cable_arm yaw differ 计算是否正确。

功能：
- 速度模式驱动指定电机恒速转动（EtherCAT / pysoem）。
- 串口读取臂 yaw 编码器角度（Modbus）。
- 调用 `CableArmYawSensor.get_yaw_angle(motor_angle, yaw_deg, offset)` 计算 differ。

注意：
- 本脚本不修改任何现有库代码，只在此文件内做映射/配置。
- `get_yaw_angle()` 当前实现会把 yaw_value(度)转成弧度，但不会转换 motor_angle / offset。
  因此更符合的用法是：motor_angle 与 offset 直接以“弧度”传入，yaw_value 传入“度”。
"""

from __future__ import annotations

import argparse
import os
import sys
import time
import math
from typing import Optional, Tuple


def _add_sys_path():
	script_dir = os.path.dirname(os.path.abspath(__file__))
	deploy_real_dir = os.path.abspath(os.path.join(script_dir, ".."))
	for p in (script_dir, deploy_real_dir):
		if p not in sys.path:
			sys.path.insert(0, p)


_add_sys_path()


try:
	from motor_igh_sdk.deploy_real_el4090_speed_pysoem import RL_Real_Speed_PySOEM
except ImportError as e:  # pragma: no cover
	raise ImportError(
		"无法导入速度模式 EtherCAT 类：motor_igh_sdk.deploy_real_el4090_speed_pysoem。"
		"请确认从 deploy/deploy_real 目录运行，且 pysoem 已安装。"
	) from e


try:
	from cable_arm_yaw_sensor import CableArmYawSensor
except ImportError:
	from hexapod_tethered_utils.cable_arm_yaw_sensor import CableArmYawSensor


def _infer_slave_passage_hexapod(motor_id: int) -> Optional[Tuple[int, int]]:
	"""按三块板的固定接线表推断 (slave_idx, passage)。passage 为 1-based。"""
	motor_id = int(motor_id)
	motor_id_map = [
		[7, 8, 9, 1, 2, 3],
		[13, 17, 18, 16, 15, 14],
		[4, 5, 6, 10, 11, 12],
	]
	for slave_idx, motor_ids in enumerate(motor_id_map):
		for passage_idx, mid in enumerate(motor_ids):
			if int(mid) == motor_id:
				return int(slave_idx), int(passage_idx + 1)
	return None


def _try_load_direction_offset_from_yaml(yaml_path: str, motor_id: int) -> Optional[Tuple[float, float]]:
	"""从 hexapod_tethered.yaml 读取 motor_directions/motor_offsets，返回 (direction, offset)。"""
	try:
		import yaml  # type: ignore
	except Exception:
		return None

	if not os.path.exists(yaml_path):
		return None

	with open(yaml_path, "r", encoding="utf-8") as f:
		cfg = yaml.safe_load(f)

	joint2motor_idx = cfg.get("joint2motor_idx", None)
	motor_directions = cfg.get("motor_directions", None)
	motor_offsets = cfg.get("motor_offsets", None)
	if not (isinstance(joint2motor_idx, list) and isinstance(motor_directions, list) and isinstance(motor_offsets, list)):
		return None

	motor_id = int(motor_id)
	try:
		policy_idx = [int(x) for x in joint2motor_idx].index(motor_id)
	except ValueError:
		return None

	if policy_idx >= len(motor_directions) or policy_idx >= len(motor_offsets):
		return None

	direction = float(motor_directions[policy_idx])
	offset = float(motor_offsets[policy_idx])
	return direction, offset


def main() -> int:
	script_dir = os.path.dirname(os.path.abspath(__file__))
	default_yaml = os.path.abspath(os.path.join(script_dir, "..", "configs", "hexapod_tethered.yaml"))

	ap = argparse.ArgumentParser(description="验证 cable_arm yaw differ 计算")
	ap.add_argument("--ifname", type=str, default="enp86s0", help="EtherCAT 网卡名")
	ap.add_argument("--motor-id", type=int, required=True, default=19, help="要验证的电机 ID（1~18 或 19 等）")
	ap.add_argument("--rpm", type=float, default=10.0, help="恒速转动指令（rpm）")
	ap.add_argument("--current-limit-01a", type=int, default=500, help="电流限幅（0.1A 为单位，例如 500=50A）")

	ap.add_argument("--yaw-port", type=str, default="/dev/ttyUSB_yaw", help="Yaw 编码器串口")
	ap.add_argument("--yaw-baud", type=int, default=115200, help="Yaw 编码器串口波特率")
	ap.add_argument("--yaw-slave-id", type=int, default=1, help="Yaw 编码器 Modbus 从机号")
	ap.add_argument("--yaw-timeout", type=float, default=0.2, help="Yaw 串口超时")
	ap.add_argument("--yaw-poll-hz", type=float, default=50.0, help="Yaw 读取频率")

	ap.add_argument(
		"--offset",
		type=float,
		default=-2.2275,
		help="get_yaw_angle 的 offset 参数（建议直接填弧度，保持与 motor_angle 一致）",
	)

	ap.add_argument("--slave-idx", type=int, default=3, help="手动指定 EtherCAT slave（可选）")
	ap.add_argument("--passage", type=int, default=1, help="手动指定 passage=1~6（可选）")
	ap.add_argument("--config-yaml", type=str, default=default_yaml, help="用于读取 motor_directions/offsets 的 yaml")

	ap.add_argument("--print-hz", type=float, default=10.0, help="打印频率")
	args = ap.parse_args()

	motor_id = int(args.motor_id)

	# 1) 启动 yaw 传感器线程
	yaw_sensor = CableArmYawSensor(
		port=str(args.yaw_port),
		baudrate=int(args.yaw_baud),
		slave_id=int(args.yaw_slave_id),
		timeout=float(args.yaw_timeout),
		poll_hz=float(args.yaw_poll_hz),
	)
	if not yaw_sensor.start():
		return 1

	# 2) 启动 EtherCAT 速度模式
	robot = RL_Real_Speed_PySOEM(str(args.ifname), motor_id=motor_id)

	# 2.1) 仅在脚本内修正 EtherCAT 地址映射（不改库文件）
	if args.slave_idx is not None and args.passage is not None:
		robot.motor_ethercat_addr_[motor_id] = {"slave": int(args.slave_idx), "passage": int(args.passage)}
	else:
		inferred = _infer_slave_passage_hexapod(motor_id)
		if inferred is not None:
			s, p = inferred
			robot.motor_ethercat_addr_[motor_id] = {"slave": int(s), "passage": int(p)}

	# 2.2) 从 yaml 修正 direction/offset（若可用）
	loaded = _try_load_direction_offset_from_yaml(str(args.config_yaml), motor_id)
	if loaded is not None:
		direction, motor_offset = loaded
		robot.motor_direction_[motor_id] = float(direction)
		robot.motor_offset_[motor_id] = float(motor_offset)
		print(f"[INFO] motor_id={motor_id} direction={direction} offset(rad)={motor_offset} loaded from {args.config_yaml}")
	else:
		print(f"[WARNING] 未从 yaml 读取 direction/offset（将使用 direction=1, offset=0）：{args.config_yaml}")

	# 速度模式 buffer 初始化
	robot.motor_command_buffer.target_speed_rpm[0] = 0.0
	robot.motor_command_buffer.current_limit_01a[0] = int(args.current_limit_01a)
	robot.motor_command_buffer.ack_status[0] = 1

	if not robot.start():
		yaw_sensor.stop()
		return 2

	print("[INFO] Running... Ctrl+C to stop")
	print(
		"Columns: t(s) | cmd_rpm | motor_pos(rad/deg) | yaw_enc(deg) | differ(rad/deg) | motor_vel(rad/s) | cur(A?)"
	)

	t0 = time.time()
	last_print = 0.0
	print_period = 1.0 / float(args.print_hz) if args.print_hz and args.print_hz > 0 else 0.1

	try:
		while True:
			# 恒速
			robot.motor_command_buffer.target_speed_rpm[0] = float(args.rpm)

			t = time.time() - t0
			if t - last_print >= print_period:
				last_print = t

				yaw_deg = yaw_sensor.get_angle()
				# if yaw_deg is not None:
				# 	yaw_deg = (360.0 - float(yaw_deg)) % 360.0
				motor_pos_rad = float(robot.motor_state_buffer.position[0])
				motor_pos_rad = motor_pos_rad % (2 * math.pi)
				motor_vel = float(robot.motor_state_buffer.velocity[0])
				motor_cur = float(robot.motor_state_buffer.torque[0])

				differ = yaw_sensor.get_yaw_angle(motor_pos_rad, yaw_deg, float(args.offset))

				motor_pos_deg = motor_pos_rad * 180.0 / 3.141592653589793
				if differ is None:
					print(
						f"t={t:7.3f} cmd_rpm={args.rpm:8.2f} "
						f"motor={motor_pos_rad:+8.4f}rad({motor_pos_deg:+8.2f}deg) "
						f"yaw=None differ=None vel={motor_vel:+8.3f} cur={motor_cur:+8.3f}"
					)
				else:
					differ_deg = float(differ) * 180.0 / 3.141592653589793
					print(
						f"t={t:7.3f} cmd_rpm={args.rpm:8.2f} "
						f"motor={motor_pos_rad:+8.4f}rad({motor_pos_deg:+8.2f}deg) "
						f"yaw={float(yaw_deg):+8.3f}deg "
						f"differ={float(differ):+8.4f}rad({differ_deg:+8.2f}deg) "
						f"vel={motor_vel:+8.3f} cur={motor_cur:+8.3f}"
					)

			time.sleep(0.01)

	except KeyboardInterrupt:
		print("\n[INFO] Stopping...")
	finally:
		try:
			robot.motor_command_buffer.target_speed_rpm[0] = 0.0
			time.sleep(0.05)
		except Exception:
			pass

		try:
			robot.stop()
		finally:
			yaw_sensor.stop()

	return 0


if __name__ == "__main__":
	raise SystemExit(main())

