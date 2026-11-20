"""19 号卷线电机/三轴力传感器/pitch 传感器标定采集脚本。

直接运行本文件即可，不需要命令行参数：

1. 所有设备就绪后，19 号电机立即施加 PRELOAD_TORQUE_NM 进行预紧；
2. 按手柄 A，将命令切换为 TARGET_TORQUE_NM；
3. 等待 STABILIZATION_TIME_S 后自动开始记录；
4. 按手柄 LB 随时将力矩置零并停止程序；
5. 程序结束时计算并打印平均值，同时追加到 summary.csv。

只需修改下方“用户配置”中的常量即可改变实验参数。
"""

import csv
import math
import os
import sys
import time
from datetime import datetime
from typing import Optional

from hexapod_tethered_utils.cable_end_pitch_sensor import CableEndPitchSensor
from hexapod_tethered_utils.cable_tension_sensor import CableTensionSensor
from hexapod_tethered_utils.joystick_reader import Gamepad
from motor_igh_sdk.deploy_real_el4090_pysoem_spool_torque import (
    RL_Real_PySOEM_WithSpoolTorque,
)


# =============================================================================
# 用户配置：每次标定前通常只需要修改 TARGET_TORQUE_NM
# =============================================================================

TARGET_TORQUE_NM = 4.0
PRELOAD_TORQUE_NM = 2.0

# 19 号卷线电机的力矩常数。驱动状态缓冲区返回实际电流（A），
# 实际力矩按项目现有约定由 电流 × 力矩常数 换算得到。
SPOOL_TORQUE_CONSTANT_NM_PER_A = 2.1

# 按 A 切换到目标力矩后，先等待机械系统稳定，再开始写 CSV。
STABILIZATION_TIME_S = 5.0
SAMPLE_RATE_HZ = 50.0

ETHERCAT_IFNAME = "enp86s0"
FORCE_SENSOR_PORT = "/dev/ttyUSB_cable_tension"
PITCH_SENSOR_PORT = "/dev/ttyUSB_pitch"
SENSOR_BAUDRATE = 115200

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
OUTPUT_ROOT = os.path.join(SCRIPT_DIR, "force_sensor_calibration_data")

# 等待传感器第一帧数据的最长时间。
SENSOR_READY_TIMEOUT_S = 10.0


RAW_COLUMNS = [
    "timestamp",
    "elapsed_s",
    "cmd_torque_nm",
    "actual_torque_nm",
    "force_x_n",
    "force_y_n",
    "force_z_n",
    "force_z_pressure_n",
    "force_norm_n",
    "pitch_rad",
    "pitch_deg",
    "sin_pitch",
    "cos_pitch",
]

SUMMARY_COLUMNS = [
    "run_finished_at",
    "target_torque_nm",
    "preload_torque_nm",
    "stabilization_time_s",
    "sample_rate_hz",
    "sample_count",
    "record_duration_s",
    "mean_cmd_torque_nm",
    "mean_actual_torque_nm",
    "mean_force_x_n",
    "mean_force_y_n",
    "mean_force_z_n",
    "mean_force_z_pressure_n",
    "mean_force_norm_n",
    "mean_pitch_rad",
    "mean_pitch_deg",
    "mean_sin_pitch",
    "mean_cos_pitch",
    "raw_csv",
]


def _torque_name(torque_nm: float) -> str:
    """生成适合目录/文件名的力矩文本，例如 4Nm、4p5Nm、-2Nm。"""
    value = f"{float(torque_nm):.6f}".rstrip("0").rstrip(".")
    if value == "-0":
        value = "0"
    return f"{value.replace('.', 'p')}Nm"


def _build_output_paths(target_torque_nm: float) -> tuple[str, str]:
    torque_name = _torque_name(target_torque_nm)
    torque_dir = os.path.join(OUTPUT_ROOT, torque_name)
    os.makedirs(torque_dir, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    raw_csv_path = os.path.join(torque_dir, f"{torque_name}_{timestamp}.csv")
    summary_csv_path = os.path.join(torque_dir, "summary.csv")
    return raw_csv_path, summary_csv_path


def _set_zero_commands(robot: RL_Real_PySOEM_WithSpoolTorque) -> None:
    """关闭 18 个腿部电机的反馈和前馈，只保留 19 号电机力矩命令。"""
    for i in range(robot.num_dofs):
        robot.motor_command_buffer.kp[i] = 0.0
        robot.motor_command_buffer.kd[i] = 0.0
        robot.motor_command_buffer.target_position[i] = 0.0
        robot.motor_command_buffer.target_velocity[i] = 0.0
        robot.motor_command_buffer.feedforward_torque[i] = 0.0


def _lb_pressed(gamepad: Gamepad) -> bool:
    return bool(gamepad.consume_lb_click() or gamepad.get_button_lb() == 1)


def _wait_for_sensor_data(
    force_sensor: CableTensionSensor,
    pitch_sensor: CableEndPitchSensor,
    gamepad: Gamepad,
    timeout_s: float,
) -> bool:
    deadline = time.monotonic() + float(timeout_s)
    while time.monotonic() < deadline:
        if _lb_pressed(gamepad):
            print("[安全停止] 等待传感器时检测到 LB。")
            return False
        if not gamepad.is_running:
            raise RuntimeError("手柄读取线程已停止，无法保证 LB 安全停机。")
        if force_sensor.get_force_xyz() is not None and pitch_sensor.get_angle() is not None:
            return True
        time.sleep(0.02)
    raise TimeoutError(f"等待传感器数据超过 {timeout_s:.1f} s。")


def _wait_for_a(gamepad: Gamepad) -> bool:
    print(
        f"预紧力矩已设为 {PRELOAD_TORQUE_NM:.3f} N·m。"
        f"确认装置安全后按 A 切换至 {TARGET_TORQUE_NM:.3f} N·m；按 LB 停止。"
    )
    while True:
        if _lb_pressed(gamepad):
            print("[安全停止] 检测到 LB。")
            return False
        if not gamepad.is_running:
            raise RuntimeError("手柄读取线程已停止，无法保证 LB 安全停机。")
        if gamepad.consume_a_click():
            return True
        time.sleep(0.01)


def _wait_until_stable(gamepad: Gamepad, duration_s: float) -> bool:
    start = time.monotonic()
    deadline = start + max(float(duration_s), 0.0)
    last_printed_second: Optional[int] = None

    while time.monotonic() < deadline:
        if _lb_pressed(gamepad):
            print("[安全停止] 稳定等待阶段检测到 LB。")
            return False
        if not gamepad.is_running:
            raise RuntimeError("手柄读取线程已停止，无法保证 LB 安全停机。")

        remaining_s = max(deadline - time.monotonic(), 0.0)
        displayed_second = int(math.ceil(remaining_s))
        if displayed_second != last_printed_second:
            print(f"等待稳定：还剩 {displayed_second} s")
            last_printed_second = displayed_second
        time.sleep(0.01)

    return True


def _append_summary(summary_path: str, summary: dict[str, object]) -> None:
    needs_header = not os.path.exists(summary_path) or os.path.getsize(summary_path) == 0
    with open(summary_path, "a", newline="", encoding="utf-8") as summary_file:
        writer = csv.DictWriter(summary_file, fieldnames=SUMMARY_COLUMNS)
        if needs_header:
            writer.writeheader()
        writer.writerow(summary)


def main() -> int:
    if SAMPLE_RATE_HZ <= 0.0:
        raise ValueError("SAMPLE_RATE_HZ 必须大于 0。")
    if STABILIZATION_TIME_S < 0.0:
        raise ValueError("STABILIZATION_TIME_S 不能小于 0。")

    raw_csv_path, summary_csv_path = _build_output_paths(TARGET_TORQUE_NM)
    print(f"本次原始数据将保存到：{raw_csv_path}")

    gamepad = None
    force_sensor = None
    pitch_sensor = None
    robot = None
    raw_file = None
    rows = 0
    recording_started_at = None
    recording_ended_at = None
    sums = {
        "cmd_torque_nm": 0.0,
        "actual_torque_nm": 0.0,
        "force_x_n": 0.0,
        "force_y_n": 0.0,
        "force_z_n": 0.0,
        "force_z_pressure_n": 0.0,
        "force_norm_n": 0.0,
        "pitch_rad": 0.0,
        "sin_pitch": 0.0,
        "cos_pitch": 0.0,
    }

    try:
        # 先确认手柄和两个传感器可用，再启动会输出力矩的 EtherCAT 电机。
        gamepad = Gamepad()
        if getattr(gamepad, "_joystick", None) is None:
            raise RuntimeError("未检测到手柄；为保证 LB 安全停机，拒绝启动电机。")

        force_sensor = CableTensionSensor(
            port=FORCE_SENSOR_PORT,
            baudrate=SENSOR_BAUDRATE,
        )
        if not force_sensor.start():
            raise RuntimeError("三轴力传感器启动失败。")

        pitch_sensor = CableEndPitchSensor(
            port=PITCH_SENSOR_PORT,
            baudrate=SENSOR_BAUDRATE,
        )
        if not pitch_sensor.start():
            raise RuntimeError("pitch 传感器启动失败。")

        robot = RL_Real_PySOEM_WithSpoolTorque(ETHERCAT_IFNAME)
        _set_zero_commands(robot)
        robot.spool_command_buffer.ctrl_status[0] = 1
        robot.spool_command_buffer.ack_status[0] = 1
        robot.spool_command_buffer.target_torque_nm[0] = float(PRELOAD_TORQUE_NM)

        if not robot.start():
            raise RuntimeError("EtherCAT 启动失败。")
        if not getattr(robot, "_spool_enabled", False):
            raise RuntimeError("未检测到 19 号卷线电机对应的 EtherCAT 从站。")

        if not _wait_for_sensor_data(
            force_sensor,
            pitch_sensor,
            gamepad,
            SENSOR_READY_TIMEOUT_S,
        ):
            return 0
        print("三轴力和 pitch 数据均已就绪。")

        if not _wait_for_a(gamepad):
            return 0

        robot.spool_command_buffer.target_torque_nm[0] = float(TARGET_TORQUE_NM)
        print(f"A 已按下：目标力矩设为 {TARGET_TORQUE_NM:.3f} N·m。")

        if not _wait_until_stable(gamepad, STABILIZATION_TIME_S):
            return 0

        raw_file = open(raw_csv_path, "w", newline="", encoding="utf-8")
        writer = csv.DictWriter(raw_file, fieldnames=RAW_COLUMNS)
        writer.writeheader()

        recording_started_at = time.monotonic()
        next_sample_at = recording_started_at
        period_s = 1.0 / float(SAMPLE_RATE_HZ)
        last_flush_at = recording_started_at
        print("系统稳定等待结束，开始记录。肉眼读取数显装置；完成后按 LB 停止。")

        while True:
            if _lb_pressed(gamepad):
                print("[安全停止] LB 已按下，立即将电机力矩置零并结束记录。")
                robot.spool_command_buffer.target_torque_nm[0] = 0.0
                break
            if not gamepad.is_running:
                raise RuntimeError("手柄读取线程已停止，立即结束实验。")
            if not force_sensor.is_running:
                raise RuntimeError("三轴力传感器读取线程已停止，立即结束实验。")
            if not pitch_sensor.is_running:
                raise RuntimeError("pitch 传感器读取线程已停止，立即结束实验。")

            force_xyz = force_sensor.get_force_xyz()
            pitch_rad = pitch_sensor.get_angle()
            if force_xyz is not None and pitch_rad is not None:
                fx, fy, fz = (float(value) for value in force_xyz)
                pitch_rad = float(pitch_rad)
                cmd_torque_nm = float(robot.spool_command_buffer.target_torque_nm[0])
                actual_torque_nm = (
                    SPOOL_TORQUE_CONSTANT_NM_PER_A
                    * float(robot.spool_state_buffer.torque[0])
                )
                force_z_pressure = abs(fz)
                force_norm = math.sqrt(fx * fx + fy * fy + fz * fz)
                sin_pitch = math.sin(pitch_rad)
                cos_pitch = math.cos(pitch_rad)
                elapsed_s = time.monotonic() - recording_started_at

                writer.writerow(
                    {
                        "timestamp": datetime.now().isoformat(timespec="milliseconds"),
                        "elapsed_s": f"{elapsed_s:.6f}",
                        "cmd_torque_nm": f"{cmd_torque_nm:.9f}",
                        "actual_torque_nm": f"{actual_torque_nm:.9f}",
                        "force_x_n": f"{fx:.9f}",
                        "force_y_n": f"{fy:.9f}",
                        "force_z_n": f"{fz:.9f}",
                        "force_z_pressure_n": f"{force_z_pressure:.9f}",
                        "force_norm_n": f"{force_norm:.9f}",
                        "pitch_rad": f"{pitch_rad:.9f}",
                        "pitch_deg": f"{math.degrees(pitch_rad):.9f}",
                        "sin_pitch": f"{sin_pitch:.9f}",
                        "cos_pitch": f"{cos_pitch:.9f}",
                    }
                )
                rows += 1
                sums["cmd_torque_nm"] += cmd_torque_nm
                sums["actual_torque_nm"] += actual_torque_nm
                sums["force_x_n"] += fx
                sums["force_y_n"] += fy
                sums["force_z_n"] += fz
                sums["force_z_pressure_n"] += force_z_pressure
                sums["force_norm_n"] += force_norm
                sums["pitch_rad"] += pitch_rad
                sums["sin_pitch"] += sin_pitch
                sums["cos_pitch"] += cos_pitch

            now = time.monotonic()
            if now - last_flush_at >= 1.0:
                raw_file.flush()
                last_flush_at = now

            next_sample_at += period_s
            sleep_s = next_sample_at - time.monotonic()
            if sleep_s > 0.0:
                time.sleep(sleep_s)
            else:
                # 避免一次过载后通过无休眠循环追赶旧的采样时刻。
                next_sample_at = time.monotonic()

        recording_ended_at = time.monotonic()

    except KeyboardInterrupt:
        print("\n检测到 Ctrl+C，停止实验。")
    except Exception as exc:
        print(f"[错误] {exc}")
        return 1
    finally:
        # 先清零命令并留出几个 EtherCAT 周期发送零力矩，然后再停止线程。
        if robot is not None:
            try:
                robot.spool_command_buffer.target_torque_nm[0] = 0.0
                time.sleep(0.10)
            except Exception:
                pass

        if raw_file is not None:
            raw_file.flush()
            raw_file.close()
        if force_sensor is not None:
            force_sensor.stop()
        if pitch_sensor is not None:
            pitch_sensor.stop()
        if gamepad is not None:
            gamepad.stop()
        if robot is not None:
            try:
                robot.stop()
            except Exception as exc:
                print(f"[警告] EtherCAT 停止时发生异常：{exc}")

        if rows > 0:
            if recording_ended_at is None:
                recording_ended_at = time.monotonic()
            duration_s = max(recording_ended_at - recording_started_at, 0.0)
            means = {key: value / rows for key, value in sums.items()}
            summary = {
                "run_finished_at": datetime.now().isoformat(timespec="seconds"),
                "target_torque_nm": f"{TARGET_TORQUE_NM:.9f}",
                "preload_torque_nm": f"{PRELOAD_TORQUE_NM:.9f}",
                "stabilization_time_s": f"{STABILIZATION_TIME_S:.6f}",
                "sample_rate_hz": f"{SAMPLE_RATE_HZ:.6f}",
                "sample_count": rows,
                "record_duration_s": f"{duration_s:.6f}",
                "mean_cmd_torque_nm": f"{means['cmd_torque_nm']:.9f}",
                "mean_actual_torque_nm": f"{means['actual_torque_nm']:.9f}",
                "mean_force_x_n": f"{means['force_x_n']:.9f}",
                "mean_force_y_n": f"{means['force_y_n']:.9f}",
                "mean_force_z_n": f"{means['force_z_n']:.9f}",
                "mean_force_z_pressure_n": f"{means['force_z_pressure_n']:.9f}",
                "mean_force_norm_n": f"{means['force_norm_n']:.9f}",
                "mean_pitch_rad": f"{means['pitch_rad']:.9f}",
                "mean_pitch_deg": f"{math.degrees(means['pitch_rad']):.9f}",
                "mean_sin_pitch": f"{means['sin_pitch']:.9f}",
                "mean_cos_pitch": f"{means['cos_pitch']:.9f}",
                "raw_csv": os.path.basename(raw_csv_path),
            }
            _append_summary(summary_csv_path, summary)

            print("\n本次标定统计：")
            print(f"  有效样本数：{rows}")
            print(f"  电机输入力矩平均值：{means['cmd_torque_nm']:.6f} N·m")
            print(f"  电机实际力矩平均值：{means['actual_torque_nm']:.6f} N·m")
            print(f"  Z 方向压力大小平均值：{means['force_z_pressure_n']:.6f} N")
            print(f"  Z 方向带符号平均值：{means['force_z_n']:.6f} N")
            print(f"  XYZ 合力平均值：{means['force_norm_n']:.6f} N")
            print(
                f"  pitch 平均值：{means['pitch_rad']:.6f} rad "
                f"({math.degrees(means['pitch_rad']):.6f} deg)"
            )
            print(f"  原始数据：{raw_csv_path}")
            print(f"  汇总数据：{summary_csv_path}")
        elif raw_file is not None or recording_started_at is not None:
            print("本次没有获得有效样本，因此未写入 summary.csv。")

    return 0


if __name__ == "__main__":
    sys.exit(main())
