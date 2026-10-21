#!/usr/bin/env python3
"""Key-driven DETA40 motion test with CSV recording and JSON summaries.

This is an observational test.  It does not change the SDK estimator or its
thresholds.  With no external position/velocity reference, the results can
validate axis/sign conventions, short-term integrated-velocity response, and
ZUPT at rest; they cannot establish absolute velocity accuracy.
"""

import argparse
import csv
import json
import math
import os
from pathlib import Path
import select
import sys
import termios
import time
import tty
from typing import Any, Dict, Iterable, List, NamedTuple, Optional, Sequence, Tuple

try:
    from .imu_sdk import IMUSDK
except ImportError:
    from imu_sdk import IMUSDK  # type: ignore[no-redef]


SCRIPT_VERSION = "1.0"
AXES = ("x", "y", "z")
HEALTH_COUNTERS = (
    "serial_error_count",
    "parse_error_count",
    "frame_error_count",
    "crc_error_count",
    "header_crc_error_count",
    "payload_crc_error_count",
    "discarded_byte_count",
    "imu_gap_over_timeout_count",
    "ahrs_gap_over_timeout_count",
    "velocity_dt_reject_count",
    "timestamp_stall_count",
)


class Stage(NamedTuple):
    name: str
    title: str
    instruction: str
    kind: str
    axis: Optional[int]
    sign: int


STAGES = (
    Stage(
        "static_start",
        "起始静止基线",
        "按 b 开始录制后，放在稳定支撑面上完全不动至少 5 秒，然后按 n。",
        "static",
        None,
        0,
    ),
    Stage(
        "x_forward",
        "X 正方向平移（前移）",
        "按 b 开始录制后，姿态保持不变；先静止约 1 秒，再沿 X/传感器正前方"
        "移动 30–50 cm，"
        "用时约 1–2 秒；停稳至少 2 秒后按 n。",
        "translation",
        0,
        1,
    ),
    Stage(
        "x_backward",
        "X 负方向平移（后移）",
        "按 b 开始录制后，姿态保持不变；先静止约 1 秒，再沿 -X/后方移动 30–50 cm；"
        "停稳至少 2 秒后按 n。",
        "translation",
        0,
        -1,
    ),
    Stage(
        "y_left",
        "Y 正方向平移（左移）",
        "按 b 开始录制后，姿态保持不变；先静止约 1 秒，再向传感器物理左侧"
        "移动 30–50 cm；"
        "停稳至少 2 秒后按 n。FLU 中左侧为 +Y。",
        "translation",
        1,
        1,
    ),
    Stage(
        "y_right",
        "Y 负方向平移（右移）",
        "按 b 开始录制后，姿态保持不变；先静止约 1 秒，再向传感器物理右侧"
        "移动 30–50 cm；"
        "停稳至少 2 秒后按 n。FLU 中右侧为 -Y。",
        "translation",
        1,
        -1,
    ),
    Stage(
        "z_up",
        "Z 正方向平移（上移）",
        "按 b 开始录制后，姿态保持不变；先静止约 1 秒，再竖直向上移动 15–30 cm；"
        "放到稳定支撑面并静止至少 2 秒后按 n。FLU 中上方为 +Z。",
        "translation",
        2,
        1,
    ),
    Stage(
        "z_down",
        "Z 负方向平移（下移）",
        "按 b 开始录制后，姿态保持不变；先静止约 1 秒，再竖直向下移动 15–30 cm；"
        "放稳至少 2 秒后按 n。FLU 中下方为 -Z。",
        "translation",
        2,
        -1,
    ),
    Stage(
        "yaw_left",
        "绕 Z 正方向旋转（左转）",
        "按 b 开始录制后，保持大致水平并绕传感器中心旋转；从上向下看逆时针"
        "转 45–90°，"
        "停稳至少 2 秒后按 n。右手定则下期望 gyro_z > 0。",
        "rotation",
        2,
        1,
    ),
    Stage(
        "yaw_right",
        "绕 Z 负方向旋转（右转）",
        "按 b 开始录制后，保持大致水平并绕传感器中心旋转；从上向下看顺时针转回，"
        "停稳至少 2 秒后按 n。期望 gyro_z < 0。",
        "rotation",
        2,
        -1,
    ),
    Stage(
        "roll_positive",
        "绕 X 正方向旋转",
        "按 b 开始录制后，按右手定则绕机体 +X 轴旋转约 30–45°，"
        "停稳至少 2 秒后按 n；"
        "期望 gyro_x > 0。不能方便完成可按 s 跳过。",
        "rotation",
        0,
        1,
    ),
    Stage(
        "roll_negative",
        "绕 X 负方向旋转",
        "按 b 开始录制后，按右手定则绕机体 -X 方向转回初始姿态，"
        "停稳至少 2 秒后按 n；"
        "期望 gyro_x < 0。",
        "rotation",
        0,
        -1,
    ),
    Stage(
        "pitch_positive",
        "绕 Y 正方向旋转",
        "按 b 开始录制后，按右手定则绕机体 +Y 轴旋转约 30–45°，"
        "停稳至少 2 秒后按 n；"
        "期望 gyro_y > 0。不能方便完成可按 s 跳过。",
        "rotation",
        1,
        1,
    ),
    Stage(
        "pitch_negative",
        "绕 Y 负方向旋转",
        "按 b 开始录制后，按右手定则绕机体 -Y 方向转回初始姿态，"
        "停稳至少 2 秒后按 n；"
        "期望 gyro_y < 0。",
        "rotation",
        1,
        -1,
    ),
    Stage(
        "static_end",
        "结束静止基线",
        "按 b 开始录制后，恢复到稳定支撑面，完全不动至少 5 秒，然后按 n；"
        "用于检查全程后的漂移。",
        "static",
        None,
        0,
    ),
)


CSV_FIELDS = (
    "session_id",
    "stage_index",
    "stage_name",
    "stage_title",
    "attempt",
    "attempt_status",
    "expected_kind",
    "expected_axis",
    "expected_sign",
    "sample_index",
    "capture_unix_s",
    "capture_monotonic_s",
    "imu_host_monotonic_s",
    "imu_sequence",
    "imu_header_crc8_raw",
    "imu_payload_crc16_high_raw",
    "imu_payload_crc16_low_raw",
    "imu_timestamp_u32_us",
    "imu_packet_count",
    "imu_packet_delta",
    "estimated_rows_missed",
    "ahrs_host_monotonic_s",
    "ahrs_sequence",
    "ahrs_header_crc8_raw",
    "ahrs_payload_crc16_high_raw",
    "ahrs_payload_crc16_low_raw",
    "ahrs_timestamp_u32_us",
    "ahrs_packet_count",
    "imu_ahrs_host_offset_s",
    "gyro_flu_x_rad_s",
    "gyro_flu_y_rad_s",
    "gyro_flu_z_rad_s",
    "accel_raw_flu_x_mps2",
    "accel_raw_flu_y_mps2",
    "accel_raw_flu_z_mps2",
    "mag_flu_x_mg",
    "mag_flu_y_mg",
    "mag_flu_z_mg",
    "vendor_gyro_frd_x_rad_s",
    "vendor_gyro_frd_y_rad_s",
    "vendor_gyro_frd_z_rad_s",
    "vendor_accel_frd_x_mps2",
    "vendor_accel_frd_y_mps2",
    "vendor_accel_frd_z_mps2",
    "imu_temperature_raw",
    "pressure_raw",
    "pressure_temperature_raw",
    "ahrs_roll_speed_rad_s",
    "ahrs_pitch_speed_rad_s",
    "ahrs_heading_speed_rad_s",
    "roll_flu_rad",
    "pitch_flu_rad",
    "heading_flu_rad",
    "vendor_roll_speed_raw",
    "vendor_pitch_speed_raw",
    "vendor_heading_speed_raw",
    "vendor_roll_raw",
    "vendor_pitch_raw",
    "vendor_heading_raw",
    "q1_vendor_raw",
    "q2_vendor_raw",
    "q3_vendor_raw",
    "q4_vendor_raw",
    "velocity_body_flu_x_mps",
    "velocity_body_flu_y_mps",
    "velocity_body_flu_z_mps",
    "velocity_world_x_mps",
    "velocity_world_y_mps",
    "velocity_world_z_mps",
    "linear_accel_body_flu_x_mps2",
    "linear_accel_body_flu_y_mps2",
    "linear_accel_body_flu_z_mps2",
    "linear_accel_world_x_mps2",
    "linear_accel_world_y_mps2",
    "linear_accel_world_z_mps2",
    "gravity_body_flu_x_unit",
    "gravity_body_flu_y_unit",
    "gravity_body_flu_z_unit",
    "accel_bias_flu_x_mps2",
    "accel_bias_flu_y_mps2",
    "accel_bias_flu_z_mps2",
    "gyro_bias_flu_x_rad_s",
    "gyro_bias_flu_y_rad_s",
    "gyro_bias_flu_z_rad_s",
    "calibrated",
    "policy_data_ready",
    "velocity_valid",
    "stationary",
    "stationary_elapsed_s",
    "accelerometer_rest_sign",
    "velocity_dt_s",
    "velocity_host_dt_s",
    "velocity_dt_source",
    "zupt_count",
    "imu_fresh",
    "ahrs_fresh",
    "imu_age_s",
    "ahrs_age_s",
    "imu_last_interarrival_s",
    "ahrs_last_interarrival_s",
    "imu_gap_over_timeout_count",
    "ahrs_gap_over_timeout_count",
    "velocity_dt_reject_count",
    "timestamp_stall_count",
    "serial_error_count",
    "parse_error_count",
    "frame_error_count",
    "crc_error_count",
    "header_crc_error_count",
    "payload_crc_error_count",
    "discarded_byte_count",
    "last_error",
    "velocity_estimator_error",
)


class TerminalKeys:
    """Read one key without Enter while preserving Ctrl+C behavior."""

    def __init__(self) -> None:
        self.fd = sys.stdin.fileno()
        self.old_settings = None  # type: Optional[List[Any]]

    def __enter__(self) -> "TerminalKeys":
        if not sys.stdin.isatty():
            raise RuntimeError("交互测试要求在真实终端中运行（stdin 必须是 TTY）")
        self.old_settings = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        termios.tcflush(self.fd, termios.TCIFLUSH)
        return self

    def __exit__(self, exc_type: Any, exc: Any, traceback: Any) -> None:
        if self.old_settings is not None:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)

    def read(self, timeout_s: float) -> Optional[str]:
        readable, _, _ = select.select([self.fd], [], [], timeout_s)
        if not readable:
            return None
        value = os.read(self.fd, 1)
        if not value:
            return None
        return value.decode("utf-8", errors="ignore").lower()

    def flush_input(self) -> None:
        termios.tcflush(self.fd, termios.TCIFLUSH)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--baudrate", type=int, default=921600)
    parser.add_argument(
        "--output-dir",
        default=str(Path(__file__).resolve().parent / "test_logs"),
        help="parent directory for the timestamped result folder",
    )
    parser.add_argument(
        "--stages",
        default="all",
        help="all, linear, angular, or comma-separated stage names",
    )
    parser.add_argument("--poll-rate", type=float, default=250.0)
    parser.add_argument("--status-rate", type=float, default=5.0)
    parser.add_argument("--calibration-timeout", type=float, default=30.0)
    parser.add_argument("--ready-still-seconds", type=float, default=0.5)
    parser.add_argument("--max-stage-seconds", type=float, default=120.0)
    parser.add_argument(
        "--list-stages",
        action="store_true",
        help="print stage names and exit without opening the serial port",
    )
    return parser.parse_args()


def select_stages(specification: str) -> List[Stage]:
    if specification == "all":
        return list(STAGES)
    if specification == "linear":
        names = {
            "static_start",
            "x_forward",
            "x_backward",
            "y_left",
            "y_right",
            "z_up",
            "z_down",
            "static_end",
        }
        return [stage for stage in STAGES if stage.name in names]
    if specification == "angular":
        names = {
            "static_start",
            "yaw_left",
            "yaw_right",
            "roll_positive",
            "roll_negative",
            "pitch_positive",
            "pitch_negative",
            "static_end",
        }
        return [stage for stage in STAGES if stage.name in names]

    requested = [item.strip() for item in specification.split(",") if item.strip()]
    known = {stage.name: stage for stage in STAGES}
    unknown = [name for name in requested if name not in known]
    if unknown:
        raise ValueError("unknown stage(s): {}".format(", ".join(unknown)))
    if not requested:
        raise ValueError("--stages must not be empty")
    return [known[name] for name in requested]


def create_session_directory(parent: str) -> Tuple[str, Path]:
    parent_path = Path(parent).expanduser().resolve()
    parent_path.mkdir(parents=True, exist_ok=True)
    base_id = time.strftime("deta40_%Y%m%d_%H%M%S")
    for suffix in range(1000):
        session_id = base_id if suffix == 0 else "{}_{:02d}".format(base_id, suffix)
        path = parent_path / session_id
        try:
            path.mkdir()
            return session_id, path
        except FileExistsError:
            continue
    raise RuntimeError("could not allocate a unique output directory")


def vector_norm(values: Sequence[float]) -> float:
    return math.sqrt(sum(float(value) * float(value) for value in values))


def wrap_pi(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def percentile(values: Sequence[float], probability: float) -> Optional[float]:
    if not values:
        return None
    ordered = sorted(values)
    position = probability * (len(ordered) - 1)
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    fraction = position - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


def gravity_angle_deg(first: Sequence[float], second: Sequence[float]) -> float:
    denominator = vector_norm(first) * vector_norm(second)
    if denominator <= 1e-12:
        return 0.0
    cosine = sum(first[index] * second[index] for index in range(3)) / denominator
    cosine = max(-1.0, min(1.0, cosine))
    return math.degrees(math.acos(cosine))


def get_value(data: Optional[Dict[str, Any]], key: str) -> Any:
    if data is None:
        return None
    return data.get(key)


def get_vector(data: Dict[str, Any], key: str, index: int) -> Any:
    value = data.get(key)
    if value is None:
        return None
    return value[index]


def same_packet(first: Optional[Dict[str, Any]], second: Optional[Dict[str, Any]]) -> bool:
    if first is None or second is None:
        return first is None and second is None
    return first.get("host_monotonic_s") == second.get("host_monotonic_s")


def take_snapshot(imu: IMUSDK) -> Optional[Dict[str, Any]]:
    """Take the tightest snapshot possible using only the public SDK API."""

    for _ in range(5):
        imu_before = imu.get_raw_imu_data()
        if imu_before is None:
            return None
        ahrs_before = imu.get_ahrs_data()
        diagnostics = imu.get_diagnostics()
        gravity = imu.get_gravity_acceleration()
        imu_after = imu.get_raw_imu_data()
        ahrs_after = imu.get_ahrs_data()
        if not same_packet(imu_before, imu_after):
            continue
        if not same_packet(ahrs_before, ahrs_after):
            continue

        imu_time = float(imu_before["host_monotonic_s"])
        ahrs_time = get_value(ahrs_before, "host_monotonic_s")
        sample = {
            "capture_unix_s": time.time(),
            "capture_monotonic_s": time.monotonic(),
            "imu_host_monotonic_s": imu_time,
            "imu_sequence": imu_before["sequence"],
            "imu_header_crc8_raw": imu_before["header_crc8"],
            "imu_payload_crc16_high_raw": imu_before["payload_crc16_high"],
            "imu_payload_crc16_low_raw": imu_before["payload_crc16_low"],
            "imu_timestamp_u32_us": imu_before["timestamp_word0_u32_us"],
            "imu_packet_count": diagnostics["imu_packet_count"],
            "ahrs_host_monotonic_s": ahrs_time,
            "ahrs_sequence": get_value(ahrs_before, "sequence"),
            "ahrs_header_crc8_raw": get_value(ahrs_before, "header_crc8"),
            "ahrs_payload_crc16_high_raw": get_value(
                ahrs_before, "payload_crc16_high"
            ),
            "ahrs_payload_crc16_low_raw": get_value(
                ahrs_before, "payload_crc16_low"
            ),
            "ahrs_timestamp_u32_us": get_value(
                ahrs_before, "timestamp_word0_u32_us"
            ),
            "ahrs_packet_count": diagnostics["ahrs_packet_count"],
            "imu_ahrs_host_offset_s": (
                None if ahrs_time is None else imu_time - float(ahrs_time)
            ),
            "gyro_flu_x_rad_s": imu_before["gyro_x"],
            "gyro_flu_y_rad_s": imu_before["gyro_y"],
            "gyro_flu_z_rad_s": imu_before["gyro_z"],
            "accel_raw_flu_x_mps2": imu_before["acc_x"],
            "accel_raw_flu_y_mps2": imu_before["acc_y"],
            "accel_raw_flu_z_mps2": imu_before["acc_z"],
            "mag_flu_x_mg": imu_before["mag_x"],
            "mag_flu_y_mg": imu_before["mag_y"],
            "mag_flu_z_mg": imu_before["mag_z"],
            "vendor_gyro_frd_x_rad_s": imu_before["vendor_gyro_x"],
            "vendor_gyro_frd_y_rad_s": imu_before["vendor_gyro_y"],
            "vendor_gyro_frd_z_rad_s": imu_before["vendor_gyro_z"],
            "vendor_accel_frd_x_mps2": imu_before["vendor_acc_x"],
            "vendor_accel_frd_y_mps2": imu_before["vendor_acc_y"],
            "vendor_accel_frd_z_mps2": imu_before["vendor_acc_z"],
            "imu_temperature_raw": imu_before["imu_temperature"],
            "pressure_raw": imu_before["pressure"],
            "pressure_temperature_raw": imu_before["pressure_temperature"],
            "ahrs_roll_speed_rad_s": get_value(ahrs_before, "roll_speed"),
            "ahrs_pitch_speed_rad_s": get_value(ahrs_before, "pitch_speed"),
            "ahrs_heading_speed_rad_s": get_value(ahrs_before, "heading_speed"),
            "roll_flu_rad": get_value(ahrs_before, "roll"),
            "pitch_flu_rad": get_value(ahrs_before, "pitch"),
            "heading_flu_rad": get_value(ahrs_before, "heading"),
            "vendor_roll_speed_raw": get_value(ahrs_before, "vendor_roll_speed"),
            "vendor_pitch_speed_raw": get_value(
                ahrs_before, "vendor_pitch_speed"
            ),
            "vendor_heading_speed_raw": get_value(
                ahrs_before, "vendor_heading_speed"
            ),
            "vendor_roll_raw": get_value(ahrs_before, "vendor_roll"),
            "vendor_pitch_raw": get_value(ahrs_before, "vendor_pitch"),
            "vendor_heading_raw": get_value(ahrs_before, "vendor_heading"),
            "q1_vendor_raw": get_value(ahrs_before, "q1"),
            "q2_vendor_raw": get_value(ahrs_before, "q2"),
            "q3_vendor_raw": get_value(ahrs_before, "q3"),
            "q4_vendor_raw": get_value(ahrs_before, "q4"),
            "velocity_body_flu_x_mps": get_vector(
                diagnostics, "linear_velocity_body", 0
            ),
            "velocity_body_flu_y_mps": get_vector(
                diagnostics, "linear_velocity_body", 1
            ),
            "velocity_body_flu_z_mps": get_vector(
                diagnostics, "linear_velocity_body", 2
            ),
            "velocity_world_x_mps": get_vector(diagnostics, "velocity_world", 0),
            "velocity_world_y_mps": get_vector(diagnostics, "velocity_world", 1),
            "velocity_world_z_mps": get_vector(diagnostics, "velocity_world", 2),
            "linear_accel_body_flu_x_mps2": get_vector(
                diagnostics, "linear_acceleration_body", 0
            ),
            "linear_accel_body_flu_y_mps2": get_vector(
                diagnostics, "linear_acceleration_body", 1
            ),
            "linear_accel_body_flu_z_mps2": get_vector(
                diagnostics, "linear_acceleration_body", 2
            ),
            "linear_accel_world_x_mps2": get_vector(
                diagnostics, "linear_acceleration_world", 0
            ),
            "linear_accel_world_y_mps2": get_vector(
                diagnostics, "linear_acceleration_world", 1
            ),
            "linear_accel_world_z_mps2": get_vector(
                diagnostics, "linear_acceleration_world", 2
            ),
            "gravity_body_flu_x_unit": gravity[0],
            "gravity_body_flu_y_unit": gravity[1],
            "gravity_body_flu_z_unit": gravity[2],
            "accel_bias_flu_x_mps2": get_vector(diagnostics, "accel_bias_flu", 0),
            "accel_bias_flu_y_mps2": get_vector(diagnostics, "accel_bias_flu", 1),
            "accel_bias_flu_z_mps2": get_vector(diagnostics, "accel_bias_flu", 2),
            "gyro_bias_flu_x_rad_s": get_vector(diagnostics, "gyro_bias_flu", 0),
            "gyro_bias_flu_y_rad_s": get_vector(diagnostics, "gyro_bias_flu", 1),
            "gyro_bias_flu_z_rad_s": get_vector(diagnostics, "gyro_bias_flu", 2),
            "calibrated": diagnostics["calibrated"],
            "policy_data_ready": diagnostics["policy_data_ready"],
            "velocity_valid": diagnostics["velocity_valid"],
            "stationary": diagnostics["stationary"],
            "stationary_elapsed_s": diagnostics["stationary_elapsed_s"],
            "accelerometer_rest_sign": diagnostics["accelerometer_rest_sign"],
            "velocity_dt_s": diagnostics["velocity_dt_s"],
            "velocity_host_dt_s": diagnostics["velocity_host_dt_s"],
            "velocity_dt_source": diagnostics["velocity_dt_source"],
            "zupt_count": diagnostics["zupt_count"],
            "imu_fresh": diagnostics["imu_fresh"],
            "ahrs_fresh": diagnostics["ahrs_fresh"],
            "imu_age_s": diagnostics["imu_age_s"],
            "ahrs_age_s": diagnostics["ahrs_age_s"],
            "imu_last_interarrival_s": diagnostics["imu_last_interarrival_s"],
            "ahrs_last_interarrival_s": diagnostics["ahrs_last_interarrival_s"],
            "imu_gap_over_timeout_count": diagnostics[
                "imu_gap_over_timeout_count"
            ],
            "ahrs_gap_over_timeout_count": diagnostics[
                "ahrs_gap_over_timeout_count"
            ],
            "velocity_dt_reject_count": diagnostics["velocity_dt_reject_count"],
            "timestamp_stall_count": diagnostics["timestamp_stall_count"],
            "serial_error_count": diagnostics["serial_error_count"],
            "parse_error_count": diagnostics["parse_error_count"],
            "frame_error_count": diagnostics["frame_error_count"],
            "crc_error_count": diagnostics["crc_error_count"],
            "header_crc_error_count": diagnostics[
                "header_crc_error_count"
            ],
            "payload_crc_error_count": diagnostics[
                "payload_crc_error_count"
            ],
            "discarded_byte_count": diagnostics["discarded_byte_count"],
            "last_error": diagnostics["last_error"],
            "velocity_estimator_error": diagnostics["velocity_estimator_error"],
        }
        return sample
    return None


def sample_is_new(sample: Dict[str, Any], previous_imu_time: Optional[float]) -> bool:
    return previous_imu_time != sample["imu_host_monotonic_s"]


def add_packet_delta(
    sample: Dict[str, Any], previous_packet_count: Optional[int]
) -> None:
    if previous_packet_count is None:
        delta = 1
    else:
        delta = max(0, int(sample["imu_packet_count"]) - previous_packet_count)
    sample["imu_packet_delta"] = delta
    sample["estimated_rows_missed"] = max(0, delta - 1)


def vector_metrics(samples: Sequence[Dict[str, Any]], fields: Sequence[str]) -> Dict[str, Any]:
    columns = [[float(sample[field]) for sample in samples] for field in fields]
    norms = [
        math.sqrt(sum(float(sample[field]) ** 2 for field in fields))
        for sample in samples
    ]
    return {
        "axes": {
            axis: {
                "min": min(column),
                "max": max(column),
                "peak_abs": max(abs(min(column)), abs(max(column))),
                "rms": math.sqrt(sum(value * value for value in column) / len(column)),
            }
            for axis, column in zip(AXES, columns)
        },
        "norm_rms": math.sqrt(sum(value * value for value in norms) / len(norms)),
        "norm_p95": percentile(norms, 0.95),
        "norm_max": max(norms),
        "final": [columns[index][-1] for index in range(3)],
    }


def integrate_signal(samples: Sequence[Dict[str, Any]], field: str) -> float:
    total = 0.0
    for previous, current in zip(samples[:-1], samples[1:]):
        dt = float(current["velocity_dt_s"])
        if not 0.0 < dt <= 0.1:
            current_timestamp = int(current["imu_timestamp_u32_us"])
            previous_timestamp = int(previous["imu_timestamp_u32_us"])
            dt = ((current_timestamp - previous_timestamp) & 0xFFFFFFFF) * 1e-6
        if not 0.0 < dt <= 0.1:
            dt = float(current["imu_host_monotonic_s"]) - float(
                previous["imu_host_monotonic_s"]
            )
        if 0.0 < dt <= 0.1:
            total += 0.5 * (float(previous[field]) + float(current[field])) * dt
    return total


def direction_assessment(
    samples: Sequence[Dict[str, Any]],
    target_field: str,
    cross_fields: Sequence[str],
    sign: int,
    minimum_signal: float,
) -> Dict[str, Any]:
    target = [float(sample[target_field]) for sample in samples]
    projected = [sign * value for value in target]
    correct_peak = max(0.0, max(projected))
    wrong_peak = max(0.0, -min(projected))
    cross_peak = max(
        math.sqrt(sum(float(sample[field]) ** 2 for field in cross_fields))
        for sample in samples
    )
    active = [value for value in projected if abs(value) >= 0.5 * minimum_signal]
    direction_fraction = (
        None
        if not active
        else sum(1 for value in active if value > 0.0) / float(len(active))
    )

    if correct_peak >= minimum_signal and correct_peak >= 1.5 * wrong_peak:
        if correct_peak >= cross_peak:
            conclusion = "consistent"
        else:
            conclusion = "sign_consistent_but_cross_axis_dominant"
    elif wrong_peak >= minimum_signal and wrong_peak >= 1.5 * correct_peak:
        conclusion = "opposite"
    else:
        conclusion = "inconclusive"

    return {
        "conclusion": conclusion,
        "correct_direction_peak": correct_peak,
        "wrong_direction_peak": wrong_peak,
        "cross_axis_peak_norm": cross_peak,
        "target_to_cross_ratio": (
            None if cross_peak <= 1e-12 else correct_peak / cross_peak
        ),
        "expected_direction_fraction_when_active": direction_fraction,
    }


def summarize_attempt(
    stage: Stage,
    stage_index: int,
    attempt: int,
    status: str,
    samples: Sequence[Dict[str, Any]],
) -> Tuple[Dict[str, Any], List[str]]:
    summary = {
        "stage_index": stage_index,
        "stage_name": stage.name,
        "stage_title": stage.title,
        "attempt": attempt,
        "status": status,
        "instruction": stage.instruction,
        "expected_kind": stage.kind,
        "expected_axis": None if stage.axis is None else AXES[stage.axis],
        "expected_sign": stage.sign,
        "sample_count": len(samples),
    }  # type: Dict[str, Any]
    lines = [
        "[{}] {} / attempt {} / {}".format(stage.name, stage.title, attempt, status)
    ]
    if not samples:
        summary["assessment"] = "no_data"
        lines.append("  no samples recorded")
        return summary, lines

    duration = max(
        0.0,
        float(samples[-1]["imu_host_monotonic_s"])
        - float(samples[0]["imu_host_monotonic_s"]),
    )
    missed = sum(int(sample["estimated_rows_missed"]) for sample in samples)
    velocity_fields = tuple(
        "velocity_body_flu_{}_mps".format(axis) for axis in AXES
    )
    acceleration_fields = tuple(
        "linear_accel_body_flu_{}_mps2".format(axis) for axis in AXES
    )
    gyro_fields = tuple("gyro_flu_{}_rad_s".format(axis) for axis in AXES)
    gravity_fields = tuple("gravity_body_flu_{}_unit".format(axis) for axis in AXES)
    velocity = vector_metrics(samples, velocity_fields)
    acceleration = vector_metrics(samples, acceleration_fields)
    gyroscope = vector_metrics(samples, gyro_fields)
    gravity = vector_metrics(samples, gravity_fields)

    first_gravity = [float(samples[0][field]) for field in gravity_fields]
    final_gravity = [float(samples[-1][field]) for field in gravity_fields]
    gravity_changes = [
        gravity_angle_deg(
            first_gravity, [float(sample[field]) for field in gravity_fields]
        )
        for sample in samples
    ]
    headings = [
        float(sample["heading_flu_rad"])
        for sample in samples
        if sample["heading_flu_rad"] is not None
    ]
    unwrapped_heading_change = sum(
        wrap_pi(current - previous)
        for previous, current in zip(headings[:-1], headings[1:])
    )

    summary.update(
        {
            "duration_s": duration,
            "observed_sample_rate_hz": (
                None if duration <= 0.0 else (len(samples) - 1) / duration
            ),
            "estimated_rows_missed": missed,
            "policy_ready_fraction": sum(
                bool(sample["policy_data_ready"]) for sample in samples
            )
            / float(len(samples)),
            "stationary_fraction": sum(
                bool(sample["stationary"]) for sample in samples
            )
            / float(len(samples)),
            "velocity_body_flu_mps": velocity,
            "linear_acceleration_body_flu_mps2": acceleration,
            "gyro_flu_rad_s": gyroscope,
            "gravity_body_flu_unit": gravity,
            "gravity_final_change_deg": gravity_angle_deg(
                first_gravity, final_gravity
            ),
            "gravity_max_change_deg": max(gravity_changes),
            "heading_unwrapped_change_deg": math.degrees(
                unwrapped_heading_change
            ),
            "zupt_count_delta": int(samples[-1]["zupt_count"])
            - int(samples[0]["zupt_count"]),
            "final_stationary": bool(samples[-1]["stationary"]),
            "final_stationary_elapsed_s": float(
                samples[-1]["stationary_elapsed_s"]
            ),
            "final_policy_data_ready": bool(samples[-1]["policy_data_ready"]),
            "final_velocity_valid": bool(samples[-1]["velocity_valid"]),
            "final_calibrated": bool(samples[-1]["calibrated"]),
            "final_velocity_norm_mps": vector_norm(velocity["final"]),
            "health_counter_deltas": {
                key: int(samples[-1][key]) - int(samples[0][key])
                for key in HEALTH_COUNTERS
            },
            "last_error": samples[-1]["last_error"],
            "velocity_estimator_error": samples[-1][
                "velocity_estimator_error"
            ],
        }
    )

    if stage.kind == "translation" and stage.axis is not None:
        target_field = velocity_fields[stage.axis]
        cross_fields = tuple(
            field for index, field in enumerate(velocity_fields) if index != stage.axis
        )
        direction = direction_assessment(
            samples, target_field, cross_fields, stage.sign, 0.01
        )
        direction["projected_velocity_time_integral_m_estimate"] = (
            stage.sign * integrate_signal(samples, target_field)
        )
        summary["direction"] = direction
        summary["assessment"] = direction["conclusion"]
        lines.append(
            "  direction={} correct_peak={:.5f} m/s wrong_peak={:.5f} m/s "
            "cross_peak={:.5f} m/s".format(
                direction["conclusion"],
                direction["correct_direction_peak"],
                direction["wrong_direction_peak"],
                direction["cross_axis_peak_norm"],
            )
        )
        lines.append(
            "  imu_only_projected_displacement_estimate={:.4f} m "
            "(no external ground truth)".format(
                direction["projected_velocity_time_integral_m_estimate"]
            )
        )
    elif stage.kind == "rotation" and stage.axis is not None:
        target_field = gyro_fields[stage.axis]
        cross_fields = tuple(
            field for index, field in enumerate(gyro_fields) if index != stage.axis
        )
        direction = direction_assessment(
            samples, target_field, cross_fields, stage.sign, 0.05
        )
        direction["integrated_target_angle_deg"] = math.degrees(
            stage.sign * integrate_signal(samples, target_field)
        )
        summary["direction"] = direction
        summary["assessment"] = direction["conclusion"]
        lines.append(
            "  direction={} correct_peak={:.5f} rad/s wrong_peak={:.5f} rad/s "
            "cross_peak={:.5f} rad/s".format(
                direction["conclusion"],
                direction["correct_direction_peak"],
                direction["wrong_direction_peak"],
                direction["cross_axis_peak_norm"],
            )
        )
        lines.append(
            "  integrated_target={:.2f} deg heading_change={:.2f} deg".format(
                direction["integrated_target_angle_deg"],
                summary["heading_unwrapped_change_deg"],
            )
        )
    else:
        summary["assessment"] = "observed"
        lines.append(
            "  static accel_norm_rms={:.5f} m/s^2 gyro_norm_rms={:.6f} rad/s "
            "velocity_norm_max={:.5f} m/s".format(
                acceleration["norm_rms"],
                gyroscope["norm_rms"],
                velocity["norm_max"],
            )
        )

    summary["provisional_assessment"] = summary["assessment"]
    validity_reasons = []  # type: List[str]
    if status != "completed":
        validity_reasons.append("attempt_status_is_{}".format(status))
    if stage.name != "calibration" and status == "completed":
        if not summary["final_policy_data_ready"]:
            validity_reasons.append("policy_not_ready_at_end")
        if not summary["final_velocity_valid"] or not summary["final_calibrated"]:
            validity_reasons.append("velocity_estimator_not_valid_at_end")
        if not summary["final_stationary"]:
            validity_reasons.append("not_stationary_at_end")
        elif summary["final_stationary_elapsed_s"] < 0.75:
            validity_reasons.append("final_stationary_window_shorter_than_0.75_s")
        if summary["policy_ready_fraction"] < 0.95:
            validity_reasons.append("policy_ready_fraction_below_0.95")
    summary["validity_reasons"] = validity_reasons
    if validity_reasons:
        summary["assessment"] = "invalid_or_incomplete"
        lines.append(
            "  final_assessment=invalid_or_incomplete reasons={}".format(
                validity_reasons
            )
        )

    lines.append(
        "  duration={:.2f}s samples={} observed_rate={:.1f}Hz missed={} "
        "final_v_norm={:.6f}m/s final_stationary={} zupt_delta={}".format(
            duration,
            len(samples),
            summary["observed_sample_rate_hz"] or 0.0,
            missed,
            summary["final_velocity_norm_mps"],
            summary["final_stationary"],
            summary["zupt_count_delta"],
        )
    )
    lines.append(
        "  gravity_change_final={:.3f}deg max={:.3f}deg counters={}".format(
            summary["gravity_final_change_deg"],
            summary["gravity_max_change_deg"],
            summary["health_counter_deltas"],
        )
    )
    return summary, lines


def sanitize_json(value: Any) -> Any:
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {str(key): sanitize_json(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [sanitize_json(item) for item in value]
    return value


def checkpoint_json(path: Path, document: Dict[str, Any]) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("w", encoding="utf-8") as output:
        json.dump(
            sanitize_json(document),
            output,
            ensure_ascii=False,
            indent=2,
            allow_nan=False,
        )
        output.write("\n")
        output.flush()
        os.fsync(output.fileno())
    os.replace(str(temporary), str(path))


def write_attempt_rows(
    writer: csv.DictWriter,
    output_file: Any,
    session_id: str,
    stage: Stage,
    stage_index: int,
    attempt: int,
    status: str,
    samples: Sequence[Dict[str, Any]],
) -> None:
    expected_axis = None if stage.axis is None else AXES[stage.axis]
    for sample_index, sample in enumerate(samples):
        row = dict(sample)
        row.update(
            {
                "session_id": session_id,
                "stage_index": stage_index,
                "stage_name": stage.name,
                "stage_title": stage.title,
                "attempt": attempt,
                "attempt_status": status,
                "expected_kind": stage.kind,
                "expected_axis": expected_axis,
                "expected_sign": stage.sign,
                "sample_index": sample_index,
            }
        )
        writer.writerow({field: row.get(field) for field in CSV_FIELDS})
    output_file.flush()
    os.fsync(output_file.fileno())


def clear_status_line() -> None:
    print("\r\033[2K", end="", flush=True)


def print_recording_status(elapsed: float, sample: Optional[Dict[str, Any]]) -> None:
    if sample is None:
        message = "录制 {:6.1f}s | 等待新 IMU 帧 | n完成 r重做 s跳过 q退出".format(
            elapsed
        )
    else:
        message = (
            "录制 {:6.1f}s | v=[{:+.3f},{:+.3f},{:+.3f}] m/s | "
            "a=[{:+.2f},{:+.2f},{:+.2f}] m/s² | stationary={} | "
            "n完成 r重做 s跳过 q退出"
        ).format(
            elapsed,
            sample["velocity_body_flu_x_mps"],
            sample["velocity_body_flu_y_mps"],
            sample["velocity_body_flu_z_mps"],
            sample["linear_accel_body_flu_x_mps2"],
            sample["linear_accel_body_flu_y_mps2"],
            sample["linear_accel_body_flu_z_mps2"],
            sample["stationary"],
        )
    print("\r\033[2K" + message, end="", flush=True)


def wait_for_stage_command(
    imu: IMUSDK, keys: TerminalKeys, stage: Stage, index: int, count: int
) -> str:
    keys.flush_input()
    print("\n[{}/{}] {} ({})".format(index, count, stage.title, stage.name))
    print(stage.instruction)
    print("按 b（或 Enter）开始录制；s 跳过；q 保存并退出。", flush=True)
    while True:
        key = keys.read(0.2)
        diagnostics = imu.get_diagnostics()
        if not diagnostics["running"]:
            raise RuntimeError(
                "IMU receiver stopped: {}".format(diagnostics["last_error"])
            )
        if key in ("b", "\r", "\n"):
            return "begin"
        if key == "s":
            return "skip"
        if key == "q":
            return "quit"


def record_attempt(
    imu: IMUSDK,
    keys: TerminalKeys,
    poll_period_s: float,
    status_period_s: float,
    max_stage_s: float,
    samples_out: Optional[List[Dict[str, Any]]] = None,
) -> Tuple[str, List[Dict[str, Any]], Optional[str]]:
    samples = [] if samples_out is None else samples_out
    previous_imu_time = None  # type: Optional[float]
    previous_packet_count = None  # type: Optional[int]
    started = time.monotonic()
    last_new_frame = started
    unhealthy_since = None  # type: Optional[float]
    next_status = started
    latest = None  # type: Optional[Dict[str, Any]]
    print("开始录制。先静止约 1 秒，再执行动作。", flush=True)

    while True:
        now = time.monotonic()
        sample = take_snapshot(imu)
        if sample is not None and sample_is_new(sample, previous_imu_time):
            add_packet_delta(sample, previous_packet_count)
            samples.append(sample)
            latest = sample
            previous_imu_time = float(sample["imu_host_monotonic_s"])
            previous_packet_count = int(sample["imu_packet_count"])
            last_new_frame = now
            healthy = (
                sample["calibrated"]
                and sample["velocity_valid"]
                and sample["policy_data_ready"]
                and sample["imu_fresh"]
                and sample["ahrs_fresh"]
            )
            if healthy:
                unhealthy_since = None
            elif unhealthy_since is None:
                unhealthy_since = now
            elif now - unhealthy_since >= 0.25:
                clear_status_line()
                return (
                    "capture_error",
                    samples,
                    "policy stream/estimator unhealthy for at least 0.25 s",
                )

        key = keys.read(0.0)
        if key == "n":
            clear_status_line()
            return "completed", samples, None
        if key == "r":
            clear_status_line()
            return "discarded_redo", samples, None
        if key == "s":
            clear_status_line()
            return "skipped_during_recording", samples, None
        if key == "q":
            clear_status_line()
            return "aborted_by_user", samples, None

        elapsed = now - started
        if elapsed >= max_stage_s:
            clear_status_line()
            return "timed_out_redo", samples, "stage exceeded maximum duration"
        if now - last_new_frame >= 1.0:
            diagnostics = imu.get_diagnostics()
            if (
                not diagnostics["running"]
                or not diagnostics["imu_fresh"]
            ):
                clear_status_line()
                return (
                    "capture_error",
                    samples,
                    "IMU stream stale/stopped: {}".format(
                        diagnostics["last_error"]
                    ),
                )
        if now >= next_status:
            print_recording_status(elapsed, latest)
            next_status = now + status_period_s
        time.sleep(poll_period_s)


def collect_calibration(
    imu: IMUSDK,
    timeout_s: float,
    ready_still_s: float,
    poll_period_s: float,
    samples_out: Optional[List[Dict[str, Any]]] = None,
) -> Tuple[List[Dict[str, Any]], Dict[str, Any]]:
    print("\n正在进行 SDK 启动静止标定：请把 IMU 放稳，不要触碰。", flush=True)
    started = time.monotonic()
    stable_since = None  # type: Optional[float]
    previous_imu_time = None  # type: Optional[float]
    previous_packet_count = None  # type: Optional[int]
    samples = [] if samples_out is None else samples_out
    next_status = started

    while True:
        now = time.monotonic()
        sample = take_snapshot(imu)
        if sample is not None and sample_is_new(sample, previous_imu_time):
            add_packet_delta(sample, previous_packet_count)
            samples.append(sample)
            previous_imu_time = float(sample["imu_host_monotonic_s"])
            previous_packet_count = int(sample["imu_packet_count"])

        diagnostics = imu.get_diagnostics()
        ready = (
            diagnostics["calibrated"]
            and diagnostics["velocity_valid"]
            and diagnostics["policy_data_ready"]
            and diagnostics["stationary"]
        )
        if ready:
            if stable_since is None:
                stable_since = now
            if now - stable_since >= ready_still_s:
                clear_status_line()
                print(
                    "标定完成：samples={} rest_sign={:+d} accel_bias={} gyro_bias={}".format(
                        diagnostics["calibration_sample_count"],
                        diagnostics["accelerometer_rest_sign"],
                        tuple(round(value, 6) for value in diagnostics["accel_bias_flu"]),
                        tuple(round(value, 6) for value in diagnostics["gyro_bias_flu"]),
                    )
                )
                return samples, diagnostics
        else:
            stable_since = None

        if now - started >= timeout_s:
            clear_status_line()
            raise RuntimeError(
                "startup calibration/readiness timed out; diagnostics={}".format(
                    diagnostics
                )
            )
        if not diagnostics["running"]:
            clear_status_line()
            raise RuntimeError(
                "IMU receiver stopped during calibration: {}".format(
                    diagnostics["last_error"]
                )
            )
        if now >= next_status:
            print(
                "\r\033[2K标定中 {:5.1f}s | samples={} calibrated={} ready={} "
                "stationary={} quiet={:.2f}s".format(
                    now - started,
                    diagnostics["calibration_sample_count"],
                    diagnostics["calibrated"],
                    diagnostics["policy_data_ready"],
                    diagnostics["stationary"],
                    diagnostics["stationary_elapsed_s"],
                ),
                end="",
                flush=True,
            )
            next_status = now + 0.2
        time.sleep(poll_period_s)


def append_report(path: Path, lines: Iterable[str]) -> None:
    with path.open("a", encoding="utf-8") as report:
        for line in lines:
            report.write(line + "\n")
        report.write("\n")
        report.flush()
        os.fsync(report.fileno())


def main() -> int:
    args = parse_args()
    if args.list_stages:
        for stage in STAGES:
            print("{:<18} {}".format(stage.name, stage.title))
        return 0
    if args.poll_rate <= 0.0 or args.status_rate <= 0.0:
        raise ValueError("--poll-rate and --status-rate must be positive")
    if args.calibration_timeout <= 0.0 or args.max_stage_seconds <= 0.0:
        raise ValueError("timeouts must be positive")
    if args.ready_still_seconds < 0.0:
        raise ValueError("--ready-still-seconds must be non-negative")

    stages = select_stages(args.stages)
    session_id, session_dir = create_session_directory(args.output_dir)
    csv_path = session_dir / "samples.csv"
    json_path = session_dir / "summary.json"
    report_path = session_dir / "report.txt"
    started_iso = time.strftime("%Y-%m-%dT%H:%M:%S%z")
    document = {
        "script": "test_guided_motion.py",
        "script_version": SCRIPT_VERSION,
        "session_id": session_id,
        "session_status": "starting",
        "started_at": started_iso,
        "ended_at": None,
        "port": args.port,
        "baudrate": args.baudrate,
        "output_frame": "FLU: x forward, y left, z up",
        "selected_stages": [stage.name for stage in stages],
        "files": {"samples_csv": str(csv_path), "report_txt": str(report_path)},
        "limitations": [
            "No external position or velocity ground truth is available.",
            "Results validate direction/axis response and ZUPT, not absolute velocity accuracy.",
            "The SDK exposes latest samples, not a queue; missed snapshots are estimated.",
            "Vendor demo does not define CRC algorithms, quaternion convention, "
            "or environmental units.",
            "discarded_byte_count includes expected unconsumed frame delimiters; "
            "it is not by itself a communication error.",
        ],
        "calibration": None,
        "attempts": [],
        "initial_diagnostics": None,
        "final_diagnostics": None,
        "error": None,
    }  # type: Dict[str, Any]
    checkpoint_json(json_path, document)
    report_path.write_text(
        "DETA40 guided motion test\n"
        "Frame: FLU (x forward, y left, z up)\n"
        "No external ground truth: assessments concern sign/axis/ZUPT only.\n\n",
        encoding="utf-8",
    )

    print("结果目录：{}".format(session_dir))
    print("逐帧数据：{}".format(csv_path))
    print("阶段摘要：{}".format(json_path))
    print("坐标系：FLU，X 前、Y 左、Z 上。")

    imu = IMUSDK(port=args.port, baudrate=args.baudrate)
    csv_file = None
    session_status = "error"
    exit_code = 1
    try:
        csv_file = csv_path.open("w", newline="", encoding="utf-8")
        writer = csv.DictWriter(csv_file, fieldnames=CSV_FIELDS)
        writer.writeheader()
        csv_file.flush()

        print("连接 DETA40：{} @ {} ...".format(args.port, args.baudrate))
        if not imu.start():
            raise RuntimeError(imu.get_diagnostics()["last_error"])
        document["initial_diagnostics"] = imu.get_diagnostics()
        document["session_status"] = "calibrating"
        checkpoint_json(json_path, document)

        calibration_stage = Stage(
            "calibration", "自动启动标定", "保持完全静止", "static", None, 0
        )
        calibration_samples = []  # type: List[Dict[str, Any]]

        def persist_calibration(status: str, diagnostics: Dict[str, Any]) -> None:
            summary, lines = summarize_attempt(
                calibration_stage,
                0,
                1,
                status,
                calibration_samples,
            )
            write_attempt_rows(
                writer,
                csv_file,
                session_id,
                calibration_stage,
                0,
                1,
                status,
                calibration_samples,
            )
            document["calibration"] = {
                "summary": summary,
                "final_diagnostics": diagnostics,
            }
            append_report(report_path, lines)
            checkpoint_json(json_path, document)

        try:
            calibration_samples, calibration_diagnostics = collect_calibration(
                imu,
                args.calibration_timeout,
                args.ready_still_seconds,
                1.0 / args.poll_rate,
                calibration_samples,
            )
        except KeyboardInterrupt:
            persist_calibration("interrupted", imu.get_diagnostics())
            raise
        except Exception:
            persist_calibration("failed", imu.get_diagnostics())
            raise
        persist_calibration("completed", calibration_diagnostics)
        document["session_status"] = "running"
        checkpoint_json(json_path, document)

        print("\n按键说明：b/Enter 开始，n 完成，r 重做，s 跳过，q 保存退出。")
        print("每个动作开始和结束都请保留静止段；屏幕只显示一行实时状态。")

        user_quit = False
        user_interrupted = False
        with TerminalKeys() as keys:
            for stage_index, stage in enumerate(stages, start=1):
                attempt = 1
                while True:
                    command = wait_for_stage_command(
                        imu, keys, stage, stage_index, len(stages)
                    )
                    if command == "quit":
                        user_quit = True
                        break
                    if command == "skip":
                        summary, lines = summarize_attempt(
                            stage,
                            stage_index,
                            attempt,
                            "skipped_before_recording",
                            [],
                        )
                        document["attempts"].append(summary)
                        append_report(report_path, lines)
                        checkpoint_json(json_path, document)
                        break

                    attempt_samples = []  # type: List[Dict[str, Any]]
                    fatal_exception = None  # type: Optional[Exception]
                    try:
                        status, samples, error = record_attempt(
                            imu,
                            keys,
                            1.0 / args.poll_rate,
                            1.0 / args.status_rate,
                            args.max_stage_seconds,
                            attempt_samples,
                        )
                    except KeyboardInterrupt:
                        clear_status_line()
                        status = "interrupted_by_ctrl_c"
                        samples = attempt_samples
                        error = None
                    except Exception as exc:
                        clear_status_line()
                        status = "capture_error"
                        samples = attempt_samples
                        error = "{}: {}".format(type(exc).__name__, exc)
                        fatal_exception = exc
                    summary, lines = summarize_attempt(
                        stage, stage_index, attempt, status, samples
                    )
                    if error is not None:
                        summary["capture_error"] = error
                        lines.append("  capture_error={}".format(error))
                    write_attempt_rows(
                        writer,
                        csv_file,
                        session_id,
                        stage,
                        stage_index,
                        attempt,
                        status,
                        samples,
                    )
                    document["attempts"].append(summary)
                    append_report(report_path, lines)
                    checkpoint_json(json_path, document)
                    print("\n".join(lines))

                    if status == "discarded_redo" or status == "timed_out_redo":
                        print("本阶段将重新开始；刚才的数据仍保留并标记为 {}。".format(status))
                        attempt += 1
                        continue
                    if status == "aborted_by_user":
                        user_quit = True
                    if status == "interrupted_by_ctrl_c":
                        user_interrupted = True
                    if status == "capture_error":
                        if fatal_exception is not None:
                            raise fatal_exception
                        raise RuntimeError(error)
                    break
                if user_quit or user_interrupted:
                    break

        if user_interrupted:
            session_status = "interrupted"
            exit_code = 130
        else:
            session_status = "user_quit" if user_quit else "completed"
            exit_code = 0
    except KeyboardInterrupt:
        clear_status_line()
        print("收到 Ctrl+C；正在保存已完成的阶段。")
        session_status = "interrupted"
        exit_code = 130
    except Exception as exc:
        clear_status_line()
        print("测试失败：{}".format(exc), file=sys.stderr)
        document["error"] = "{}: {}".format(type(exc).__name__, exc)
        session_status = "error"
        exit_code = 1
    finally:
        try:
            document["final_diagnostics"] = imu.get_diagnostics()
        except Exception as exc:
            document["final_diagnostics"] = {
                "unavailable": "{}: {}".format(type(exc).__name__, exc)
            }
        imu.stop()
        if csv_file is not None:
            csv_file.flush()
            try:
                os.fsync(csv_file.fileno())
            except OSError:
                pass
            csv_file.close()
        document["session_status"] = session_status
        document["ended_at"] = time.strftime("%Y-%m-%dT%H:%M:%S%z")
        checkpoint_json(json_path, document)

    print("\n测试状态：{}".format(session_status))
    print("请把这个目录发给我分析：{}".format(session_dir))
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
