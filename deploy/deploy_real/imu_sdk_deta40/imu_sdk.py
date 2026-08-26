#!/usr/bin/env python3
"""Threaded FDILink DETA40 IMU wrapper used by the sim2real scripts.

The serial protocol and packet layouts come from ``demo.py`` in this directory.
Only the IMU (0x40) and AHRS (0x41) packets are decoded.  All serial reading,
gravity extraction, gravity compensation, and velocity integration happen in a
background thread, so the public getters only copy the latest snapshot.

Internal calculations deliberately stay in the hardware frame:

    X forward, Y right, Z down (FRD)

Vector-valued public results are converted only when returned:

    X forward, Y left, Z up (FLU) = [x, -y, -z]

FRD and FLU are both right-handed; their relationship is a 180-degree rotation
around X.  Quaternion fields are converted only in the public return methods as
``[w, x, -y, -z]``.  All internal attitude math still uses the raw FRD
quaternion.
"""

import math
import struct
import threading
import time
from typing import Dict, List, Optional, Sequence, Tuple

import serial
from serial import EIGHTBITS, PARITY_NONE, STOPBITS_ONE


FRAME_HEAD = 0xFC
FRAME_END = 0xFD

TYPE_IMU = 0x40
TYPE_AHRS = 0x41

IMU_LEN = 0x38  # 12 float32 values + one int64 timestamp = 56 bytes
AHRS_LEN = 0x30  # 10 float32 values + one int64 timestamp = 48 bytes

GRAVITY_MPS2 = 9.80665


def calc_crc8(data: bytes) -> int:
    """FDILink header CRC-8 (Dallas/Maxim reflected polynomial 0x8C)."""

    crc = 0
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = (crc >> 1) ^ 0x8C if crc & 0x01 else crc >> 1
    return crc & 0xFF


def calc_crc16(data: bytes) -> int:
    """FDILink payload CRC-16/CCITT with initial value zero."""

    crc = 0
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def _vector_norm(values: Sequence[float]) -> float:
    return math.sqrt(sum(float(value) * float(value) for value in values))


def _normalize_quaternion(
    quaternion: Sequence[float],
) -> Optional[Tuple[float, float, float, float]]:
    if len(quaternion) != 4:
        return None
    values = tuple(float(value) for value in quaternion)
    if not all(math.isfinite(value) for value in values):
        return None
    norm = _vector_norm(values)
    if norm <= 1.0e-12:
        return None
    return tuple(value / norm for value in values)  # type: ignore[return-value]


def _cross(a: Sequence[float], b: Sequence[float]) -> Tuple[float, float, float]:
    return (
        float(a[1]) * float(b[2]) - float(a[2]) * float(b[1]),
        float(a[2]) * float(b[0]) - float(a[0]) * float(b[2]),
        float(a[0]) * float(b[1]) - float(a[1]) * float(b[0]),
    )


def _rotate_body_to_world(
    quaternion: Sequence[float], vector: Sequence[float]
) -> Tuple[float, float, float]:
    """Rotate an FRD body vector into the right-handed NED/world frame.

    FDILink supplies the AHRS quaternion as ``[w, x, y, z]``.  This is the
    standard body-to-world quaternion convention used by the reference SDK.
    """

    qw, qx, qy, qz = (float(value) for value in quaternion)
    q_vector = (qx, qy, qz)
    v = (float(vector[0]), float(vector[1]), float(vector[2]))
    twice_cross = tuple(2.0 * value for value in _cross(q_vector, v))
    second_cross = _cross(q_vector, twice_cross)
    return (
        v[0] + qw * twice_cross[0] + second_cross[0],
        v[1] + qw * twice_cross[1] + second_cross[1],
        v[2] + qw * twice_cross[2] + second_cross[2],
    )


def _rotate_world_to_body(
    quaternion: Sequence[float], vector: Sequence[float]
) -> Tuple[float, float, float]:
    qw, qx, qy, qz = (float(value) for value in quaternion)
    return _rotate_body_to_world((qw, -qx, -qy, -qz), vector)


class IMUSDK:
    """Non-blocking DETA40 interface compatible with the sim2real scripts.

    Args:
        port: Serial device path.
        baudrate: Serial baud rate.
        timeout_s: Short serial read timeout used by the worker thread.
        gravity_mps2: Local gravity magnitude removed before integration.
        stationary_calibration_s: Startup time for estimating accelerometer
            bias.  Keep the robot still during this period if possible.  This
            calibration is non-blocking: acceleration and velocity continue to
            update even if calibration cannot finish.  Set to zero to disable
            startup bias calibration.
        accelerometer_rest_sign: Accelerometer convention in the raw FRD frame.
            Use +1 if a stationary sensor measures +g along gravity, -1 if it
            measures -g (specific force), or 0 to detect it automatically.
        accel_lpf_alpha: New-sample weight of the compensated-acceleration LPF.
        velocity_decay_time_s: Time constant of a weak velocity leak used to
            limit long-term IMU-only drift.  Set to zero to disable it.
        max_speed_mps: Norm limit for the IMU-integrated velocity estimate.
        stationary_accel_threshold_mps2: Maximum gravity-compensated
            acceleration norm considered stationary after calibration.
        stationary_gyro_threshold_rad_s: Maximum angular-speed norm considered
            stationary after calibration.
        stationary_hold_s: Required quiet duration before applying a zero
            velocity update.  Set to zero to disable zero-velocity updates.
        stationary_bias_alpha: Online accelerometer-bias correction weight
            while a zero-velocity state is confirmed.

    The class intentionally does not decode or use any navigation/GPS packet.
    """

    def __init__(
        self,
        port: str = "/dev/ttyUSB0",
        baudrate: int = 921600,
        timeout_s: float = 0.02,
        gravity_mps2: float = GRAVITY_MPS2,
        stationary_calibration_s: float = 1.0,
        accel_lpf_alpha: float = 0.25,
        velocity_decay_time_s: float = 20.0,
        max_speed_mps: float = 3.0,
        accelerometer_rest_sign: int = 0,
        stationary_accel_threshold_mps2: float = 0.12,
        stationary_gyro_threshold_rad_s: float = 0.1,
        stationary_hold_s: float = 0.25,
        stationary_bias_alpha: float = 0.02,
    ) -> None:
        if timeout_s <= 0.0:
            raise ValueError("timeout_s must be positive")
        if gravity_mps2 <= 0.0:
            raise ValueError("gravity_mps2 must be positive")
        if stationary_calibration_s < 0.0:
            raise ValueError("stationary_calibration_s must be non-negative")
        if int(accelerometer_rest_sign) not in (-1, 0, 1):
            raise ValueError("accelerometer_rest_sign must be -1, 0, or +1")
        if not 0.0 < accel_lpf_alpha <= 1.0:
            raise ValueError("accel_lpf_alpha must be in (0, 1]")
        if velocity_decay_time_s < 0.0:
            raise ValueError("velocity_decay_time_s must be non-negative")
        if max_speed_mps <= 0.0:
            raise ValueError("max_speed_mps must be positive")
        if stationary_accel_threshold_mps2 <= 0.0:
            raise ValueError("stationary_accel_threshold_mps2 must be positive")
        if stationary_gyro_threshold_rad_s <= 0.0:
            raise ValueError("stationary_gyro_threshold_rad_s must be positive")
        if stationary_hold_s < 0.0:
            raise ValueError("stationary_hold_s must be non-negative")
        if not 0.0 <= stationary_bias_alpha <= 1.0:
            raise ValueError("stationary_bias_alpha must be in [0, 1]")

        self.port = str(port)
        self.baudrate = int(baudrate)
        self.timeout_s = float(timeout_s)
        self.gravity_mps2 = float(gravity_mps2)
        self.stationary_calibration_s = float(stationary_calibration_s)
        self.accelerometer_rest_sign = int(accelerometer_rest_sign)
        self.accel_lpf_alpha = float(accel_lpf_alpha)
        self.velocity_decay_time_s = float(velocity_decay_time_s)
        self.max_speed_mps = float(max_speed_mps)
        self.stationary_accel_threshold_mps2 = float(
            stationary_accel_threshold_mps2
        )
        self.stationary_gyro_threshold_rad_s = float(
            stationary_gyro_threshold_rad_s
        )
        self.stationary_hold_s = float(stationary_hold_s)
        self.stationary_bias_alpha = float(stationary_bias_alpha)

        self.serial: Optional[serial.Serial] = None
        self.thread: Optional[threading.Thread] = None
        self.running = False

        self.lock = threading.Lock()
        self._lifecycle_lock = threading.Lock()
        self._stop_event = threading.Event()

        self.valid_imu = False
        self.valid_ahrs = False

        self._raw_imu: Dict[str, float] = {
            "gyro_x": 0.0,
            "gyro_y": 0.0,
            "gyro_z": 0.0,
            "acc_x": 0.0,
            "acc_y": 0.0,
            "acc_z": 0.0,
            "mag_x": 0.0,
            "mag_y": 0.0,
            "mag_z": 0.0,
            "temperature": 0.0,
            "pressure": 0.0,
            "pressure_temperature": 0.0,
            "timestamp": 0.0,
        }
        self._raw_ahrs: Dict[str, float] = {
            "roll_speed": 0.0,
            "pitch_speed": 0.0,
            "heading_speed": 0.0,
            "roll": 0.0,
            "pitch": 0.0,
            "heading": 0.0,
            "qw": 1.0,
            "qx": 0.0,
            "qy": 0.0,
            "qz": 0.0,
            "timestamp": 0.0,
        }

        # Every derived quantity below remains in the raw right-handed FRD/NED
        # convention.  Y and Z signs are changed only in public return methods.
        self._quaternion_raw = (1.0, 0.0, 0.0, 0.0)
        self._gravity_body_raw = [0.0, 0.0, 1.0]
        self._linear_acceleration_body_raw = [0.0, 0.0, 0.0]
        self._linear_acceleration_world_raw = [0.0, 0.0, 0.0]
        self._filtered_linear_acceleration_body_raw: Optional[List[float]] = None
        self._velocity_world_raw = [0.0, 0.0, 0.0]
        self._last_linear_acceleration_world_raw: Optional[List[float]] = None

        self._accelerometer_bias_body_raw = [0.0, 0.0, 0.0]
        self._detected_accelerometer_rest_sign: Optional[int] = (
            self.accelerometer_rest_sign
            if self.accelerometer_rest_sign != 0
            else None
        )
        self._calibration_sum = [0.0, 0.0, 0.0]
        self._calibration_count = 0
        self._calibration_started_t: Optional[float] = None
        self._calibrated = self.stationary_calibration_s == 0.0

        self._last_device_timestamp_us: Optional[int] = None
        self._last_sample_host_t: Optional[float] = None
        self._last_velocity_dt_s = 0.0
        self._last_imu_host_t = 0.0
        self._last_ahrs_host_t = 0.0
        self._stationary_elapsed_s = 0.0
        self._stationary = False

        self._imu_packet_count = 0
        self._ahrs_packet_count = 0
        self._ignored_packet_count = 0
        self._crc_error_count = 0
        self._frame_error_count = 0

    # ------------------------------ public API ------------------------------
    def start(self) -> bool:
        """Open the port and start the background reader; safe to call twice."""

        with self._lifecycle_lock:
            if self.thread is not None and self.thread.is_alive():
                return True

            try:
                serial_port = serial.Serial(
                    port=self.port,
                    baudrate=self.baudrate,
                    bytesize=EIGHTBITS,
                    parity=PARITY_NONE,
                    stopbits=STOPBITS_ONE,
                    timeout=self.timeout_s,
                )
            except (OSError, serial.SerialException, ValueError) as exc:
                print("IMU SDK (DETA40): failed to open {}: {}".format(self.port, exc))
                return False

            with self.lock:
                self._reset_runtime_state_locked(clear_bias=True)

            self.serial = serial_port
            self._stop_event.clear()
            self.running = True
            try:
                self.thread = threading.Thread(
                    target=self._read_loop,
                    name="deta40-imu-reader",
                    daemon=True,
                )
                self.thread.start()
            except Exception as exc:
                self.running = False
                serial_port.close()
                self.serial = None
                self.thread = None
                print("IMU SDK (DETA40): failed to start reader thread: {}".format(exc))
                return False

            print(
                "IMU SDK (DETA40): started on {} @ {}".format(
                    self.port, self.baudrate
                )
            )
            if self.stationary_calibration_s > 0.0:
                print(
                    "IMU SDK (DETA40): keep the robot still for {:.1f} s "
                    "while accelerometer bias is calibrated".format(
                        self.stationary_calibration_s
                    )
                )
            return True

    def stop(self) -> None:
        """Stop the worker and close the serial port."""

        with self._lifecycle_lock:
            self._stop_event.set()
            self.running = False
            serial_port = self.serial
            worker = self.thread

            if serial_port is not None:
                try:
                    cancel_read = getattr(serial_port, "cancel_read", None)
                    if callable(cancel_read):
                        cancel_read()
                except (OSError, serial.SerialException):
                    pass
                try:
                    if serial_port.is_open:
                        serial_port.close()
                except (OSError, serial.SerialException):
                    pass

            if worker is not None and worker.is_alive() and worker is not threading.current_thread():
                worker.join(timeout=max(1.0, 5.0 * self.timeout_s))

            self.serial = None
            self.thread = None

    def get_imu_data(self) -> Optional[Dict[str, float]]:
        """Return the latest IMU snapshot without waiting.

        Gyroscope, accelerometer, and magnetometer vectors use the requested
        FLU output signs ``[x, -y, -z]``.  Acceleration still includes gravity,
        exactly as reported by the device.  Quaternion fields are converted
        from raw FRD to FLU as ``[w, x, -y, -z]``.
        """

        with self.lock:
            if not (self.valid_imu and self.valid_ahrs):
                return None
            return {
                "gyro_x": float(self._raw_imu["gyro_x"]),
                "gyro_y": -float(self._raw_imu["gyro_y"]),
                "gyro_z": -float(self._raw_imu["gyro_z"]),
                "acc_x": float(self._raw_imu["acc_x"]),
                "acc_y": -float(self._raw_imu["acc_y"]),
                "acc_z": -float(self._raw_imu["acc_z"]),
                "mag_x": float(self._raw_imu["mag_x"]),
                "mag_y": -float(self._raw_imu["mag_y"]),
                "mag_z": -float(self._raw_imu["mag_z"]),
                "temperature": float(self._raw_imu["temperature"]),
                "pressure": float(self._raw_imu["pressure"]),
                "pressure_temperature": float(
                    self._raw_imu["pressure_temperature"]
                ),
                "qw": float(self._quaternion_raw[0]),
                "qx": float(self._quaternion_raw[1]),
                "qy": -float(self._quaternion_raw[2]),
                "qz": -float(self._quaternion_raw[3]),
                "timestamp": float(self._raw_imu["timestamp"]),
            }

    def get_ahrs_data(self) -> Optional[Dict[str, float]]:
        """Return the latest AHRS snapshot without waiting.

        Angular-rate components, Euler angles, and quaternion use FLU signs.
        Internally the raw FRD attitude remains unchanged.
        """

        with self.lock:
            if not self.valid_ahrs:
                return None
            return {
                "roll_speed": float(self._raw_ahrs["roll_speed"]),
                "pitch_speed": -float(self._raw_ahrs["pitch_speed"]),
                "heading_speed": -float(self._raw_ahrs["heading_speed"]),
                "roll": float(self._raw_ahrs["roll"]),
                "pitch": -float(self._raw_ahrs["pitch"]),
                "heading": -float(self._raw_ahrs["heading"]),
                "qw": float(self._quaternion_raw[0]),
                "qx": float(self._quaternion_raw[1]),
                "qy": -float(self._quaternion_raw[2]),
                "qz": -float(self._quaternion_raw[3]),
                "timestamp": float(self._raw_ahrs["timestamp"]),
            }

    def get_linear_velocity(self) -> List[float]:
        """Return gravity-compensated velocity in FLU body axes, in m/s."""

        with self.lock:
            velocity_body_raw = _rotate_world_to_body(
                self._quaternion_raw, self._velocity_world_raw
            )
            return self._to_output_vector(velocity_body_raw)

    def get_gravity_acceleration(self) -> List[float]:
        """Return the unit gravity direction in FLU body axes.

        Because output Z points up, a level IMU returns approximately
        ``[0, 0, -1]``.
        """

        with self.lock:
            return self._to_output_vector(self._gravity_body_raw)

    def reset_linear_velocity(self) -> None:
        """Reset the IMU-only velocity integrator without blocking the reader."""

        with self.lock:
            self._reset_velocity_locked()

    def begin_stationary_calibration(self) -> None:
        """Restart accelerometer-bias calibration while the robot is still."""

        with self.lock:
            self._accelerometer_bias_body_raw = [0.0, 0.0, 0.0]
            self._reset_calibration_locked()
            self._calibrated = self.stationary_calibration_s == 0.0
            self._reset_velocity_locked()

    def get_diagnostics(self) -> Dict[str, object]:
        """Return status useful to the two standalone validation scripts."""

        with self.lock:
            now = time.monotonic()
            velocity_body_raw = _rotate_world_to_body(
                self._quaternion_raw, self._velocity_world_raw
            )
            return {
                "running": bool(self.running),
                "imu_valid": bool(self.valid_imu),
                "ahrs_valid": bool(self.valid_ahrs),
                "calibrated": bool(self._calibrated),
                "imu_age_s": (
                    now - self._last_imu_host_t
                    if self._last_imu_host_t > 0.0
                    else float("inf")
                ),
                "ahrs_age_s": (
                    now - self._last_ahrs_host_t
                    if self._last_ahrs_host_t > 0.0
                    else float("inf")
                ),
                "imu_packet_count": int(self._imu_packet_count),
                "ahrs_packet_count": int(self._ahrs_packet_count),
                "ignored_packet_count": int(self._ignored_packet_count),
                "crc_error_count": int(self._crc_error_count),
                "frame_error_count": int(self._frame_error_count),
                "velocity_dt_s": float(self._last_velocity_dt_s),
                "velocity": self._to_output_vector(velocity_body_raw),
                "gravity_direction": self._to_output_vector(
                    self._gravity_body_raw
                ),
                "linear_acceleration": self._to_output_vector(
                    self._linear_acceleration_body_raw
                ),
                "accelerometer_bias": self._to_output_vector(
                    self._accelerometer_bias_body_raw
                ),
                "calibration_sample_count": int(self._calibration_count),
                "stationary": bool(self._stationary),
                "stationary_elapsed_s": float(self._stationary_elapsed_s),
                "accelerometer_rest_sign": (
                    int(self._detected_accelerometer_rest_sign)
                    if self._detected_accelerometer_rest_sign is not None
                    else 0
                ),
            }

    def __enter__(self) -> "IMUSDK":
        if not self.start():
            raise RuntimeError("failed to start DETA40 IMU")
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.stop()

    # -------------------------- background reader ---------------------------
    def _read_exact(self, size: int) -> bytes:
        serial_port = self.serial
        if serial_port is None or size <= 0:
            return b""

        result = bytearray()
        while len(result) < size and not self._stop_event.is_set():
            chunk = serial_port.read(size - len(result))
            if not chunk:
                break
            result.extend(chunk)
        return bytes(result)

    def _read_loop(self) -> None:
        try:
            while not self._stop_event.is_set():
                serial_port = self.serial
                if serial_port is None or not serial_port.is_open:
                    break

                first_byte = self._read_exact(1)
                if not first_byte:
                    continue
                if first_byte[0] != FRAME_HEAD:
                    continue

                header_rest = self._read_exact(6)
                if len(header_rest) != 6:
                    self._increment_counter("_frame_error_count")
                    continue
                header = first_byte + header_rest
                _, data_type, payload_len, _, header_crc8, crc16_h, crc16_l = struct.unpack(
                    "<7B", header
                )

                if calc_crc8(header[:4]) != header_crc8:
                    self._increment_counter("_crc_error_count")
                    continue

                expected_len = {TYPE_IMU: IMU_LEN, TYPE_AHRS: AHRS_LEN}.get(
                    data_type
                )
                if expected_len is None:
                    # Consume the complete unknown frame to retain byte alignment,
                    # but intentionally do not decode or use it.
                    discarded = self._read_exact(int(payload_len) + 1)
                    if len(discarded) != int(payload_len) + 1:
                        self._increment_counter("_frame_error_count")
                    self._increment_counter("_ignored_packet_count")
                    continue

                if int(payload_len) != expected_len:
                    discarded = self._read_exact(int(payload_len) + 1)
                    if len(discarded) != int(payload_len) + 1:
                        self._increment_counter("_frame_error_count")
                    self._increment_counter("_frame_error_count")
                    continue

                payload_and_end = self._read_exact(expected_len + 1)
                if len(payload_and_end) != expected_len + 1:
                    self._increment_counter("_frame_error_count")
                    continue
                if payload_and_end[-1] != FRAME_END:
                    self._increment_counter("_frame_error_count")
                    continue

                payload = payload_and_end[:-1]
                expected_crc16 = (int(crc16_h) << 8) | int(crc16_l)
                if calc_crc16(payload) != expected_crc16:
                    self._increment_counter("_crc_error_count")
                    continue

                self._parse_payload(data_type, payload, time.monotonic())
        except (OSError, serial.SerialException) as exc:
            if not self._stop_event.is_set():
                print("IMU SDK (DETA40): serial reader stopped: {}".format(exc))
        except Exception as exc:
            if not self._stop_event.is_set():
                print("IMU SDK (DETA40): unexpected reader error: {}".format(exc))
        finally:
            self.running = False

    def _parse_payload(
        self, data_type: int, payload: bytes, arrival_t: Optional[float] = None
    ) -> None:
        """Decode one already-validated payload in the reader thread."""

        host_t = time.monotonic() if arrival_t is None else float(arrival_t)

        if data_type == TYPE_IMU:
            if len(payload) != IMU_LEN:
                return
            values = struct.unpack("<12fq", payload)
            if not all(math.isfinite(float(value)) for value in values[:12]):
                return

            with self.lock:
                names = (
                    "gyro_x",
                    "gyro_y",
                    "gyro_z",
                    "acc_x",
                    "acc_y",
                    "acc_z",
                    "mag_x",
                    "mag_y",
                    "mag_z",
                    "temperature",
                    "pressure",
                    "pressure_temperature",
                )
                for name, value in zip(names, values[:12]):
                    self._raw_imu[name] = float(value)
                self._raw_imu["timestamp"] = float(values[12])
                self.valid_imu = True
                self._last_imu_host_t = host_t
                self._imu_packet_count += 1

                if self.valid_ahrs:
                    self._update_velocity_and_acceleration_locked(
                        int(values[12]), host_t
                    )

        elif data_type == TYPE_AHRS:
            if len(payload) != AHRS_LEN:
                return
            values = struct.unpack("<10fq", payload)
            if not all(math.isfinite(float(value)) for value in values[:10]):
                return
            quaternion = _normalize_quaternion(values[6:10])
            if quaternion is None:
                return

            with self.lock:
                names = (
                    "roll_speed",
                    "pitch_speed",
                    "heading_speed",
                    "roll",
                    "pitch",
                    "heading",
                    "qw",
                    "qx",
                    "qy",
                    "qz",
                )
                for name, value in zip(names, values[:10]):
                    self._raw_ahrs[name] = float(value)
                self._raw_ahrs["timestamp"] = float(values[10])
                self._quaternion_raw = quaternion
                self.valid_ahrs = True
                self._last_ahrs_host_t = host_t
                self._ahrs_packet_count += 1
                self._update_gravity_locked()

    # ----------------------------- derived data -----------------------------
    @staticmethod
    def _to_output_vector(values: Sequence[float]) -> List[float]:
        # This is the only raw-FRD -> requested-FLU vector sign conversion.
        return [float(values[0]), -float(values[1]), -float(values[2])]

    def _update_gravity_locked(self) -> None:
        # World/NED gravity points along +Z because the raw frame uses Z down.
        gravity = _rotate_world_to_body(
            self._quaternion_raw, (0.0, 0.0, 1.0)
        )
        norm = _vector_norm(gravity)
        if norm <= 1.0e-12 or not math.isfinite(norm):
            self._gravity_body_raw = [0.0, 0.0, 1.0]
        else:
            self._gravity_body_raw = [value / norm for value in gravity]

    def _update_velocity_and_acceleration_locked(
        self, device_timestamp_us: int, host_t: float
    ) -> None:
        self._update_gravity_locked()

        measured_acceleration = (
            float(self._raw_imu["acc_x"]),
            float(self._raw_imu["acc_y"]),
            float(self._raw_imu["acc_z"]),
        )
        gyro = (
            float(self._raw_imu["gyro_x"]),
            float(self._raw_imu["gyro_y"]),
            float(self._raw_imu["gyro_z"]),
        )

        # Determine whether this DETA40 reports +g or -g at rest, then remove
        # that attitude-dependent component.  Many accelerometers report
        # specific force (-g in the raw Z-down frame), while some configured
        # outputs report acceleration including +g.  Auto detection handles
        # both without changing any internal coordinate convention.
        acceleration_without_gravity = self._remove_gravity_locked(
            measured_acceleration
        )

        dt = self._compute_sample_dt_locked(device_timestamp_us, host_t)

        if not self._calibrated:
            calibration_completed = self._update_calibration_locked(
                acceleration_without_gravity, gyro, host_t
            )
            if calibration_completed:
                # Calibration completed while stationary and reset the
                # integrator.  Start from a clean timestamp on the next sample.
                self._linear_acceleration_body_raw = [0.0, 0.0, 0.0]
                self._linear_acceleration_world_raw = [0.0, 0.0, 0.0]
                return

        unbiased_acceleration = [
            acceleration_without_gravity[index]
            - self._accelerometer_bias_body_raw[index]
            for index in range(3)
        ]
        if self._filtered_linear_acceleration_body_raw is None:
            filtered_acceleration = list(unbiased_acceleration)
        else:
            alpha = self.accel_lpf_alpha
            filtered_acceleration = [
                alpha * unbiased_acceleration[index]
                + (1.0 - alpha)
                * self._filtered_linear_acceleration_body_raw[index]
                for index in range(3)
            ]
        self._filtered_linear_acceleration_body_raw = list(filtered_acceleration)

        if self._update_stationary_state_locked(
            acceleration_without_gravity,
            filtered_acceleration,
            gyro,
            dt,
        ):
            return

        # Remove tiny residual noise without treating zero acceleration as zero
        # velocity (an IMU alone cannot make that distinction).
        if _vector_norm(filtered_acceleration) < 0.03:
            filtered_acceleration = [0.0, 0.0, 0.0]

        self._linear_acceleration_body_raw = list(filtered_acceleration)
        acceleration_world = _rotate_body_to_world(
            self._quaternion_raw, filtered_acceleration
        )
        self._linear_acceleration_world_raw = list(acceleration_world)

        if dt is None:
            self._last_linear_acceleration_world_raw = list(acceleration_world)
            return

        previous_acceleration = self._last_linear_acceleration_world_raw
        if previous_acceleration is None:
            previous_acceleration = list(acceleration_world)

        decay = 1.0
        if self.velocity_decay_time_s > 0.0:
            decay = math.exp(-dt / self.velocity_decay_time_s)

        self._velocity_world_raw = [
            decay * self._velocity_world_raw[index]
            + 0.5
            * (previous_acceleration[index] + acceleration_world[index])
            * dt
            for index in range(3)
        ]
        self._last_linear_acceleration_world_raw = list(acceleration_world)

        speed = _vector_norm(self._velocity_world_raw)
        if speed > self.max_speed_mps:
            scale = self.max_speed_mps / speed
            self._velocity_world_raw = [
                value * scale for value in self._velocity_world_raw
            ]

    def _update_stationary_state_locked(
        self,
        acceleration_without_gravity: Sequence[float],
        filtered_acceleration: Sequence[float],
        gyro: Sequence[float],
        dt: Optional[float],
    ) -> bool:
        """Apply a zero-velocity update after a confirmed quiet interval."""

        stationary_candidate = bool(
            self._calibrated
            and self.stationary_hold_s > 0.0
            and _vector_norm(filtered_acceleration)
            <= self.stationary_accel_threshold_mps2
            and _vector_norm(gyro) <= self.stationary_gyro_threshold_rad_s
        )

        if not stationary_candidate:
            self._stationary_elapsed_s = 0.0
            self._stationary = False
            return False

        if dt is not None:
            self._stationary_elapsed_s += dt
        if self._stationary_elapsed_s < self.stationary_hold_s:
            self._stationary = False
            return False

        self._stationary = True

        # Track slow thermal/bias changes only after stationary status has been
        # held long enough.  This removes the persistent few cm/s^2 residual
        # that would otherwise integrate into a large velocity drift.
        alpha = self.stationary_bias_alpha
        if alpha > 0.0:
            self._accelerometer_bias_body_raw = [
                (1.0 - alpha) * self._accelerometer_bias_body_raw[index]
                + alpha * float(acceleration_without_gravity[index])
                for index in range(3)
            ]

        self._velocity_world_raw = [0.0, 0.0, 0.0]
        self._linear_acceleration_body_raw = [0.0, 0.0, 0.0]
        self._linear_acceleration_world_raw = [0.0, 0.0, 0.0]
        self._filtered_linear_acceleration_body_raw = [0.0, 0.0, 0.0]
        self._last_linear_acceleration_world_raw = [0.0, 0.0, 0.0]
        return True

    def _remove_gravity_locked(
        self, measured_acceleration: Sequence[float]
    ) -> List[float]:
        if self._detected_accelerometer_rest_sign is None:
            alignment = sum(
                float(measured_acceleration[index])
                * self._gravity_body_raw[index]
                for index in range(3)
            )
            # Gravity dominates ordinary hand/robot acceleration, so the sign
            # of this projection is a reliable startup discriminator.  Delay
            # the decision only for free-fall-like samples near zero.
            if abs(alignment) >= 0.25 * self.gravity_mps2:
                self._detected_accelerometer_rest_sign = (
                    1 if alignment >= 0.0 else -1
                )
                convention = (
                    "+g along gravity"
                    if self._detected_accelerometer_rest_sign > 0
                    else "-g specific force"
                )
                print(
                    "IMU SDK (DETA40): detected accelerometer convention: {}".format(
                        convention
                    )
                )

        rest_sign = self._detected_accelerometer_rest_sign
        if rest_sign is None:
            # This is used only until a non-free-fall sample establishes the
            # sign.  It avoids withholding acceleration/velocity updates.
            rest_sign = 1

        return [
            float(measured_acceleration[index])
            - rest_sign * self.gravity_mps2 * self._gravity_body_raw[index]
            for index in range(3)
        ]

    def _compute_sample_dt_locked(
        self, device_timestamp_us: int, host_t: float
    ) -> Optional[float]:
        device_dt: Optional[float] = None
        host_dt: Optional[float] = None

        if self._last_device_timestamp_us is not None:
            device_dt = (
                int(device_timestamp_us) - self._last_device_timestamp_us
            ) * 1.0e-6
        if self._last_sample_host_t is not None:
            host_dt = float(host_t) - self._last_sample_host_t

        self._last_device_timestamp_us = int(device_timestamp_us)
        self._last_sample_host_t = float(host_t)

        for candidate in (device_dt, host_dt):
            if candidate is not None and 1.0e-5 <= candidate <= 0.2:
                self._last_velocity_dt_s = float(candidate)
                return float(candidate)

        self._last_velocity_dt_s = 0.0
        return None

    def _update_calibration_locked(
        self,
        acceleration_without_gravity: Sequence[float],
        gyro: Sequence[float],
        host_t: float,
    ) -> bool:
        # Reject movement during startup calibration and restart the quiet
        # window.  The generous acceleration bound tolerates mounting offsets.
        if (
            _vector_norm(gyro) > 0.1
            or _vector_norm(acceleration_without_gravity) > 2.0
        ):
            self._reset_calibration_locked()
            return False

        if self._calibration_started_t is None:
            self._calibration_started_t = float(host_t)

        for index in range(3):
            self._calibration_sum[index] += float(
                acceleration_without_gravity[index]
            )
        self._calibration_count += 1

        elapsed = float(host_t) - self._calibration_started_t
        if elapsed < self.stationary_calibration_s or self._calibration_count < 20:
            return False

        self._accelerometer_bias_body_raw = [
            value / float(self._calibration_count)
            for value in self._calibration_sum
        ]
        self._calibrated = True
        self._reset_velocity_locked()
        print(
            "IMU SDK (DETA40): accelerometer calibration complete "
            "({} samples)".format(self._calibration_count)
        )
        return True

    def _reset_velocity_locked(self) -> None:
        self._velocity_world_raw = [0.0, 0.0, 0.0]
        self._linear_acceleration_body_raw = [0.0, 0.0, 0.0]
        self._linear_acceleration_world_raw = [0.0, 0.0, 0.0]
        self._filtered_linear_acceleration_body_raw = None
        self._last_linear_acceleration_world_raw = None
        self._last_device_timestamp_us = None
        self._last_sample_host_t = None
        self._last_velocity_dt_s = 0.0
        self._stationary_elapsed_s = 0.0
        self._stationary = False

    def _reset_calibration_locked(self) -> None:
        self._calibration_sum = [0.0, 0.0, 0.0]
        self._calibration_count = 0
        self._calibration_started_t = None

    def _reset_runtime_state_locked(self, clear_bias: bool) -> None:
        self.valid_imu = False
        self.valid_ahrs = False
        self._quaternion_raw = (1.0, 0.0, 0.0, 0.0)
        self._gravity_body_raw = [0.0, 0.0, 1.0]
        self._detected_accelerometer_rest_sign = (
            self.accelerometer_rest_sign
            if self.accelerometer_rest_sign != 0
            else None
        )
        self._last_imu_host_t = 0.0
        self._last_ahrs_host_t = 0.0
        self._imu_packet_count = 0
        self._ahrs_packet_count = 0
        self._ignored_packet_count = 0
        self._crc_error_count = 0
        self._frame_error_count = 0
        if clear_bias:
            self._accelerometer_bias_body_raw = [0.0, 0.0, 0.0]
        self._reset_calibration_locked()
        self._calibrated = self.stationary_calibration_s == 0.0
        self._reset_velocity_locked()

    def _increment_counter(self, attribute: str) -> None:
        with self.lock:
            setattr(self, attribute, int(getattr(self, attribute)) + 1)


__all__ = ["IMUSDK", "calc_crc8", "calc_crc16"]
