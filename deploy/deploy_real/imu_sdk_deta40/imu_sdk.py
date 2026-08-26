import math
import struct
import threading
import time
from collections import deque
from typing import Dict, Optional, Tuple

import serial

# ================= Protocol Constants (FDILink) =================
FRAME_HEAD = 0xFC
FRAME_END = 0xFD

TYPE_IMU = 0x40
TYPE_AHRS = 0x41
TYPE_INSGPS = 0x42

IMU_LEN = 0x38  # 56 bytes
AHRS_LEN = 0x30  # 48 bytes
INSGPS_LEN = 0x48  # 72 bytes

# First-order low-pass filter applied to IMU measurements and world-frame
# gravity-compensated acceleration.
# filtered = alpha * current + (1 - alpha) * previous_filtered
IMU_LPF_ALPHA = 0.8

GRAVITY_ACCELERATION = 9.81
VELOCITY_DT_MIN_S = 0.0001
VELOCITY_DT_MAX_S = 0.05
MAX_ESTIMATED_SPEED_MPS = 2.0
MAX_AHRS_AGE_S = 0.05
MAX_PENDING_IMU_SAMPLES = 256
BIAS_CALIBRATION_WINDOW_SAMPLES = 200
BIAS_CALIBRATION_MIN_SAMPLES = 20
BIAS_CALIBRATION_MAX_ACCEL_MPS2 = 0.75
BIAS_CALIBRATION_MAX_GYRO_RAD_S = 0.15


# ================= CRC Tables (from official C++ crc_table.cpp) =================
CRC8Table = [
    0, 94, 188, 226, 97, 63, 221, 131, 194, 156, 126, 32, 163, 253, 31, 65,
    157, 195, 33, 127, 252, 162, 64, 30, 95, 1, 227, 189, 62, 96, 130, 220,
    35, 125, 159, 193, 66, 28, 254, 160, 225, 191, 93, 3, 128, 222, 60, 98,
    190, 224, 2, 92, 223, 129, 99, 61, 124, 34, 192, 158, 29, 67, 161, 255,
    70, 24, 250, 164, 39, 121, 155, 197, 132, 218, 56, 102, 229, 187, 89, 7,
    219, 133, 103, 57, 186, 228, 6, 88, 25, 71, 165, 251, 120, 38, 196, 154,
    101, 59, 217, 135, 4, 90, 184, 230, 167, 249, 27, 69, 198, 152, 122, 36,
    248, 166, 68, 26, 153, 199, 37, 123, 58, 100, 134, 216, 91, 5, 231, 185,
    140, 210, 48, 110, 237, 179, 81, 15, 78, 16, 242, 172, 47, 113, 147, 205,
    17, 79, 173, 243, 112, 46, 204, 146, 211, 141, 111, 49, 178, 236, 14, 80,
    175, 241, 19, 77, 206, 144, 114, 44, 109, 51, 209, 143, 12, 82, 176, 238,
    50, 108, 142, 208, 83, 13, 239, 177, 240, 174, 76, 18, 145, 207, 45, 115,
    202, 148, 118, 40, 171, 245, 23, 73, 8, 86, 180, 234, 105, 55, 213, 139,
    87, 9, 235, 181, 54, 104, 138, 212, 149, 203, 41, 119, 244, 170, 72, 22,
    233, 183, 85, 11, 136, 214, 52, 106, 43, 117, 151, 201, 74, 20, 246, 168,
    116, 42, 200, 150, 21, 75, 169, 247, 182, 232, 10, 84, 215, 137, 107, 53,
]

CRC16Table = [
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
    0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
    0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
    0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
    0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
    0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
    0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
    0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
    0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
    0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
    0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
    0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
    0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
    0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
    0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
    0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
    0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
    0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
    0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
    0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
    0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
    0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
    0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0,
]


def calc_crc8(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc = CRC8Table[crc ^ byte]
    return crc


def calc_crc16(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc = (CRC16Table[((crc >> 8) ^ byte) & 0xFF] ^ (crc << 8)) & 0xFFFF
    return crc


def _normalize_quat(q: Tuple[float, float, float, float]) -> Tuple[float, float, float, float]:
    qw, qx, qy, qz = q
    n2 = qw * qw + qx * qx + qy * qy + qz * qz
    if not math.isfinite(n2) or n2 <= 1e-12:
        return (1.0, 0.0, 0.0, 0.0)
    inv = (n2 ** -0.5)
    return (qw * inv, qx * inv, qy * inv, qz * inv)


class IMUSDK:
    """FDILink IMU SDK wrapper for DETA40.

    Public API is kept identical to the existing sim2real `imu_sdk` module:
      - start()/stop()
      - get_linear_velocity()
      - get_gravity_acceleration()
      - get_imu_data()  (gyro/acc/quaternion)

    Axis convention (same as old implementation):
      - Raw sensor frame: X forward, Y right, Z down
      - Policy/body(URDF) frame: X forward, Y left, Z up
        => (x, y, z)_urdf = (x, -y, -z)_imu
    """

    def __init__(self, port: str = "/dev/ttyUSB0", baudrate: int = 921600, timeout_s: float = 0.02):
        self.port = port
        self.baudrate = int(baudrate)
        self.timeout_s = float(timeout_s)

        self.serial: Optional[serial.Serial] = None
        self.running = False
        self.thread: Optional[threading.Thread] = None
        self.lock = threading.Lock()

        self.valid_imu = False
        self.valid_ahrs = False
        self.valid_insgps = False

        # Latest decoded raw packets (sensor frame)
        self._raw_imu: Dict[str, float] = {
            "gyro_x": 0.0,
            "gyro_y": 0.0,
            "gyro_z": 0.0,
            "acc_x": 0.0,
            "acc_y": 0.0,
            "acc_z": 0.0,
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
        self._raw_insgps: Dict[str, float] = {
            "body_vel_x": 0.0,
            "body_vel_y": 0.0,
            "body_vel_z": 0.0,
            "body_acc_x": 0.0,
            "body_acc_y": 0.0,
            "body_acc_z": 0.0,
            "timestamp": 0.0,
        }

        # URDF/body-frame state
        self._quat_urdf: Tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)
        self._gyro_urdf = [0.0, 0.0, 0.0]
        self._acc_urdf = [0.0, 0.0, 0.0]
        self._filtered_gyro_urdf = None
        self._filtered_acc_urdf = None

        self._gravity_vec = [0.0, 0.0, -1.0]

        # Velocity sources
        self._vel_from_insgps = [0.0, 0.0, 0.0]
        self._filtered_vel_from_insgps = None
        self._last_insgps_t = 0.0

        # IMU-only velocity estimation. Acceleration and velocity are kept in
        # the non-rotating world frame; get_linear_velocity() converts the
        # result to the latest policy/body frame.
        self._specific_force_urdf = [0.0, 0.0, 0.0]
        self._linear_acceleration_world = [0.0, 0.0, 0.0]
        self._filtered_linear_acceleration_world = None
        self._estimated_velocity_world = [0.0, 0.0, 0.0]
        self._estimated_velocity = [0.0, 0.0, 0.0]
        self._last_acceleration_world = None
        self._last_velocity_sample_time_s = None
        self._last_velocity_device_timestamp_us = None
        self._velocity_time_source = None
        self._last_velocity_dt_s = 0.0
        self._last_velocity_update_host_t = 0.0
        self._velocity_update_count = 0
        self._velocity_rejected_sample_count = 0
        self._last_imu_arrival_t = 0.0
        self._last_ahrs_arrival_t = 0.0
        self._last_imu_ahrs_delta_s = float("inf")
        self._ahrs_history = deque(maxlen=2)
        self._pending_imu_samples = deque()

        # Body-frame accelerometer bias is estimated only when an external
        # caller confirms the robot is stationary via reset_linear_velocity().
        self._accelerometer_bias_urdf = [0.0, 0.0, 0.0]
        self._quiet_bias_candidates = deque(
            maxlen=BIAS_CALIBRATION_WINDOW_SAMPLES
        )
        self._last_bias_calibration_sample_count = 0

    # -------------------- Public API --------------------
    def start(self) -> bool:
        try:
            self.serial = serial.Serial(self.port, self.baudrate, timeout=self.timeout_s)
            self.running = True
            self.thread = threading.Thread(target=self._read_loop, daemon=True)
            self.thread.start()
            print(f"IMU SDK (deta40): started on {self.port} @ {self.baudrate}")
            return True
        except Exception as e:
            print(f"IMU SDK (deta40): failed to open serial port: {e}")
            return False

    def stop(self) -> None:
        self.running = False
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=1.0)
        if self.serial and self.serial.is_open:
            try:
                self.serial.close()
            except Exception:
                pass

    def get_imu_data(self) -> Optional[Dict[str, float]]:
        with self.lock:
            if not (self.valid_imu and self.valid_ahrs):
                return None
            return {
                "gyro_x": float(self._gyro_urdf[0]),
                "gyro_y": float(self._gyro_urdf[1]),
                "gyro_z": float(self._gyro_urdf[2]),
                "acc_x": float(self._acc_urdf[0]),
                "acc_y": float(self._acc_urdf[1]),
                "acc_z": float(self._acc_urdf[2]),
                "qw": float(self._quat_urdf[0]),
                "qx": float(self._quat_urdf[1]),
                "qy": float(self._quat_urdf[2]),
                "qz": float(self._quat_urdf[3]),
                "timestamp": float(self._raw_imu.get("timestamp", 0.0)),
            }

    def get_ahrs_data(self) -> Optional[Dict[str, float]]:
        with self.lock:
            if not self.valid_ahrs:
                return None
            return dict(self._raw_ahrs)

    def get_linear_velocity(self):
        """IMU-integrated base velocity [vx, vy, vz] in body frame (m/s)."""
        with self.lock:
            # INSGPS is decoded only for diagnostics and never reaches the policy.
            self._estimated_velocity = list(
                self._rotate_world_to_body(
                    self._quat_urdf,
                    self._estimated_velocity_world,
                )
            )
            return [float(v) for v in self._estimated_velocity]

    def reset_linear_velocity(self) -> bool:
        """Reset the IMU-only velocity integrator to zero.

        Call this only when the robot is known to be stationary. An IMU alone
        cannot distinguish rest from constant-velocity translation.

        Returns True when enough recent quiet IMU samples were available to
        refresh the body-frame accelerometer bias estimate.
        """
        with self.lock:
            bias_calibrated = self._calibrate_accelerometer_bias_locked()
            self._reset_velocity_estimation_locked()
            return bias_calibrated

    def begin_stationary_calibration(self) -> None:
        """Discard old bias candidates before a known stationary hold."""
        with self.lock:
            self._quiet_bias_candidates.clear()
            self._last_bias_calibration_sample_count = 0

    def get_velocity_diagnostics(self) -> Dict[str, object]:
        """Return a read-only snapshot of velocity selection and compensation.

        INSGPS is reported only as a comparison signal. ``selected_velocity`` is
        always the IMU-integrated velocity returned by :meth:`get_linear_velocity`.
        """
        with self.lock:
            now = time.monotonic()
            insgps_age_s = (
                max(0.0, now - self._last_insgps_t)
                if self.valid_insgps
                else float("inf")
            )
            insgps_fresh = self.valid_insgps and insgps_age_s < 0.5
            ahrs_age_s = (
                max(0.0, now - self._last_ahrs_arrival_t)
                if self.valid_ahrs
                else float("inf")
            )
            ahrs_fresh = self.valid_ahrs and ahrs_age_s <= MAX_AHRS_AGE_S
            imu_age_s = (
                max(0.0, now - self._last_imu_arrival_t)
                if self.valid_imu
                else float("inf")
            )
            imu_fresh = self.valid_imu and imu_age_s <= MAX_AHRS_AGE_S
            velocity_update_age_s = (
                max(0.0, now - self._last_velocity_update_host_t)
                if self._last_velocity_update_host_t > 0.0
                else float("inf")
            )
            integrated_velocity = list(
                self._rotate_world_to_body(
                    self._quat_urdf,
                    self._estimated_velocity_world,
                )
            )
            compensated_acceleration = list(
                self._rotate_world_to_body(
                    self._quat_urdf,
                    self._linear_acceleration_world,
                )
            )
            return {
                "source": "imu_integration",
                "imu_valid": bool(self.valid_imu),
                "imu_fresh": bool(imu_fresh),
                "imu_age_s": float(imu_age_s),
                "ahrs_valid": bool(self.valid_ahrs),
                "ahrs_fresh": bool(ahrs_fresh),
                "ahrs_age_s": float(ahrs_age_s),
                "imu_ahrs_timestamp_delta_s": float(
                    self._last_imu_ahrs_delta_s
                ),
                "insgps_valid": bool(self.valid_insgps),
                "insgps_fresh": bool(insgps_fresh),
                "insgps_age_s": float(insgps_age_s),
                "insgps_velocity": [float(v) for v in self._vel_from_insgps],
                "imu_integrated_velocity": [
                    float(v) for v in integrated_velocity
                ],
                "imu_integrated_velocity_world": [
                    float(v) for v in self._estimated_velocity_world
                ],
                # Backward-compatible diagnostic name.
                "fallback_velocity": [float(v) for v in integrated_velocity],
                "selected_velocity": [float(v) for v in integrated_velocity],
                "compensated_acceleration": compensated_acceleration,
                "compensated_acceleration_world": [
                    float(v) for v in self._linear_acceleration_world
                ],
                "velocity_time_source": self._velocity_time_source,
                "velocity_update_age_s": float(velocity_update_age_s),
                "velocity_estimator_valid": bool(
                    imu_fresh
                    and ahrs_fresh
                    and self._velocity_time_source is not None
                    and velocity_update_age_s <= MAX_AHRS_AGE_S
                ),
                "velocity_dt_s": float(self._last_velocity_dt_s),
                "velocity_update_count": int(self._velocity_update_count),
                "velocity_rejected_sample_count": int(
                    self._velocity_rejected_sample_count
                ),
                "pending_imu_sample_count": len(self._pending_imu_samples),
                "accelerometer_bias_body": [
                    float(v) for v in self._accelerometer_bias_urdf
                ],
                "bias_calibration_sample_count": int(
                    self._last_bias_calibration_sample_count
                ),
            }

    def get_gravity_acceleration(self):
        """Gravity direction unit vector in body frame (compatible with sim2sim gravity obs)."""
        with self.lock:
            return list(self._gravity_vec)

    # -------------------- Internal: decoding --------------------
    def _read_exact(self, n: int) -> bytes:
        if not self.serial:
            return b""
        data = self.serial.read(n)
        return data if data is not None else b""

    def _read_loop(self) -> None:
        while self.running:
            if not self.serial or not self.serial.is_open:
                time.sleep(0.1)
                continue

            try:
                b = self._read_exact(1)
                if not b:
                    continue
                if b[0] != FRAME_HEAD:
                    continue

                header_rest = self._read_exact(6)
                if len(header_rest) != 6:
                    continue
                header = b + header_rest

                # header: start,type,len,sn,crc8,crc16_h,crc16_l
                _, h_type, h_len, _, h_crc8, h_crc16_h, h_crc16_l = struct.unpack("7B", header)
                if calc_crc8(header[:4]) != h_crc8:
                    continue

                expected_crc16 = (h_crc16_h << 8) | h_crc16_l
                payload_len = int(h_len)
                data_bytes = self._read_exact(payload_len + 1)  # payload + FRAME_END
                if len(data_bytes) != payload_len + 1:
                    continue

                payload = data_bytes[:-1]
                if data_bytes[-1] != FRAME_END:
                    continue
                if calc_crc16(payload) != expected_crc16:
                    continue

                self._parse_payload(h_type, payload)

            except serial.SerialException as e:
                print(f"IMU SDK (deta40): serial error: {e}")
                time.sleep(1.0)
            except Exception as e:
                print(f"IMU SDK (deta40): unexpected error: {e}")
                time.sleep(0.2)

    def _parse_payload(self, data_type: int, payload: bytes) -> None:
        with self.lock:
            if data_type == TYPE_IMU:
                if len(payload) != IMU_LEN:
                    return

                # payload: 12 floats + 1 int64 timestamp
                vals = struct.unpack("<12fq", payload)
                self._raw_imu["gyro_x"] = float(vals[0])
                self._raw_imu["gyro_y"] = float(vals[1])
                self._raw_imu["gyro_z"] = float(vals[2])
                self._raw_imu["acc_x"] = float(vals[3])
                self._raw_imu["acc_y"] = float(vals[4])
                self._raw_imu["acc_z"] = float(vals[5])
                self._raw_imu["timestamp"] = int(vals[12])

                imu_values = vals[:6]
                if not all(math.isfinite(float(v)) for v in imu_values):
                    return

                self.valid_imu = True
                self._last_imu_arrival_t = time.monotonic()
                self._update_imu_measurement_state_locked()
                imu_timestamp_us = int(vals[12])
                if imu_timestamp_us > 0:
                    self._queue_imu_sample_locked(imu_timestamp_us)
                    self._drain_aligned_imu_samples_locked()
                elif self._attitude_is_fresh_for_imu_locked(imu_timestamp_us):
                    # Old firmware without a usable device timestamp cannot be
                    # interpolated. Use the latest fresh AHRS and host time.
                    self._update_velocity_estimation_locked(
                        imu_timestamp_us,
                        self._specific_force_urdf,
                        self._quat_urdf,
                        self._gyro_urdf,
                    )
                else:
                    self._velocity_rejected_sample_count += 1

            elif data_type == TYPE_AHRS:
                if len(payload) != AHRS_LEN:
                    return

                # payload: 10 floats + 1 int64 timestamp
                vals = struct.unpack("<10fq", payload)
                self._raw_ahrs["roll_speed"] = float(vals[0])
                self._raw_ahrs["pitch_speed"] = float(vals[1])
                self._raw_ahrs["heading_speed"] = float(vals[2])
                self._raw_ahrs["roll"] = float(vals[3])
                self._raw_ahrs["pitch"] = float(vals[4])
                self._raw_ahrs["heading"] = float(vals[5])
                self._raw_ahrs["qw"] = float(vals[6])
                self._raw_ahrs["qx"] = float(vals[7])
                self._raw_ahrs["qy"] = float(vals[8])
                self._raw_ahrs["qz"] = float(vals[9])
                self._raw_ahrs["timestamp"] = int(vals[10])

                quaternion = (float(vals[6]), float(vals[7]), float(vals[8]), float(vals[9]))
                quaternion_norm_squared = sum(v * v for v in quaternion)
                if (
                    not all(math.isfinite(v) for v in quaternion)
                    or quaternion_norm_squared <= 1e-12
                ):
                    return

                self.valid_ahrs = True
                self._last_ahrs_arrival_t = time.monotonic()
                # AHRS packets refresh orientation and the body-frame view of
                # velocity, but must never integrate the previous IMU sample.
                self._update_orientation_state_locked()
                self._append_ahrs_sample_locked(int(vals[10]))
                self._drain_aligned_imu_samples_locked()

            elif data_type == TYPE_INSGPS:
                if len(payload) != INSGPS_LEN:
                    return

                # Official demo reads 0x48 bytes, but only documents the first fields.
                # We parse the first 12 floats (48 bytes) and ignore the remaining bytes.
                f = struct.unpack("<12f", payload[:48])
                self._raw_insgps["body_vel_x"] = float(f[0])
                self._raw_insgps["body_vel_y"] = float(f[1])
                self._raw_insgps["body_vel_z"] = float(f[2])
                self._raw_insgps["body_acc_x"] = float(f[3])
                self._raw_insgps["body_acc_y"] = float(f[4])
                self._raw_insgps["body_acc_z"] = float(f[5])
                self._raw_insgps["timestamp"] = time.time()

                # Coordinate transform IMU->URDF: (x, y, z) -> (x, -y, -z)
                vx = float(self._raw_insgps["body_vel_x"])
                vy = -float(self._raw_insgps["body_vel_y"])
                vz = -float(self._raw_insgps["body_vel_z"])
                self._vel_from_insgps = self._lpf_vector(
                    [vx, vy, vz],
                    "_filtered_vel_from_insgps",
                )
                self._last_insgps_t = time.monotonic()
                self.valid_insgps = True

    # -------------------- Internal: math --------------------
    def _lpf_vector(self, values, state_attr: str):
        current = [float(v) for v in values]
        previous = getattr(self, state_attr)
        if previous is None:
            filtered = current
        else:
            alpha = float(IMU_LPF_ALPHA)
            filtered = [
                alpha * current[i] + (1.0 - alpha) * previous[i]
                for i in range(3)
            ]
        setattr(self, state_attr, filtered)
        return list(filtered)

    def _update_orientation_state_locked(self) -> None:
        """Update quaternion/gravity without reusing an old IMU sample."""
        qw = float(self._raw_ahrs["qw"])
        qx = float(self._raw_ahrs["qx"])
        qy = -float(self._raw_ahrs["qy"])
        qz = -float(self._raw_ahrs["qz"])
        self._quat_urdf = _normalize_quat((qw, qx, qy, qz))

        self._gravity_vec = list(self._compute_gravity_vec_for_obs(self._quat_urdf))
        self._estimated_velocity = list(
            self._rotate_world_to_body(
                self._quat_urdf,
                self._estimated_velocity_world,
            )
        )

    def _update_imu_measurement_state_locked(self) -> None:
        """Transform/filter one newly received IMU packet."""
        gx = float(self._raw_imu["gyro_x"])
        gy = -float(self._raw_imu["gyro_y"])
        gz = -float(self._raw_imu["gyro_z"])
        self._gyro_urdf = self._lpf_vector([gx, gy, gz], "_filtered_gyro_urdf")

        ax = float(self._raw_imu["acc_x"])
        ay = -float(self._raw_imu["acc_y"])
        az = -float(self._raw_imu["acc_z"])
        self._specific_force_urdf = [ax, ay, az]
        self._acc_urdf = self._lpf_vector([ax, ay, az], "_filtered_acc_urdf")

    def _attitude_is_fresh_for_imu_locked(self, imu_timestamp_us: int) -> bool:
        """Reject integration when the latest orientation is stale."""
        if not self.valid_ahrs:
            self._last_imu_ahrs_delta_s = float("inf")
            return False

        host_age_s = time.monotonic() - self._last_ahrs_arrival_t
        if host_age_s < 0.0 or host_age_s > MAX_AHRS_AGE_S:
            self._last_imu_ahrs_delta_s = float("inf")
            return False

        ahrs_timestamp_us = int(self._raw_ahrs.get("timestamp", 0))
        if int(imu_timestamp_us) > 0 and ahrs_timestamp_us > 0:
            timestamp_delta_s = (
                int(imu_timestamp_us) - ahrs_timestamp_us
            ) * 1e-6
            self._last_imu_ahrs_delta_s = float(timestamp_delta_s)
            if abs(timestamp_delta_s) > MAX_AHRS_AGE_S:
                return False
        else:
            self._last_imu_ahrs_delta_s = float(host_age_s)

        return True

    def _append_ahrs_sample_locked(self, timestamp_us: int) -> None:
        """Store a timestamped orientation for delayed IMU interpolation."""
        if int(timestamp_us) <= 0:
            return

        sample = (
            int(timestamp_us),
            tuple(float(v) for v in self._quat_urdf),
            time.monotonic(),
        )
        if self._ahrs_history:
            previous_timestamp_us = int(self._ahrs_history[-1][0])
            if int(timestamp_us) < previous_timestamp_us:
                # Device restart or a time discontinuity: no pre-restart IMU
                # sample may be integrated with the new attitude timeline.
                self._ahrs_history.clear()
                self._reset_velocity_estimation_locked()
            elif int(timestamp_us) == previous_timestamp_us:
                self._ahrs_history[-1] = sample
                return

        self._ahrs_history.append(sample)

    def _queue_imu_sample_locked(self, timestamp_us: int) -> None:
        """Queue an immutable IMU snapshot until AHRS timestamps bracket it."""
        timestamp_us = int(timestamp_us)
        if (
            self._last_velocity_device_timestamp_us is not None
            and timestamp_us <= self._last_velocity_device_timestamp_us
        ):
            self._velocity_rejected_sample_count += 1
            return
        if self._pending_imu_samples:
            last_pending_timestamp_us = int(self._pending_imu_samples[-1][0])
            if timestamp_us <= last_pending_timestamp_us:
                self._velocity_rejected_sample_count += 1
                return

        if len(self._pending_imu_samples) >= MAX_PENDING_IMU_SAMPLES:
            self._pending_imu_samples.popleft()
            self._velocity_rejected_sample_count += 1

        self._pending_imu_samples.append(
            (
                timestamp_us,
                tuple(float(v) for v in self._specific_force_urdf),
                tuple(float(v) for v in self._gyro_urdf),
                time.monotonic(),
            )
        )

    @staticmethod
    def _interpolate_quaternion(q0, q1, fraction: float):
        """Shortest-path normalized lerp, sufficient over one AHRS period."""
        q0 = _normalize_quat(tuple(float(v) for v in q0))
        q1 = _normalize_quat(tuple(float(v) for v in q1))
        if sum(q0[i] * q1[i] for i in range(4)) < 0.0:
            q1 = tuple(-v for v in q1)
        fraction = min(1.0, max(0.0, float(fraction)))
        interpolated = tuple(
            (1.0 - fraction) * q0[i] + fraction * q1[i]
            for i in range(4)
        )
        return _normalize_quat(interpolated)

    def _drain_aligned_imu_samples_locked(self) -> None:
        """Integrate queued IMU samples with timestamp-interpolated attitude."""
        now = time.monotonic()
        max_alignment_delta_us = int(MAX_AHRS_AGE_S * 1e6)

        while self._pending_imu_samples:
            (
                imu_timestamp_us,
                specific_force_urdf,
                gyro_urdf,
                imu_host_time,
            ) = self._pending_imu_samples[0]

            if not self._ahrs_history:
                if now - imu_host_time > MAX_AHRS_AGE_S:
                    self._pending_imu_samples.popleft()
                    self._velocity_rejected_sample_count += 1
                    continue
                break

            oldest_ahrs = self._ahrs_history[0]
            newest_ahrs = self._ahrs_history[-1]
            oldest_timestamp_us = int(oldest_ahrs[0])
            newest_timestamp_us = int(newest_ahrs[0])

            sample_quaternion = None
            nearest_timestamp_us = newest_timestamp_us

            if imu_timestamp_us < oldest_timestamp_us:
                if oldest_timestamp_us - imu_timestamp_us > max_alignment_delta_us:
                    self._pending_imu_samples.popleft()
                    self._velocity_rejected_sample_count += 1
                    continue
                sample_quaternion = oldest_ahrs[1]
                nearest_timestamp_us = oldest_timestamp_us
            elif imu_timestamp_us <= newest_timestamp_us:
                if len(self._ahrs_history) == 1:
                    sample_quaternion = newest_ahrs[1]
                else:
                    interval_us = newest_timestamp_us - oldest_timestamp_us
                    if interval_us <= 0:
                        sample_quaternion = newest_ahrs[1]
                    else:
                        fraction = (
                            imu_timestamp_us - oldest_timestamp_us
                        ) / interval_us
                        sample_quaternion = self._interpolate_quaternion(
                            oldest_ahrs[1],
                            newest_ahrs[1],
                            fraction,
                        )
                    if (
                        abs(imu_timestamp_us - oldest_timestamp_us)
                        < abs(imu_timestamp_us - newest_timestamp_us)
                    ):
                        nearest_timestamp_us = oldest_timestamp_us
            else:
                # Wait at most one bounded alignment window for the next AHRS.
                if (
                    imu_timestamp_us - newest_timestamp_us
                    > max_alignment_delta_us
                    or now - imu_host_time > MAX_AHRS_AGE_S
                ):
                    self._pending_imu_samples.popleft()
                    self._velocity_rejected_sample_count += 1
                    continue
                break

            self._pending_imu_samples.popleft()
            self._last_imu_ahrs_delta_s = (
                imu_timestamp_us - nearest_timestamp_us
            ) * 1e-6
            self._update_velocity_estimation_locked(
                imu_timestamp_us,
                specific_force_urdf,
                sample_quaternion,
                gyro_urdf,
            )

    @staticmethod
    def _body_to_world_rotation_matrix(
        q: Tuple[float, float, float, float]
    ) -> Tuple[Tuple[float, float, float], ...]:
        """Return the physical body-to-world rotation represented by ``q``.

        DETA40 reports a body-to-world quaternion.  The policy gravity
        observation historically uses the opposite multiplication convention,
        so observation projection and physical vector rotation must not share
        the same helper (see :meth:`_compute_gravity_vec_for_obs`).
        """
        qw, qx, qy, qz = _normalize_quat(q)
        return (
            (
                1.0 - 2.0 * (qy * qy + qz * qz),
                2.0 * (qx * qy - qw * qz),
                2.0 * (qx * qz + qw * qy),
            ),
            (
                2.0 * (qx * qy + qw * qz),
                1.0 - 2.0 * (qx * qx + qz * qz),
                2.0 * (qy * qz - qw * qx),
            ),
            (
                2.0 * (qx * qz - qw * qy),
                2.0 * (qy * qz + qw * qx),
                1.0 - 2.0 * (qx * qx + qy * qy),
            ),
        )

    @classmethod
    def _rotate_world_to_body(cls, q, vector):
        rotation = cls._body_to_world_rotation_matrix(q)
        return tuple(
            sum(rotation[row][column] * float(vector[row]) for row in range(3))
            for column in range(3)
        )

    @classmethod
    def _rotate_body_to_world(cls, q, vector):
        rotation = cls._body_to_world_rotation_matrix(q)
        return tuple(
            sum(rotation[row][column] * float(vector[column]) for column in range(3))
            for row in range(3)
        )

    @staticmethod
    def _compute_gravity_vec_for_obs(q: Tuple[float, float, float, float]) -> Tuple[float, float, float]:
        # Keep the historical policy convention used by the C++ deployment.
        # This is R(q) * gravity, whereas physical world-to-body projection is
        # R(q).T * gravity and is handled separately below.
        qw, qx, qy, qz = _normalize_quat(q)
        gx = -2.0 * (qx * qz + qw * qy)
        gy = -2.0 * (qy * qz - qw * qx)
        gz = -(1.0 - 2.0 * (qx * qx + qy * qy))
        n = (gx * gx + gy * gy + gz * gz) ** 0.5
        if n > 1e-2:
            return (gx / n, gy / n, gz / n)
        return (0.0, 0.0, -1.0)

    @staticmethod
    def _compute_gravity_vec_for_comp(q: Tuple[float, float, float, float]) -> Tuple[float, float, float]:
        # Physical gravity projection in the accelerometer/body frame.  Do not
        # replace this with the policy observation formula above: their
        # horizontal signs differ for the DETA40 quaternion convention.
        gx, gy, gz = IMUSDK._rotate_world_to_body(q, (0.0, 0.0, -1.0))
        n = math.sqrt(gx * gx + gy * gy + gz * gz)
        if n > 1e-2:
            return (gx / n, gy / n, gz / n)
        return (0.0, 0.0, -1.0)

    def _calibrate_accelerometer_bias_locked(self) -> bool:
        sample_count = len(self._quiet_bias_candidates)
        self._last_bias_calibration_sample_count = sample_count
        if sample_count < BIAS_CALIBRATION_MIN_SAMPLES:
            return False

        # Component-wise median rejects isolated contact/vibration samples.
        calibrated_bias = []
        for axis in range(3):
            values = sorted(float(sample[axis]) for sample in self._quiet_bias_candidates)
            middle = len(values) // 2
            if len(values) % 2:
                calibrated_bias.append(values[middle])
            else:
                calibrated_bias.append(0.5 * (values[middle - 1] + values[middle]))

        self._accelerometer_bias_urdf = calibrated_bias
        self._quiet_bias_candidates.clear()
        return True

    def _reset_velocity_estimation_locked(self) -> None:
        self._linear_acceleration_world = [0.0, 0.0, 0.0]
        self._filtered_linear_acceleration_world = None
        self._estimated_velocity_world = [0.0, 0.0, 0.0]
        self._estimated_velocity = [0.0, 0.0, 0.0]
        self._last_acceleration_world = None
        self._last_velocity_sample_time_s = None
        self._last_velocity_device_timestamp_us = None
        self._velocity_time_source = None
        self._last_velocity_dt_s = 0.0
        self._last_velocity_update_host_t = 0.0
        self._pending_imu_samples.clear()

    def _update_velocity_estimation_locked(
        self,
        timestamp_us: int,
        specific_force_urdf=None,
        sample_quaternion=None,
        gyro_urdf=None,
    ) -> None:
        """Integrate one fresh IMU sample in the non-rotating world frame."""
        if specific_force_urdf is None:
            specific_force_urdf = self._specific_force_urdf
        if sample_quaternion is None:
            sample_quaternion = self._quat_urdf
        if gyro_urdf is None:
            gyro_urdf = self._gyro_urdf

        specific_force_urdf = [float(v) for v in specific_force_urdf]
        sample_quaternion = _normalize_quat(
            tuple(float(v) for v in sample_quaternion)
        )
        gyro_urdf = [float(v) for v in gyro_urdf]

        gravity_body = self._compute_gravity_vec_for_comp(sample_quaternion)
        bias_candidate = [
            specific_force_urdf[i] + GRAVITY_ACCELERATION * gravity_body[i]
            for i in range(3)
        ]
        candidate_norm = math.sqrt(sum(v * v for v in bias_candidate))
        gyro_norm = math.sqrt(sum(v * v for v in gyro_urdf))
        if (
            candidate_norm <= BIAS_CALIBRATION_MAX_ACCEL_MPS2
            and gyro_norm <= BIAS_CALIBRATION_MAX_GYRO_RAD_S
        ):
            self._quiet_bias_candidates.append(tuple(bias_candidate))

        corrected_specific_force_urdf = [
            specific_force_urdf[i] - self._accelerometer_bias_urdf[i]
            for i in range(3)
        ]
        specific_force_world = self._rotate_body_to_world(
            sample_quaternion,
            corrected_specific_force_urdf,
        )
        acceleration_world = [
            float(specific_force_world[0]),
            float(specific_force_world[1]),
            float(specific_force_world[2] - GRAVITY_ACCELERATION),
        ]
        if not all(math.isfinite(v) for v in acceleration_world):
            self._velocity_rejected_sample_count += 1
            return

        # Filter only after gravity compensation in the fixed world frame. A
        # body-frame LPF followed by a newer attitude leaks gravity during pitch.
        acceleration_world = self._lpf_vector(
            acceleration_world,
            "_filtered_linear_acceleration_world",
        )
        self._linear_acceleration_world = list(acceleration_world)

        if int(timestamp_us) > 0:
            sample_time_s = float(timestamp_us) * 1e-6
            time_source = "device_timestamp"
        else:
            sample_time_s = time.monotonic()
            time_source = "monotonic_fallback"

        if (
            self._last_velocity_sample_time_s is None
            or self._velocity_time_source != time_source
        ):
            self._last_velocity_sample_time_s = sample_time_s
            self._last_velocity_device_timestamp_us = (
                int(timestamp_us) if time_source == "device_timestamp" else None
            )
            self._velocity_time_source = time_source
            self._last_acceleration_world = list(acceleration_world)
            self._last_velocity_dt_s = 0.0
            self._last_velocity_update_host_t = time.monotonic()
            return

        if time_source == "device_timestamp":
            previous_timestamp_us = self._last_velocity_device_timestamp_us
            if previous_timestamp_us is None:
                previous_timestamp_us = int(timestamp_us)
            dt = (int(timestamp_us) - previous_timestamp_us) * 1e-6
        else:
            dt = float(sample_time_s - self._last_velocity_sample_time_s)
        self._last_velocity_dt_s = dt

        if dt < 0.0:
            # A backwards device timestamp normally means the IMU rebooted.
            self._reset_velocity_estimation_locked()
            self._last_velocity_sample_time_s = sample_time_s
            self._last_velocity_device_timestamp_us = (
                int(timestamp_us) if time_source == "device_timestamp" else None
            )
            self._velocity_time_source = time_source
            self._last_acceleration_world = list(acceleration_world)
            self._velocity_rejected_sample_count += 1
            return

        if dt < VELOCITY_DT_MIN_S or dt > VELOCITY_DT_MAX_S:
            # Rebase the trapezoid after duplicates or dropped-packet gaps; do
            # not integrate a stale acceleration across the missing interval.
            self._last_velocity_sample_time_s = sample_time_s
            self._last_velocity_device_timestamp_us = (
                int(timestamp_us) if time_source == "device_timestamp" else None
            )
            self._last_acceleration_world = list(acceleration_world)
            self._velocity_rejected_sample_count += 1
            return

        previous_acceleration = self._last_acceleration_world
        if previous_acceleration is None:
            previous_acceleration = acceleration_world

        estimated_velocity_world = [
            self._estimated_velocity_world[i]
            + 0.5
            * (float(previous_acceleration[i]) + float(acceleration_world[i]))
            * dt
            for i in range(3)
        ]

        # Preserve the old estimator's safety bound without applying its
        # packet-rate-dependent 0.95/0.7 decay to genuine constant velocity.
        speed = math.sqrt(sum(v * v for v in estimated_velocity_world))
        if speed > MAX_ESTIMATED_SPEED_MPS:
            scale = MAX_ESTIMATED_SPEED_MPS / speed
            estimated_velocity_world = [v * scale for v in estimated_velocity_world]

        self._estimated_velocity_world = estimated_velocity_world
        self._estimated_velocity = list(
            self._rotate_world_to_body(
                self._quat_urdf,
                self._estimated_velocity_world,
            )
        )
        self._last_acceleration_world = list(acceleration_world)
        self._last_velocity_sample_time_s = sample_time_s
        self._last_velocity_device_timestamp_us = (
            int(timestamp_us) if time_source == "device_timestamp" else None
        )
        self._last_velocity_update_host_t = time.monotonic()
        self._velocity_update_count += 1
