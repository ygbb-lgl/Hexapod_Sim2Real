import struct
import threading
import time
from typing import Dict, Optional, Tuple

import serial

# ================= Protocol Constants (FDILink) =================
FRAME_HEAD = 0xFC
FRAME_END = 0xFD

TYPE_IMU = 0x40
TYPE_AHRS = 0x41

IMU_LEN = 0x38  # 56 bytes
AHRS_LEN = 0x30  # 48 bytes


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
    if n2 <= 1e-12:
        return (1.0, 0.0, 0.0, 0.0)
    inv = (n2 ** -0.5)
    return (qw * inv, qx * inv, qy * inv, qz * inv)


class IMUSDK:
    """FDILink IMU SDK (Python).

    This implementation is intentionally aligned with the working C++ reference:
      - rl_real_el4090.cpp::IMURecv() coordinate transform
      - rl_real_el4090.cpp::UpdateVelocityEstimation() velocity integration
      - rl_real_el4090.cpp::RunModel() gravity_vec formula

    Coordinate frames:
      - Raw IMU frame (from sensor): X forward, Y right, Z down
      - URDF/body frame (used by policy): X forward, Y left, Z up
        => (x, y, z)_urdf = (x, -y, -z)_imu
        => quaternion (w, x, y, z)_urdf = (w, x, -y, -z)_imu
    """

    def __init__(self, port: str = "/dev/ttyUSB0", baudrate: int = 921600, timeout_s: float = 0.02):
        self.port = port
        self.baudrate = int(baudrate)
        self.timeout_s = float(timeout_s)

        self.serial: Optional[serial.Serial] = None
        self.running = False
        self.thread: Optional[threading.Thread] = None
        self.lock = threading.Lock()

        # Latest decoded raw packets (IMU frame)
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
            "temp": 0.0,
            "pressure": 0.0,
            "pressure_temp": 0.0,
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

        self.valid_imu = False
        self.valid_ahrs = False

        # URDF/body-frame state (what deployment should use)
        self._quat_urdf: Tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)
        self._gyro_urdf = [0.0, 0.0, 0.0]
        self._acc_urdf = [0.0, 0.0, 0.0]

        # Gravity direction in body frame (unit vector)
        self._gravity_vec = [0.0, 0.0, -1.0]

        # Velocity estimation state (body frame)
        self._estimated_velocity = [0.0, 0.0, 0.0]
        self._last_acc = [0.0, 0.0, 0.0]
        self._last_vel_update_t = time.monotonic()

    # -------------------- Public API --------------------
    def start(self) -> bool:
        try:
            self.serial = serial.Serial(self.port, self.baudrate, timeout=self.timeout_s)
            self.running = True
            self.thread = threading.Thread(target=self._read_loop, daemon=True)
            self.thread.start()
            print(f"IMU SDK: started on {self.port} @ {self.baudrate}")
            return True
        except Exception as e:
            print(f"IMU SDK: failed to open serial port: {e}")
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
        """Latest IMU packet data in URDF/body frame."""
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
        """Estimated base linear velocity [vx, vy, vz] in body frame (m/s)."""
        with self.lock:
            # Align with rl_real_el4090.cpp: obs.lin_vel[2] is forced to 0 before
            # feeding the policy network. We do the same here to avoid z-drift
            # from the accelerometer integration poisoning downstream consumers.
            return [float(self._estimated_velocity[0]), float(self._estimated_velocity[1]), float(self._estimated_velocity[2])]

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
                h_start, h_type, h_len, h_sn, h_crc8, h_crc16_h, h_crc16_l = struct.unpack("7B", header)
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
                print(f"IMU SDK: serial error: {e}")
                time.sleep(1.0)
            except Exception as e:
                print(f"IMU SDK: unexpected error: {e}")
                time.sleep(0.2)

    def _parse_payload(self, data_type: int, payload: bytes) -> None:
        # Decode under lock to keep state consistent.
        with self.lock:
            if data_type == TYPE_IMU:
                if len(payload) != IMU_LEN:
                    return
                vals = struct.unpack("<12fq", payload)
                # Raw IMU frame (no sign changes here)
                self._raw_imu["gyro_x"] = float(vals[0])
                self._raw_imu["gyro_y"] = float(vals[1])
                self._raw_imu["gyro_z"] = float(vals[2])
                self._raw_imu["acc_x"] = float(vals[3])
                self._raw_imu["acc_y"] = float(vals[4])
                self._raw_imu["acc_z"] = float(vals[5])
                self._raw_imu["mag_x"] = float(vals[6])
                self._raw_imu["mag_y"] = float(vals[7])
                self._raw_imu["mag_z"] = float(vals[8])
                self._raw_imu["temp"] = float(vals[9])
                self._raw_imu["pressure"] = float(vals[10])
                self._raw_imu["pressure_temp"] = float(vals[11])
                self._raw_imu["timestamp"] = float(vals[12])
                self.valid_imu = True

            elif data_type == TYPE_AHRS:
                if len(payload) != AHRS_LEN:
                    return
                vals = struct.unpack("<10fq", payload)
                # Raw IMU frame
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
                self._raw_ahrs["timestamp"] = float(vals[10])
                self.valid_ahrs = True

            # If we have both packets, update URDF-frame state + velocity/gravity.
            if self.valid_imu and self.valid_ahrs:
                self._update_urdf_state_locked()

    # -------------------- Internal: math (aligned with C++) --------------------
    def _update_urdf_state_locked(self) -> None:
        # 1) Coordinate transform IMU->URDF (exactly like rl_real_el4090.cpp::IMURecv)
        qw = float(self._raw_ahrs["qw"])
        qx = float(self._raw_ahrs["qx"])
        qy = -float(self._raw_ahrs["qy"])
        qz = -float(self._raw_ahrs["qz"])
        self._quat_urdf = _normalize_quat((qw, qx, qy, qz))

        gx = float(self._raw_imu["gyro_x"])
        gy = -float(self._raw_imu["gyro_y"])
        gz = -float(self._raw_imu["gyro_z"])
        self._gyro_urdf = [gx, gy, gz]

        ax = float(self._raw_imu["acc_x"])
        ay = -float(self._raw_imu["acc_y"])
        az = -float(self._raw_imu["acc_z"])
        self._acc_urdf = [ax, ay, az]

        # 2) Gravity vector for observation (aligned with rl_real_el4090.cpp::RunModel)
        self._gravity_vec = list(self._compute_gravity_vec_for_obs(self._quat_urdf))

        # 3) Velocity estimation (aligned with rl_real_el4090.cpp::UpdateVelocityEstimation)
        self._update_velocity_estimation_locked()

    @staticmethod
    def _compute_gravity_vec_for_obs(q: Tuple[float, float, float, float]) -> Tuple[float, float, float]:
        qw, qx, qy, qz = q
        gx = -2.0 * (qx * qz + qw * qy)
        gy = -2.0 * (qy * qz - qw * qx)
        gz = -(1.0 - 2.0 * (qx * qx + qy * qy))
        n = (gx * gx + gy * gy + gz * gz) ** 0.5
        if n > 1e-2:
            return (gx / n, gy / n, gz / n)
        return (0.0, 0.0, -1.0)

    @staticmethod
    def _compute_gravity_vec_for_comp(q: Tuple[float, float, float, float]) -> Tuple[float, float, float]:
        # Note: this is the version used by UpdateVelocityEstimation in C++.
        qw, qx, qy, qz = q
        gx = -2.0 * (qx * qz - qw * qy)
        gy = -2.0 * (qy * qz + qw * qx)
        gz = -(1.0 - 2.0 * (qx * qx + qy * qy))
        return (gx, gy, gz)

    def _update_velocity_estimation_locked(self) -> None:
        now = time.monotonic()
        dt = float(now - self._last_vel_update_t)
        self._last_vel_update_t = now

        # Keep the same dt gate as C++.
        if dt > 0.01 or dt < 0.0001:
            return

        gravity_body_x, gravity_body_y, gravity_body_z = self._compute_gravity_vec_for_comp(self._quat_urdf)

        # Gravity compensation
        g = 9.81
        # gai
        ax_comp = self._acc_urdf[0] + gravity_body_x * g
        ay_comp = self._acc_urdf[1] + gravity_body_y * g
        az_comp = self._acc_urdf[2] + gravity_body_z * g

        gyro_norm = (self._gyro_urdf[0] ** 2 + self._gyro_urdf[1] ** 2 + self._gyro_urdf[2] ** 2) ** 0.5
        acc_norm = (ax_comp ** 2 + ay_comp ** 2 + az_comp ** 2) ** 0.5

        decay = 0.95
        if acc_norm < 0.5 and gyro_norm < 0.1:
            decay = 0.7

        self._estimated_velocity[0] = decay * self._estimated_velocity[0] + 0.5 * (ax_comp + self._last_acc[0]) * dt
        self._estimated_velocity[1] = decay * self._estimated_velocity[1] + 0.5 * (ay_comp + self._last_acc[1]) * dt
        self._estimated_velocity[2] = decay * self._estimated_velocity[2] + 0.5 * (az_comp + self._last_acc[2]) * dt

        self._last_acc[0] = ax_comp
        self._last_acc[1] = ay_comp
        self._last_acc[2] = az_comp

        max_vel = 2.0
        for i in range(3):
            if self._estimated_velocity[i] > max_vel:
                self._estimated_velocity[i] = max_vel
            elif self._estimated_velocity[i] < -max_vel:
                self._estimated_velocity[i] = -max_vel
