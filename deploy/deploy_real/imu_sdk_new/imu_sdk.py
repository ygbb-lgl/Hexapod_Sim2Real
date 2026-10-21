#!/usr/bin/env python3
"""Threaded DETA40 reader compatible with ``deploy_real_hexapod.py``.

The wire layout in this module starts from the vendor demo in this directory.
The demo reads the two CRC fields without supplying their algorithms.  The
algorithms and coverage used here were therefore established independently
from a complete capture produced by this DETA40: CRC-8/MAXIM-DOW covers the
first four header bytes and CRC-16/XMODEM covers the payload.  Consequently
this SDK:

* parses the packet types and offsets shown by the demo;
* rejects known packet types unless both header and payload CRCs validate;
* keeps Q1--Q4 as raw vendor values rather than guessing their order; and
* computes projected gravity from the documented roll/pitch fields; and
* estimates short-term velocity by gravity-compensating and integrating the
  0x40 accelerometer, with a stationary-start bias calibration and ZUPT.

The DETA40 body frame is FRD (x forward, y right, z down).  Public vector
outputs used by the policy are converted to FLU (x forward, y left, z up) with
``(x, y, z) -> (x, -y, -z)``.

The basic DETA40 does not emit the demo's optional 0x42 ``BodyVelocity``
packet.  Its 0x40 accelerometer is therefore treated as specific force.  The
estimator rotates it into a world FLU frame with the latest 0x41 ZYX Euler
attitude, adds world gravity ``[0, 0, -9.81] m/s^2``, and integrates there.
The result is rotated back to body FLU for the deployment API.  This is a
short-term inertial estimate: without an external velocity observation, ZUPT
cannot distinguish true rest from perfectly smooth constant-velocity motion.
"""

from __future__ import annotations

import atexit
import math
from numbers import Integral
import os
import struct
import sys
import threading
import time
from typing import Any, Dict, Iterable, List, Mapping, Optional, Tuple

try:
    import serial
except ImportError:  # Keep the module importable for offline parser tests.
    serial = None  # type: ignore[assignment]


Vector3 = Tuple[float, float, float]
Matrix3 = Tuple[Vector3, Vector3, Vector3]


FRAME_HEAD = 0xFC
FRAME_END = 0xFD

TYPE_IMU = 0x40
TYPE_AHRS = 0x41
TYPE_INSGPS = 0x42
TYPE_GEODETIC_POS = 0x5C
TYPE_GROUND = 0xF0
TYPE_SYS_STATE = 0x50

PAYLOAD_LENGTHS: Mapping[int, int] = {
    TYPE_IMU: 0x38,
    TYPE_AHRS: 0x30,
    TYPE_INSGPS: 0x48,
    TYPE_GEODETIC_POS: 0x20,
    TYPE_SYS_STATE: 0x64,
}

_POLICY_STREAM_NAMES: Mapping[int, str] = {
    TYPE_IMU: "imu",
    TYPE_AHRS: "ahrs",
    TYPE_INSGPS: "insgps",
}

_ZERO_VECTOR: Vector3 = (0.0, 0.0, 0.0)
_LEVEL_GRAVITY_FLU: Vector3 = (0.0, 0.0, -1.0)
_GRAVITY_MPS2 = 9.81
_WORLD_GRAVITY_FLU_MPS2: Vector3 = (0.0, 0.0, -_GRAVITY_MPS2)
_TIMESTAMP_STALL_LIMIT = 3


def calc_crc8(data: bytes) -> int:
    """Return DETA40 header CRC-8/MAXIM-DOW.

    Parameters are poly=0x31 (reflected form 0x8c), init=0x00,
    refin/refout=True, and xorout=0x00.  The frame CRC covers
    ``[0xfc, type, payload_length, sequence]``.
    """

    crc = 0
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = (crc >> 1) ^ 0x8C if crc & 0x01 else crc >> 1
    return crc & 0xFF


def calc_crc16(data: bytes) -> int:
    """Return DETA40 payload CRC-16/XMODEM.

    Parameters are poly=0x1021, init=0x0000, refin/refout=False, and
    xorout=0x0000.  The result is transmitted high byte first in the two CRC
    bytes stored in the frame header.
    """

    crc = 0
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def _frd_to_flu(x: float, y: float, z: float) -> Vector3:
    """Rotate a body-frame vector from FRD to FLU."""

    return (float(x), -float(y), -float(z))


def _all_finite(values: Iterable[float]) -> bool:
    return all(math.isfinite(value) for value in values)


def _norm(vector: Vector3) -> float:
    return math.sqrt(sum(value * value for value in vector))


def _rotation_body_to_world(
    roll: float, pitch: float, heading: float
) -> Matrix3:
    """Return the active FLU body-to-world ZYX rotation matrix."""

    sin_roll = math.sin(roll)
    cos_roll = math.cos(roll)
    sin_pitch = math.sin(pitch)
    cos_pitch = math.cos(pitch)
    sin_heading = math.sin(heading)
    cos_heading = math.cos(heading)
    return (
        (
            cos_heading * cos_pitch,
            cos_heading * sin_pitch * sin_roll
            - sin_heading * cos_roll,
            cos_heading * sin_pitch * cos_roll
            + sin_heading * sin_roll,
        ),
        (
            sin_heading * cos_pitch,
            sin_heading * sin_pitch * sin_roll
            + cos_heading * cos_roll,
            sin_heading * sin_pitch * cos_roll
            - cos_heading * sin_roll,
        ),
        (
            -sin_pitch,
            cos_pitch * sin_roll,
            cos_pitch * cos_roll,
        ),
    )


def _rotate(matrix: Matrix3, vector: Vector3) -> Vector3:
    """Apply a 3x3 row-major matrix to a vector."""

    return tuple(
        sum(matrix[row][column] * vector[column] for column in range(3))
        for row in range(3)
    )  # type: ignore[return-value]


def _rotate_transpose(matrix: Matrix3, vector: Vector3) -> Vector3:
    """Apply the transpose of a 3x3 row-major matrix to a vector."""

    return tuple(
        sum(matrix[row][column] * vector[row] for row in range(3))
        for column in range(3)
    )  # type: ignore[return-value]


def _age(now: float, timestamp: Optional[float]) -> Optional[float]:
    if timestamp is None:
        return None
    return max(0.0, now - timestamp)


class _FDILinkStreamParser:
    """Incremental parser for the frame layout consumed by the vendor demo."""

    def __init__(self) -> None:
        self.buffer = bytearray()
        self.frame_error_count = 0
        self.crc_error_count = 0
        self.header_crc_error_count = 0
        self.payload_crc_error_count = 0
        self.discarded_byte_count = 0
        self.ground_candidate_count = 0
        self.resyncing_after_ground = False

    def reset(self) -> None:
        self.buffer.clear()
        self.frame_error_count = 0
        self.crc_error_count = 0
        self.header_crc_error_count = 0
        self.payload_crc_error_count = 0
        self.discarded_byte_count = 0
        self.ground_candidate_count = 0
        self.resyncing_after_ground = False

    def feed(self, chunk: bytes) -> List[Tuple[Dict[str, int], bytes]]:
        if chunk:
            self.buffer.extend(chunk)

        frames: List[Tuple[Dict[str, int], bytes]] = []
        while True:
            head_index = self.buffer.find(bytes((FRAME_HEAD,)))
            if head_index < 0:
                self.discarded_byte_count += len(self.buffer)
                self.buffer.clear()
                break
            if head_index:
                del self.buffer[:head_index]
                self.discarded_byte_count += head_index

            # The demo identifies 0xf0 after reading only fc, type, and length,
            # then resumes its byte-wise head search without defining the rest
            # of that packet.  Do the same instead of trusting an unknown length
            # and potentially consuming a following valid frame.
            if len(self.buffer) < 3:
                break

            packet_type = self.buffer[1]
            payload_length = self.buffer[2]
            if packet_type == TYPE_GROUND:
                del self.buffer[:3]
                self.ground_candidate_count += 1
                self.discarded_byte_count += 3
                self.resyncing_after_ground = True
                continue

            # fc, type, length, sn, crc8, crc16_h, crc16_l
            if len(self.buffer) < 7:
                break

            expected_length = PAYLOAD_LENGTHS.get(packet_type)
            if expected_length is None or payload_length != expected_length:
                # Only discard the candidate head so a following 0xfc can be
                # found even after a corrupted length byte.
                del self.buffer[0]
                if not self.resyncing_after_ground:
                    self.frame_error_count += 1
                self.discarded_byte_count += 1
                continue

            expected_header_crc = self.buffer[4]
            actual_header_crc = calc_crc8(bytes(self.buffer[:4]))
            if actual_header_crc != expected_header_crc:
                # The length byte cannot be trusted when its header CRC fails.
                # Drop only this candidate head so the next valid 0xfc remains
                # available to the byte-wise resynchronizer.
                del self.buffer[0]
                if not self.resyncing_after_ground:
                    self.crc_error_count += 1
                    self.header_crc_error_count += 1
                self.discarded_byte_count += 1
                continue

            frame_length = 7 + payload_length
            if len(self.buffer) < frame_length:
                break

            payload = bytes(self.buffer[7:frame_length])
            expected_payload_crc = (
                (int(self.buffer[5]) << 8) | int(self.buffer[6])
            )
            actual_payload_crc = calc_crc16(payload)
            if actual_payload_crc != expected_payload_crc:
                if not self.resyncing_after_ground:
                    self.crc_error_count += 1
                    self.payload_crc_error_count += 1
                # Even with a valid header, an inserted or missing payload byte
                # can make the claimed frame extend into the following frame.
                # Drop only this candidate head and search byte-wise so that a
                # later valid 0xfc is never swallowed.
                del self.buffer[0]
                self.discarded_byte_count += 1
                continue

            header = {
                "type": packet_type,
                "length": payload_length,
                "sequence": self.buffer[3],
                "crc8": self.buffer[4],
                "crc16_high": self.buffer[5],
                "crc16_low": self.buffer[6],
            }
            del self.buffer[:frame_length]

            # FRAME_END is defined by the demo but never consumed there.  Do
            # not require it; if present it will be discarded while searching
            # for the next 0xfc.
            frames.append((header, payload))
            self.resyncing_after_ground = False

        # Bound garbage retained after a damaged stream.  A valid payload is at
        # most 255 bytes because the length field is one byte.
        if len(self.buffer) > 4096:
            keep = self.buffer[-7:]
            self.discarded_byte_count += len(self.buffer) - len(keep)
            self.buffer = bytearray(keep)
            self.frame_error_count += 1

        return frames


class IMUSDK:
    """Read DETA40 packets in a background thread and expose latest samples.

    Parameters needed by the deployment script are ``port`` and ``baudrate``.
    ``data_timeout_s`` controls when a cached sample is considered stale.  A
    stale/missing velocity returns zero and stale/missing gravity returns the
    safe level default ``[0, 0, -1]``.  ``get_imu_data`` is also the policy
    health gate: with the default ``strict_policy_health=True`` it raises until
    IMU and AHRS are fresh and the stationary-start velocity estimator is
    calibrated.  ``get_raw_imu_data`` remains available during calibration.
    """

    def __init__(
        self,
        port: str = "/dev/ttyUSB0",
        baudrate: int = 921600,
        timeout: float = 0.1,
        data_timeout_s: float = 0.1,
        strict_policy_health: bool = True,
        stationary_calibration_s: float = 1.0,
        accelerometer_rest_sign: int = 0,
        stationary_accel_threshold_mps2: float = 0.12,
        stationary_gyro_threshold_rad_s: float = 0.15,
        stationary_hold_s: float = 0.25,
        max_integration_dt_s: float = 0.05,
    ) -> None:
        if not port:
            raise ValueError("port must be a non-empty string")
        if isinstance(baudrate, bool) or not isinstance(baudrate, Integral):
            raise ValueError("baudrate must be a positive integer")
        if baudrate <= 0:
            raise ValueError("baudrate must be a positive integer")
        try:
            timeout_value = float(timeout)
            data_timeout_value = float(data_timeout_s)
            calibration_value = float(stationary_calibration_s)
            stationary_accel_value = float(
                stationary_accel_threshold_mps2
            )
            stationary_gyro_value = float(
                stationary_gyro_threshold_rad_s
            )
            stationary_hold_value = float(stationary_hold_s)
            max_integration_dt_value = float(max_integration_dt_s)
        except (TypeError, ValueError, OverflowError) as exc:
            raise ValueError(
                "estimator timing and thresholds must be finite numbers"
            ) from exc
        if not math.isfinite(timeout_value) or timeout_value <= 0.0:
            raise ValueError("timeout must be finite and positive")
        if not math.isfinite(data_timeout_value) or data_timeout_value <= 0.0:
            raise ValueError("data_timeout_s must be finite and positive")
        if not math.isfinite(calibration_value) or calibration_value < 0.0:
            raise ValueError(
                "stationary_calibration_s must be finite and non-negative"
            )
        if (
            not math.isfinite(stationary_accel_value)
            or stationary_accel_value <= 0.0
        ):
            raise ValueError(
                "stationary_accel_threshold_mps2 must be finite and positive"
            )
        if (
            not math.isfinite(stationary_gyro_value)
            or stationary_gyro_value <= 0.0
        ):
            raise ValueError(
                "stationary_gyro_threshold_rad_s must be finite and positive"
            )
        if (
            not math.isfinite(stationary_hold_value)
            or stationary_hold_value < 0.0
        ):
            raise ValueError(
                "stationary_hold_s must be finite and non-negative"
            )
        if (
            not math.isfinite(max_integration_dt_value)
            or max_integration_dt_value <= 0.0
        ):
            raise ValueError(
                "max_integration_dt_s must be finite and positive"
            )
        if (
            isinstance(accelerometer_rest_sign, bool)
            or not isinstance(accelerometer_rest_sign, Integral)
            or int(accelerometer_rest_sign) not in (-1, 0, 1)
        ):
            raise ValueError("accelerometer_rest_sign must be -1, 0, or 1")

        self.port: str = port
        self.baudrate: int = int(baudrate)
        self.timeout: float = timeout_value
        self.data_timeout_s: float = data_timeout_value
        self.strict_policy_health: bool = bool(strict_policy_health)
        self.stationary_calibration_s: float = calibration_value
        self.stationary_accel_threshold_mps2: float = stationary_accel_value
        self.stationary_gyro_threshold_rad_s: float = stationary_gyro_value
        self.stationary_hold_s: float = stationary_hold_value
        self.max_integration_dt_s: float = max_integration_dt_value
        self._configured_accelerometer_rest_sign: int = int(
            accelerometer_rest_sign
        )
        # Public for compatibility; 0 means auto until the first usable pair.
        self.accelerometer_rest_sign: int = int(accelerometer_rest_sign)

        self._state_lock: Any = threading.Lock()
        self._lifecycle_lock: Any = threading.Lock()
        self._stop_event: threading.Event = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._serial: Any = None
        self._parser: _FDILinkStreamParser = _FDILinkStreamParser()

        self._running: bool = False
        self._last_error: Optional[str] = None
        self._serial_error_count: int = 0
        self._parse_error_count: int = 0
        self._unsupported_packet_count: int = 0
        self._packet_counts: Dict[int, int] = {
            packet_type: 0 for packet_type in PAYLOAD_LENGTHS
        }
        self._packet_counts[TYPE_GROUND] = 0
        self._last_headers: Dict[int, Dict[str, int]] = {}
        self._last_packet_monotonic: Dict[int, float] = {}
        self._last_interarrival_s: Dict[int, Optional[float]] = {
            packet_type: None for packet_type in _POLICY_STREAM_NAMES
        }
        self._max_interarrival_s: Dict[int, Optional[float]] = {
            packet_type: None for packet_type in _POLICY_STREAM_NAMES
        }
        self._gap_over_timeout_count: Dict[int, int] = {
            packet_type: 0 for packet_type in _POLICY_STREAM_NAMES
        }

        self._imu_data: Optional[Dict[str, Any]] = None
        self._ahrs_data: Optional[Dict[str, Any]] = None
        self._insgps_data: Optional[Dict[str, Any]] = None
        self._geodetic_data: Optional[Dict[str, Any]] = None
        self._system_state: Optional[Dict[str, Any]] = None
        self._linear_velocity: Vector3 = _ZERO_VECTOR
        self._gravity_flu: Vector3 = _LEVEL_GRAVITY_FLU
        self._linear_acceleration_body: Vector3 = _ZERO_VECTOR
        self._linear_acceleration_world: Vector3 = _ZERO_VECTOR
        self._velocity_world: Vector3 = _ZERO_VECTOR
        self._accel_bias_flu: Vector3 = _ZERO_VECTOR
        self._gyro_bias_flu: Vector3 = _ZERO_VECTOR
        self._calibration_accel_bias_sum: List[float] = [0.0, 0.0, 0.0]
        self._calibration_gyro_bias_sum: List[float] = [0.0, 0.0, 0.0]
        self._calibration_accel_bias_sq_sum: List[float] = [0.0, 0.0, 0.0]
        self._calibration_gyro_bias_sq_sum: List[float] = [0.0, 0.0, 0.0]
        self._calibration_sample_count: int = 0
        self._calibration_stationary_since: Optional[float] = None
        self._stationary_since: Optional[float] = None
        self._stationary_elapsed_s: float = 0.0
        self._calibrated: bool = False
        self._stationary: bool = False
        self._velocity_valid: bool = False
        self._previous_acceleration_world: Optional[Vector3] = None
        self._last_estimator_update_monotonic: Optional[float] = None
        self._last_imu_timestamp_us: Optional[int] = None
        self._velocity_dt_s: Optional[float] = None
        self._velocity_host_dt_s: Optional[float] = None
        self._velocity_dt_source: str = "vendor_timestamp_word0_us"
        self._velocity_dt_reject_count: int = 0
        self._timestamp_stall_count: int = 0
        self._timestamp_stall_consecutive: int = 0
        self._zupt_count: int = 0
        self._velocity_estimator_error: Optional[str] = None
        # The current deployment script does not call imu.stop().  Keep normal
        # interpreter shutdown deterministic without changing that script.
        atexit.register(self.stop)

    def start(self) -> bool:
        """Open the serial port and start the receiver thread.

        Returns ``True`` when the port was opened and the thread was started;
        packet availability is reported separately by ``get_diagnostics``.
        The method is idempotent.
        """

        with self._lifecycle_lock:
            if self._thread is not None and self._thread.is_alive():
                stop_requested = self._stop_event.is_set()
                with self._state_lock:
                    receiver_running = self._running
                if receiver_running and not stop_requested:
                    return True
                if not stop_requested:
                    self._stop_event.set()
                self._set_start_error(
                    "previous receiver thread is still stopping"
                )
                return False

            # Clean up a receiver that exited because of a serial error.
            old_serial = self._serial
            if old_serial is not None:
                try:
                    old_serial.close()
                except Exception:
                    pass
            self._serial = None
            self._thread = None

            if serial is None:
                message = "pyserial is not installed"
                self._set_start_error(message)
                print("[IMUSDK] {}".format(message), file=sys.stderr)
                return False

            stop_event = threading.Event()
            self._stop_event = stop_event
            self._reset_runtime_state()
            serial_kwargs = {
                "port": self.port,
                "baudrate": self.baudrate,
                "bytesize": serial.EIGHTBITS,
                "parity": serial.PARITY_NONE,
                "stopbits": serial.STOPBITS_ONE,
                "timeout": self.timeout,
            }
            try:
                try:
                    # pyserial implements this with a non-blocking exclusive
                    # lock on POSIX. Windows ports are exclusive by default;
                    # older pyserial versions may not accept the kwarg.
                    serial_port = serial.Serial(
                        exclusive=True, **serial_kwargs
                    )
                except TypeError as exc:
                    if os.name == "posix":
                        raise RuntimeError(
                            "installed pyserial does not support "
                            "exclusive ports"
                        ) from exc
                    serial_port = serial.Serial(**serial_kwargs)
            except Exception as exc:
                message = "unable to open {}: {}".format(self.port, exc)
                self._set_start_error(message)
                print("[IMUSDK] {}".format(message), file=sys.stderr)
                return False

            self._serial = serial_port
            with self._state_lock:
                self._running = True
                self._last_error = None
            self._thread = threading.Thread(
                target=self._receive_loop,
                args=(serial_port, stop_event),
                name="DETA40Receiver",
                daemon=True,
            )
            try:
                self._thread.start()
            except Exception as exc:
                try:
                    serial_port.close()
                except Exception:
                    pass
                self._serial = None
                self._thread = None
                message = "unable to start receiver thread: {}".format(exc)
                self._set_start_error(message)
                print("[IMUSDK] {}".format(message), file=sys.stderr)
                return False
            return True

    def stop(self) -> None:
        """Stop reception and close the port; safe to call repeatedly."""

        with self._lifecycle_lock:
            self._stop_event.set()
            serial_port = self._serial
            if serial_port is not None:
                try:
                    serial_port.close()
                except Exception:
                    pass

            thread = self._thread
            if thread is not None and thread is not threading.current_thread():
                thread.join(timeout=max(1.0, self.timeout + 0.5))

            thread_still_alive = thread is not None and thread.is_alive()
            if not thread_still_alive:
                self._serial = None
                self._thread = None
            with self._state_lock:
                self._running = False
                if thread_still_alive:
                    self._last_error = (
                        "receiver thread did not stop before timeout"
                    )

    close = stop

    def __enter__(self) -> "IMUSDK":
        if not self.start():
            raise RuntimeError(self.get_diagnostics()["last_error"])
        return self

    def __exit__(self, exc_type: Any, exc: Any, traceback: Any) -> None:
        self.stop()

    def get_imu_data(self) -> Optional[Dict[str, Any]]:
        """Return IMU data only while every policy-required stream is fresh.

        In strict mode a missing/stale IMU or AHRS stream, incomplete startup
        calibration, or invalid velocity estimate raises ``RuntimeError``.
        Setting ``strict_policy_health=False`` restores a ``None`` fallback.
        """

        now = time.monotonic()
        with self._state_lock:
            policy_ready = self._policy_ready_locked(now)
            if self._imu_data is None or not policy_ready:
                if self.strict_policy_health:
                    raise RuntimeError(
                        "DETA40 policy data unavailable: require fresh "
                        "0x40 IMU/0x41 AHRS and a calibrated, valid "
                        "integrated velocity estimate"
                    )
                return None
            return dict(self._imu_data)

    def get_raw_imu_data(self) -> Optional[Dict[str, Any]]:
        """Return latest finite 0x40 IMU sample regardless of other streams."""

        now = time.monotonic()
        with self._state_lock:
            timestamp = self._last_packet_monotonic.get(TYPE_IMU)
            if self._imu_data is None or not self._is_fresh(now, timestamp):
                return None
            return dict(self._imu_data)

    def get_ahrs_data(self) -> Optional[Dict[str, Any]]:
        """Return latest AHRS Euler data plus raw vendor Q1--Q4 values."""

        now = time.monotonic()
        with self._state_lock:
            timestamp = self._last_packet_monotonic.get(TYPE_AHRS)
            if self._ahrs_data is None or not self._is_fresh(now, timestamp):
                return None
            return dict(self._ahrs_data)

    def get_insgps_data(self) -> Optional[Dict[str, Any]]:
        """Return optional 0x42 data for diagnostics only."""

        now = time.monotonic()
        with self._state_lock:
            timestamp = self._last_packet_monotonic.get(TYPE_INSGPS)
            if self._insgps_data is None or not self._is_fresh(now, timestamp):
                return None
            return dict(self._insgps_data)

    def get_geodetic_position(self) -> Optional[Dict[str, Any]]:
        now = time.monotonic()
        with self._state_lock:
            timestamp = self._last_packet_monotonic.get(TYPE_GEODETIC_POS)
            if self._geodetic_data is None or not self._is_fresh(
                now, timestamp
            ):
                return None
            return dict(self._geodetic_data)

    def get_linear_velocity(self) -> Vector3:
        """Return gravity-compensated integrated body velocity in FLU [m/s].

        Zero is returned until startup stationary calibration completes and
        whenever IMU/AHRS data or the estimator becomes invalid.  The optional
        0x42 packet never drives this API.
        """

        now = time.monotonic()
        with self._state_lock:
            if not self._policy_ready_locked(now):
                return _ZERO_VECTOR
            return tuple(self._linear_velocity)  # type: ignore[return-value]

    def get_angular_velocity(self) -> Vector3:
        """Return latest gyroscope vector in FLU [rad/s], or safe zeros."""

        data = self.get_raw_imu_data()
        if data is None:
            return _ZERO_VECTOR
        return (
            float(data["gyro_x"]),
            float(data["gyro_y"]),
            float(data["gyro_z"]),
        )

    def get_gravity_acceleration(self) -> Vector3:
        """Return unit gravity direction in the FLU body frame.

        Despite the legacy method name this is a dimensionless direction, as
        expected by ``deploy_real_hexapod.py``.  A level sensor reports
        approximately ``[0, 0, -1]``.
        """

        now = time.monotonic()
        with self._state_lock:
            timestamp = self._last_packet_monotonic.get(TYPE_AHRS)
            if not self._is_fresh(now, timestamp):
                return _LEVEL_GRAVITY_FLU
            return tuple(self._gravity_flu)  # type: ignore[return-value]

    def get_diagnostics(self) -> Dict[str, Any]:
        """Return communication, freshness, and parser diagnostics."""

        now = time.monotonic()
        with self._state_lock:
            ages = {
                "imu_age_s": _age(
                    now, self._last_packet_monotonic.get(TYPE_IMU)
                ),
                "ahrs_age_s": _age(
                    now, self._last_packet_monotonic.get(TYPE_AHRS)
                ),
                "insgps_age_s": _age(
                    now, self._last_packet_monotonic.get(TYPE_INSGPS)
                ),
                "geodetic_age_s": _age(
                    now, self._last_packet_monotonic.get(TYPE_GEODETIC_POS)
                ),
            }
            imu_fresh = self._is_fresh(
                now, self._last_packet_monotonic.get(TYPE_IMU)
            )
            ahrs_fresh = self._is_fresh(
                now, self._last_packet_monotonic.get(TYPE_AHRS)
            )
            insgps_fresh = self._is_fresh(
                now, self._last_packet_monotonic.get(TYPE_INSGPS)
            )
            policy_ready = self._policy_ready_locked(now)
            stream_timing: Dict[str, Any] = {}
            for packet_type, name in _POLICY_STREAM_NAMES.items():
                stream_timing[name + "_last_interarrival_s"] = (
                    self._last_interarrival_s[packet_type]
                )
                stream_timing[name + "_max_interarrival_s"] = (
                    self._max_interarrival_s[packet_type]
                )
                stream_timing[name + "_gap_over_timeout_count"] = (
                    self._gap_over_timeout_count[packet_type]
                )
            diagnostics: Dict[str, Any] = {
                "running": self._running,
                "stop_requested": self._stop_event.is_set(),
                "port": self.port,
                "baudrate": self.baudrate,
                "data_timeout_s": self.data_timeout_s,
                "output_frame": "FLU (x forward, y left, z up)",
                "strict_policy_health": self.strict_policy_health,
                "last_error": self._last_error,
                "serial_error_count": self._serial_error_count,
                "parse_error_count": self._parse_error_count,
                "frame_error_count": self._parser.frame_error_count,
                "crc_error_count": self._parser.crc_error_count,
                "header_crc_error_count": (
                    self._parser.header_crc_error_count
                ),
                "payload_crc_error_count": (
                    self._parser.payload_crc_error_count
                ),
                "discarded_byte_count": self._parser.discarded_byte_count,
                "ground_candidate_count": self._parser.ground_candidate_count,
                "unsupported_packet_count": self._unsupported_packet_count,
                "imu_packet_count": self._packet_counts.get(TYPE_IMU, 0),
                "ahrs_packet_count": self._packet_counts.get(TYPE_AHRS, 0),
                "insgps_packet_count": self._packet_counts.get(TYPE_INSGPS, 0),
                "geodetic_packet_count": self._packet_counts.get(
                    TYPE_GEODETIC_POS, 0
                ),
                "system_state_packet_count": self._packet_counts.get(
                    TYPE_SYS_STATE, 0
                ),
                "ground_packet_count": self._packet_counts.get(TYPE_GROUND, 0),
                "imu_fresh": imu_fresh,
                "ahrs_fresh": ahrs_fresh,
                "insgps_fresh": insgps_fresh,
                "policy_data_ready": policy_ready,
                "velocity_available": policy_ready,
                "velocity_source": "integrated_imu",
                "velocity_world": tuple(self._velocity_world),
                "linear_velocity_body": tuple(self._linear_velocity),
                "linear_acceleration": tuple(
                    self._linear_acceleration_body
                ),
                "linear_acceleration_body": tuple(
                    self._linear_acceleration_body
                ),
                "linear_acceleration_world": tuple(
                    self._linear_acceleration_world
                ),
                "calibrated": self._calibrated,
                "velocity_valid": self._velocity_valid,
                "stationary": self._stationary,
                "stationary_elapsed_s": self._stationary_elapsed_s,
                "calibration_sample_count": (
                    self._calibration_sample_count
                ),
                "accelerometer_rest_sign": self.accelerometer_rest_sign,
                "accel_bias_flu": tuple(self._accel_bias_flu),
                "gyro_bias_flu": tuple(self._gyro_bias_flu),
                "velocity_dt_s": (
                    0.0
                    if self._velocity_dt_s is None
                    else self._velocity_dt_s
                ),
                "velocity_host_dt_s": (
                    0.0
                    if self._velocity_host_dt_s is None
                    else self._velocity_host_dt_s
                ),
                "velocity_dt_source": self._velocity_dt_source,
                "velocity_dt_reject_count": (
                    self._velocity_dt_reject_count
                ),
                "imu_timestamp_us": self._last_imu_timestamp_us,
                "timestamp_stall_count": self._timestamp_stall_count,
                "timestamp_stall_consecutive": (
                    self._timestamp_stall_consecutive
                ),
                "timestamp_stall_limit": _TIMESTAMP_STALL_LIMIT,
                "zupt_count": self._zupt_count,
                "velocity_estimator_error": (
                    self._velocity_estimator_error
                ),
                "gravity_available": (
                    self._ahrs_data is not None and ahrs_fresh
                ),
                "crc_validation_supported": True,
                "crc_validation_enabled": True,
                "crc_note": (
                    "CRC-8/MAXIM-DOW over the first four header bytes; "
                    "CRC-16/XMODEM over the payload; parameters verified "
                    "against a complete DETA40 hardware capture"
                ),
                "last_headers": {
                    packet_type: dict(header)
                    for packet_type, header in self._last_headers.items()
                },
            }
            diagnostics.update(ages)
            diagnostics.update(stream_timing)
            return diagnostics

    def _receive_loop(
        self, serial_port: Any, stop_event: threading.Event
    ) -> None:
        try:
            while not stop_event.is_set():
                try:
                    waiting = int(getattr(serial_port, "in_waiting", 0) or 0)
                    read_size = min(max(waiting, 1), 4096)
                    chunk = serial_port.read(read_size)
                except Exception as exc:
                    if stop_event.is_set():
                        break
                    # Publish terminal intent before taking the state lock so a
                    # concurrent start() cannot mistake this generation for a
                    # healthy, idempotently running receiver.
                    stop_event.set()
                    with self._state_lock:
                        self._serial_error_count += 1
                        self._last_error = "serial read failed: {}".format(exc)
                    break

                if not chunk:
                    continue

                for header, payload in self._parser.feed(chunk):
                    self._handle_packet(header, payload)
        finally:
            stop_event.set()
            try:
                serial_port.close()
            except Exception:
                pass
            with self._state_lock:
                if threading.current_thread() is self._thread:
                    self._running = False

    def _handle_packet(
        self, header: Mapping[str, int], payload: bytes
    ) -> None:
        packet_type = header["type"]
        now = time.monotonic()
        parsed: Optional[Dict[str, Any]] = None
        gravity: Optional[Vector3] = None
        try:
            if packet_type == TYPE_IMU:
                parsed = self._parse_imu(payload, header, now)
            elif packet_type == TYPE_AHRS:
                parsed, gravity = self._parse_ahrs(payload, header, now)
            elif packet_type == TYPE_INSGPS:
                parsed, _ = self._parse_insgps(payload, header, now)
            elif packet_type == TYPE_GEODETIC_POS:
                parsed = self._parse_geodetic(payload, header, now)
            elif packet_type == TYPE_SYS_STATE:
                # The vendor demo's System_status unpack is internally
                # inconsistent (2-byte slice passed to an 8-byte double).
                # Preserve the bytes without inventing a field layout.
                parsed = self._packet_metadata(header, now)
                parsed["raw_payload"] = bytes(payload)
            elif packet_type == TYPE_GROUND:
                pass
            else:
                with self._state_lock:
                    self._unsupported_packet_count += 1
                return
        except (ValueError, struct.error, OverflowError) as exc:
            with self._state_lock:
                self._parse_error_count += 1
                self._last_error = "packet 0x{:02x} parse failed: {}".format(
                    packet_type, exc
                )
            return

        with self._state_lock:
            if packet_type == TYPE_IMU:
                self._imu_data = parsed
            elif packet_type == TYPE_AHRS:
                self._ahrs_data = parsed
                if gravity is not None:
                    self._gravity_flu = gravity
            elif packet_type == TYPE_INSGPS:
                self._insgps_data = parsed
            elif packet_type == TYPE_GEODETIC_POS:
                self._geodetic_data = parsed
            elif packet_type == TYPE_SYS_STATE:
                self._system_state = parsed
            self._packet_counts[packet_type] = (
                self._packet_counts.get(packet_type, 0) + 1
            )
            self._last_headers[packet_type] = dict(header)
            previous_packet_time = self._last_packet_monotonic.get(packet_type)
            if (
                packet_type in _POLICY_STREAM_NAMES
                and previous_packet_time is not None
            ):
                interarrival_s = max(0.0, now - previous_packet_time)
                self._last_interarrival_s[packet_type] = interarrival_s
                previous_max = self._max_interarrival_s[packet_type]
                if previous_max is None or interarrival_s > previous_max:
                    self._max_interarrival_s[packet_type] = interarrival_s
                if interarrival_s > self.data_timeout_s:
                    self._gap_over_timeout_count[packet_type] += 1
            self._last_packet_monotonic[packet_type] = now
            if packet_type == TYPE_IMU and parsed is not None:
                self._update_velocity_estimator_locked(parsed, now)
            elif (
                packet_type == TYPE_AHRS
                and self._calibrated
                and self._velocity_valid
            ):
                rotation = self._attitude_rotation_locked()
                if rotation is not None:
                    self._linear_velocity = _rotate_transpose(
                        rotation, self._velocity_world
                    )

    def _policy_ready_locked(self, now: float) -> bool:
        receiver_healthy = self._running and not self._stop_event.is_set()
        imu_fresh = self._is_fresh(
            now, self._last_packet_monotonic.get(TYPE_IMU)
        )
        ahrs_fresh = self._is_fresh(
            now, self._last_packet_monotonic.get(TYPE_AHRS)
        )
        return bool(
            receiver_healthy
            and self._imu_data is not None
            and self._ahrs_data is not None
            and imu_fresh
            and ahrs_fresh
            and self._calibrated
            and self._velocity_valid
        )

    def _attitude_rotation_locked(self) -> Optional[Matrix3]:
        if self._ahrs_data is None:
            return None
        angles = (
            float(self._ahrs_data["roll"]),
            float(self._ahrs_data["pitch"]),
            float(self._ahrs_data["heading"]),
        )
        if not _all_finite(angles):
            return None
        return _rotation_body_to_world(*angles)

    def _reset_calibration_window_locked(self) -> None:
        self._calibration_accel_bias_sum = [0.0, 0.0, 0.0]
        self._calibration_gyro_bias_sum = [0.0, 0.0, 0.0]
        self._calibration_accel_bias_sq_sum = [0.0, 0.0, 0.0]
        self._calibration_gyro_bias_sq_sum = [0.0, 0.0, 0.0]
        self._calibration_sample_count = 0
        self._calibration_stationary_since = None
        self._stationary_since = None
        self._stationary_elapsed_s = 0.0
        self._stationary = False

    def _invalidate_velocity_estimator_locked(self, reason: str) -> None:
        self._calibrated = False
        self._velocity_valid = False
        self._velocity_world = _ZERO_VECTOR
        self._linear_velocity = _ZERO_VECTOR
        self._linear_acceleration_body = _ZERO_VECTOR
        self._linear_acceleration_world = _ZERO_VECTOR
        self._accel_bias_flu = _ZERO_VECTOR
        self._gyro_bias_flu = _ZERO_VECTOR
        self._previous_acceleration_world = None
        self._last_imu_timestamp_us = None
        self._velocity_dt_s = None
        self._velocity_host_dt_s = None
        self._timestamp_stall_consecutive = 0
        self._velocity_estimator_error = reason
        self._reset_calibration_window_locked()

    def _update_velocity_estimator_locked(
        self, imu_data: Mapping[str, Any], now: float
    ) -> None:
        """Update the inertial velocity estimate from one parsed 0x40 sample."""

        ahrs_timestamp = self._last_packet_monotonic.get(TYPE_AHRS)
        rotation = self._attitude_rotation_locked()
        if not self._is_fresh(now, ahrs_timestamp) or rotation is None:
            if self._calibrated:
                self._invalidate_velocity_estimator_locked(
                    "AHRS unavailable or stale during velocity integration"
                )
            else:
                self._reset_calibration_window_locked()
            self._last_estimator_update_monotonic = None
            self._last_imu_timestamp_us = None
            self._velocity_dt_s = None
            self._velocity_host_dt_s = None
            return

        acceleration = (
            float(imu_data["acc_x"]),
            float(imu_data["acc_y"]),
            float(imu_data["acc_z"]),
        )
        angular_velocity = (
            float(imu_data["gyro_x"]),
            float(imu_data["gyro_y"]),
            float(imu_data["gyro_z"]),
        )
        if not _all_finite(acceleration + angular_velocity):
            self._invalidate_velocity_estimator_locked(
                "non-finite IMU value during velocity integration"
            )
            self._last_estimator_update_monotonic = None
            self._last_imu_timestamp_us = None
            self._velocity_dt_s = None
            self._velocity_host_dt_s = None
            return

        previous_time = self._last_estimator_update_monotonic
        host_dt = None if previous_time is None else now - previous_time
        self._last_estimator_update_monotonic = now
        self._velocity_host_dt_s = host_dt

        # The vendor demo labels the first 32-bit timestamp word as
        # ``Timestamp(us)``.  The real basic DETA40 advances it by exactly
        # 10000 us at 100 Hz.  Use modulo arithmetic so signed decoding and
        # the 32-bit wrap do not disturb integration.  Host time remains a
        # watchdog for a stalled receiver and a fallback for synthetic input.
        timestamp_word0 = imu_data.get("timestamp_word0")
        sensor_timestamp_us: Optional[int]
        if isinstance(timestamp_word0, Integral) and not isinstance(
            timestamp_word0, bool
        ):
            sensor_timestamp_us = int(timestamp_word0) & 0xFFFFFFFF
        else:
            sensor_timestamp_us = None
        previous_sensor_timestamp_us = self._last_imu_timestamp_us
        self._last_imu_timestamp_us = sensor_timestamp_us
        if (
            sensor_timestamp_us is not None
            and previous_sensor_timestamp_us is not None
        ):
            sensor_delta_us = (
                sensor_timestamp_us - previous_sensor_timestamp_us
            ) & 0xFFFFFFFF
            dt: Optional[float] = sensor_delta_us * 1.0e-6
            self._velocity_dt_source = "vendor_timestamp_word0_us"
            if sensor_delta_us == 0:
                self._timestamp_stall_count += 1
                self._timestamp_stall_consecutive += 1
            else:
                self._timestamp_stall_consecutive = 0
        elif sensor_timestamp_us is not None:
            dt = None
            self._velocity_dt_source = "vendor_timestamp_word0_us"
            self._timestamp_stall_consecutive = 0
        else:
            dt = host_dt
            self._velocity_dt_source = "host_monotonic_fallback"
            self._timestamp_stall_consecutive = 0
        self._velocity_dt_s = dt
        if self._timestamp_stall_consecutive >= _TIMESTAMP_STALL_LIMIT:
            stalled_packets = self._timestamp_stall_consecutive
            self._velocity_dt_reject_count += 1
            self._invalidate_velocity_estimator_locked(
                "vendor IMU timestamp stalled for {} packets".format(
                    stalled_packets
                )
            )
            self._last_estimator_update_monotonic = now
            self._velocity_host_dt_s = host_dt
            return
        integration_gap = (
            (dt is not None and dt > self.max_integration_dt_s)
            or (
                host_dt is not None
                and host_dt > self.max_integration_dt_s
            )
        )
        if integration_gap:
            self._velocity_dt_reject_count += 1
            self._invalidate_velocity_estimator_locked(
                "IMU integration gap sensor={}s host={}s exceeds "
                "{:.6f}s".format(
                    "n/a" if dt is None else "{:.6f}".format(dt),
                    (
                        "n/a"
                        if host_dt is None
                        else "{:.6f}".format(host_dt)
                    ),
                    self.max_integration_dt_s,
                )
            )
            self._last_estimator_update_monotonic = now
            self._last_imu_timestamp_us = sensor_timestamp_us
            self._velocity_host_dt_s = host_dt
            self._velocity_dt_source = (
                "vendor_timestamp_word0_us"
                if sensor_timestamp_us is not None
                else "host_monotonic_fallback"
            )
            dt = None
            self._velocity_dt_s = None
        elif dt is not None and dt <= 0.0:
            self._velocity_dt_reject_count += 1
            dt = None
            self._velocity_dt_s = None

        gravity_unit_body = _rotate_transpose(
            rotation, _LEVEL_GRAVITY_FLU
        )
        gravity_body_mps2 = _rotate_transpose(
            rotation, _WORLD_GRAVITY_FLU_MPS2
        )
        configured_sign = self._configured_accelerometer_rest_sign
        if configured_sign == 0 and not self._calibrated:
            alignment = sum(
                acceleration[index] * gravity_unit_body[index]
                for index in range(3)
            )
            detected_sign = 1 if alignment >= 0.0 else -1
            if self.accelerometer_rest_sign not in (0, detected_sign):
                self._reset_calibration_window_locked()
            self.accelerometer_rest_sign = detected_sign
        elif configured_sign != 0:
            self.accelerometer_rest_sign = configured_sign

        rest_sign = self.accelerometer_rest_sign
        if rest_sign == 0:
            self._reset_calibration_window_locked()
            return
        expected_rest_acceleration = tuple(
            rest_sign * value for value in gravity_body_mps2
        )  # type: ignore[assignment]

        if not self._calibrated:
            accel_magnitude_error = abs(
                _norm(acceleration) - _GRAVITY_MPS2
            )
            gyro_magnitude = _norm(angular_velocity)
            calibration_stationary = (
                accel_magnitude_error
                <= self.stationary_accel_threshold_mps2
                and gyro_magnitude
                <= self.stationary_gyro_threshold_rad_s
            )
            if not calibration_stationary:
                self._reset_calibration_window_locked()
                return

            if self._calibration_stationary_since is None:
                self._calibration_stationary_since = now
            self._stationary_elapsed_s = max(
                0.0, now - self._calibration_stationary_since
            )
            accel_bias_sample = tuple(
                acceleration[index] - expected_rest_acceleration[index]
                for index in range(3)
            )
            for index in range(3):
                self._calibration_accel_bias_sum[index] += (
                    accel_bias_sample[index]
                )
                self._calibration_gyro_bias_sum[index] += (
                    angular_velocity[index]
                )
                self._calibration_accel_bias_sq_sum[index] += (
                    accel_bias_sample[index] * accel_bias_sample[index]
                )
                self._calibration_gyro_bias_sq_sum[index] += (
                    angular_velocity[index] * angular_velocity[index]
                )
            self._calibration_sample_count += 1
            if (
                self._stationary_elapsed_s
                < self.stationary_calibration_s
            ):
                return

            sample_count = float(self._calibration_sample_count)
            self._accel_bias_flu = tuple(
                total / sample_count
                for total in self._calibration_accel_bias_sum
            )  # type: ignore[assignment]
            self._gyro_bias_flu = tuple(
                total / sample_count
                for total in self._calibration_gyro_bias_sum
            )  # type: ignore[assignment]
            accel_std = tuple(
                math.sqrt(
                    max(
                        0.0,
                        self._calibration_accel_bias_sq_sum[index]
                        / sample_count
                        - self._accel_bias_flu[index]
                        * self._accel_bias_flu[index],
                    )
                )
                for index in range(3)
            )
            gyro_std = tuple(
                math.sqrt(
                    max(
                        0.0,
                        self._calibration_gyro_bias_sq_sum[index]
                        / sample_count
                        - self._gyro_bias_flu[index]
                        * self._gyro_bias_flu[index],
                    )
                )
                for index in range(3)
            )
            if (
                max(accel_std) > self.stationary_accel_threshold_mps2
                or max(gyro_std)
                > self.stationary_gyro_threshold_rad_s
            ):
                self._accel_bias_flu = _ZERO_VECTOR
                self._gyro_bias_flu = _ZERO_VECTOR
                self._reset_calibration_window_locked()
                return
            self._calibrated = True
            self._velocity_valid = True
            self._stationary = True
            self._stationary_since = self._calibration_stationary_since
            self._velocity_world = _ZERO_VECTOR
            self._linear_velocity = _ZERO_VECTOR
            self._linear_acceleration_body = _ZERO_VECTOR
            self._linear_acceleration_world = _ZERO_VECTOR
            self._previous_acceleration_world = _ZERO_VECTOR
            self._velocity_estimator_error = None
            return

        corrected_force: Vector3 = (
            acceleration[0] - self._accel_bias_flu[0],
            acceleration[1] - self._accel_bias_flu[1],
            acceleration[2] - self._accel_bias_flu[2],
        )
        corrected_angular_velocity: Vector3 = (
            angular_velocity[0] - self._gyro_bias_flu[0],
            angular_velocity[1] - self._gyro_bias_flu[1],
            angular_velocity[2] - self._gyro_bias_flu[2],
        )
        # ``rest_sign`` also identifies the accelerometer convention.  A
        # conventional specific-force sensor has sign -1 and reports
        # ``f = a - g``; the opposite convention has sign +1 and reports
        # ``-f``.  Multiplying the gravity-removed delta by ``-rest_sign``
        # therefore gives physical body acceleration for both conventions.
        linear_acceleration_body: Vector3 = (
            -rest_sign
            * (corrected_force[0] - expected_rest_acceleration[0]),
            -rest_sign
            * (corrected_force[1] - expected_rest_acceleration[1]),
            -rest_sign
            * (corrected_force[2] - expected_rest_acceleration[2]),
        )
        linear_acceleration_world = _rotate(
            rotation, linear_acceleration_body
        )
        if not _all_finite(
            linear_acceleration_body + linear_acceleration_world
        ):
            self._invalidate_velocity_estimator_locked(
                "non-finite gravity-compensated acceleration"
            )
            self._last_estimator_update_monotonic = now
            return
        self._linear_acceleration_body = linear_acceleration_body
        self._linear_acceleration_world = linear_acceleration_world

        stationary_measurement = (
            _norm(linear_acceleration_body)
            <= self.stationary_accel_threshold_mps2
            and _norm(corrected_angular_velocity)
            <= self.stationary_gyro_threshold_rad_s
        )
        if stationary_measurement:
            if self._stationary_since is None:
                self._stationary_since = now
            self._stationary_elapsed_s = max(
                0.0, now - self._stationary_since
            )
            if (
                self.stationary_hold_s > 0.0
                and self._stationary_elapsed_s >= self.stationary_hold_s
            ):
                if not self._stationary:
                    self._zupt_count += 1
                self._stationary = True
                self._velocity_world = _ZERO_VECTOR
                self._linear_velocity = _ZERO_VECTOR
                self._previous_acceleration_world = (
                    linear_acceleration_world
                )
                self._velocity_valid = True
                self._velocity_estimator_error = None
                return
            if self.stationary_hold_s == 0.0:
                self._stationary = False
        else:
            self._stationary_since = None
            self._stationary_elapsed_s = 0.0
            self._stationary = False

        previous_acceleration = self._previous_acceleration_world
        self._previous_acceleration_world = linear_acceleration_world
        if dt is None or previous_acceleration is None:
            self._velocity_valid = True
            return

        next_velocity_world: Vector3 = tuple(
            self._velocity_world[index]
            + 0.5
            * (previous_acceleration[index] + linear_acceleration_world[index])
            * dt
            for index in range(3)
        )  # type: ignore[assignment]
        if not _all_finite(next_velocity_world):
            self._invalidate_velocity_estimator_locked(
                "non-finite integrated velocity"
            )
            self._last_estimator_update_monotonic = now
            return
        self._velocity_world = next_velocity_world
        self._linear_velocity = _rotate_transpose(
            rotation, self._velocity_world
        )
        self._velocity_valid = True
        self._velocity_estimator_error = None

    @staticmethod
    def _packet_metadata(
        header: Mapping[str, int], host_monotonic: float
    ) -> Dict[str, Any]:
        return {
            "sequence": int(header["sequence"]),
            "header_crc8": int(header["crc8"]),
            "payload_crc16_high": int(header["crc16_high"]),
            "payload_crc16_low": int(header["crc16_low"]),
            "host_monotonic_s": float(host_monotonic),
        }

    def _parse_imu(
        self, payload: bytes, header: Mapping[str, int], now: float
    ) -> Dict[str, Any]:
        values = struct.unpack_from("<12f", payload, 0)
        gyro_raw = values[0:3]
        accel_raw = values[3:6]
        if not _all_finite(gyro_raw + accel_raw):
            raise ValueError("non-finite gyro/accelerometer value")

        gyro = _frd_to_flu(*gyro_raw)
        accel = _frd_to_flu(*accel_raw)
        magnetometer = _frd_to_flu(*values[6:9])
        timestamp_word0, timestamp_word1 = struct.unpack_from(
            "<ii", payload, 48
        )

        result = self._packet_metadata(header, now)
        result.update(
            {
                "gyro_x": gyro[0],
                "gyro_y": gyro[1],
                "gyro_z": gyro[2],
                "acc_x": accel[0],
                "acc_y": accel[1],
                "acc_z": accel[2],
                "mag_x": magnetometer[0],
                "mag_y": magnetometer[1],
                "mag_z": magnetometer[2],
                "imu_temperature": float(values[9]),
                "pressure": float(values[10]),
                "pressure_temperature": float(values[11]),
                "vendor_gyro_x": float(gyro_raw[0]),
                "vendor_gyro_y": float(gyro_raw[1]),
                "vendor_gyro_z": float(gyro_raw[2]),
                "vendor_acc_x": float(accel_raw[0]),
                "vendor_acc_y": float(accel_raw[1]),
                "vendor_acc_z": float(accel_raw[2]),
                "timestamp_word0": int(timestamp_word0),
                "timestamp_word1": int(timestamp_word1),
                "timestamp_word0_u32_us": int(timestamp_word0) & 0xFFFFFFFF,
            }
        )
        return result

    def _parse_ahrs(
        self, payload: bytes, header: Mapping[str, int], now: float
    ) -> Tuple[Dict[str, Any], Vector3]:
        values = struct.unpack_from("<10f", payload, 0)
        rates_raw = values[0:3]
        roll_raw, pitch_raw, heading_raw = values[3:6]
        if not _all_finite(values[0:6]):
            raise ValueError("non-finite AHRS rate/Euler value")

        # Conjugating a ZYX attitude by Rx(pi), i.e. converting both the body
        # and navigation axes from FRD/NED-style axes to FLU/z-up-style axes,
        # yields roll'=roll, pitch'=-pitch, heading'=-heading.
        rates_flu = _frd_to_flu(*rates_raw)
        roll_flu = float(roll_raw)
        pitch_flu = -float(pitch_raw)
        heading_flu = -float(heading_raw)

        sin_roll = math.sin(roll_flu)
        cos_roll = math.cos(roll_flu)
        sin_pitch = math.sin(pitch_flu)
        cos_pitch = math.cos(pitch_flu)
        gravity = (
            sin_pitch,
            -sin_roll * cos_pitch,
            -cos_roll * cos_pitch,
        )

        timestamp_word0, timestamp_word1 = struct.unpack_from(
            "<ii", payload, 40
        )
        result = self._packet_metadata(header, now)
        result.update(
            {
                "roll_speed": rates_flu[0],
                "pitch_speed": rates_flu[1],
                "heading_speed": rates_flu[2],
                "roll": roll_flu,
                "pitch": pitch_flu,
                "heading": heading_flu,
                # The demo does not define quaternion order/direction.  Keep
                # these names raw and do not alias them to qw/qx/qy/qz.
                "q1": float(values[6]),
                "q2": float(values[7]),
                "q3": float(values[8]),
                "q4": float(values[9]),
                "vendor_roll_speed": float(rates_raw[0]),
                "vendor_pitch_speed": float(rates_raw[1]),
                "vendor_heading_speed": float(rates_raw[2]),
                "vendor_roll": float(roll_raw),
                "vendor_pitch": float(pitch_raw),
                "vendor_heading": float(heading_raw),
                "timestamp_word0": int(timestamp_word0),
                "timestamp_word1": int(timestamp_word1),
                "timestamp_word0_u32_us": int(timestamp_word0) & 0xFFFFFFFF,
            }
        )
        return result, gravity

    def _parse_insgps(
        self, payload: bytes, header: Mapping[str, int], now: float
    ) -> Tuple[Dict[str, Any], Vector3]:
        values = struct.unpack_from("<16f", payload, 0)
        body_velocity_raw = values[0:3]
        body_acceleration_raw = values[3:6]
        if not _all_finite(body_velocity_raw + body_acceleration_raw):
            raise ValueError("non-finite body velocity/acceleration value")

        body_velocity = _frd_to_flu(*body_velocity_raw)
        body_acceleration = _frd_to_flu(*body_acceleration_raw)
        timestamp_word0, timestamp_word1 = struct.unpack_from(
            "<ii", payload, 64
        )

        result = self._packet_metadata(header, now)
        result.update(
            {
                "body_velocity_x": body_velocity[0],
                "body_velocity_y": body_velocity[1],
                "body_velocity_z": body_velocity[2],
                "body_acceleration_x": body_acceleration[0],
                "body_acceleration_y": body_acceleration[1],
                "body_acceleration_z": body_acceleration[2],
                "vendor_body_velocity_x": float(body_velocity_raw[0]),
                "vendor_body_velocity_y": float(body_velocity_raw[1]),
                "vendor_body_velocity_z": float(body_velocity_raw[2]),
                "vendor_body_acceleration_x": float(body_acceleration_raw[0]),
                "vendor_body_acceleration_y": float(body_acceleration_raw[1]),
                "vendor_body_acceleration_z": float(body_acceleration_raw[2]),
                # Navigation-frame N/E/D fields are preserved, not subjected
                # to the body FRD->FLU mapping.
                "location_north": float(values[6]),
                "location_east": float(values[7]),
                "location_down": float(values[8]),
                "velocity_north": float(values[9]),
                "velocity_east": float(values[10]),
                "velocity_down": float(values[11]),
                "acceleration_north": float(values[12]),
                "acceleration_east": float(values[13]),
                "acceleration_down": float(values[14]),
                "pressure_altitude": float(values[15]),
                "timestamp_word0": int(timestamp_word0),
                "timestamp_word1": int(timestamp_word1),
            }
        )
        return result, body_velocity

    def _parse_geodetic(
        self, payload: bytes, header: Mapping[str, int], now: float
    ) -> Dict[str, Any]:
        latitude, longitude, height = struct.unpack_from("<3d", payload, 0)
        if not _all_finite((latitude, longitude, height)):
            raise ValueError("non-finite geodetic value")
        result = self._packet_metadata(header, now)
        result.update(
            {
                "latitude": float(latitude),
                "longitude": float(longitude),
                "height": float(height),
                # The demo does not name bytes 24:32.
                "trailing_bytes": bytes(payload[24:32]),
            }
        )
        return result

    def _is_fresh(self, now: float, timestamp: Optional[float]) -> bool:
        return timestamp is not None and now - timestamp <= self.data_timeout_s

    def _set_start_error(self, message: str) -> None:
        with self._state_lock:
            self._running = False
            self._last_error = message

    def _reset_runtime_state(self) -> None:
        self._parser.reset()
        with self._state_lock:
            self._running = False
            self._last_error = None
            self._serial_error_count = 0
            self._parse_error_count = 0
            self._unsupported_packet_count = 0
            for packet_type in list(self._packet_counts):
                self._packet_counts[packet_type] = 0
            self._last_headers.clear()
            self._last_packet_monotonic.clear()
            for packet_type in _POLICY_STREAM_NAMES:
                self._last_interarrival_s[packet_type] = None
                self._max_interarrival_s[packet_type] = None
                self._gap_over_timeout_count[packet_type] = 0
            self._imu_data = None
            self._ahrs_data = None
            self._insgps_data = None
            self._geodetic_data = None
            self._system_state = None
            self._linear_velocity = _ZERO_VECTOR
            self._gravity_flu = _LEVEL_GRAVITY_FLU
            self._linear_acceleration_body = _ZERO_VECTOR
            self._linear_acceleration_world = _ZERO_VECTOR
            self._velocity_world = _ZERO_VECTOR
            self._accel_bias_flu = _ZERO_VECTOR
            self._gyro_bias_flu = _ZERO_VECTOR
            self._calibration_accel_bias_sum = [0.0, 0.0, 0.0]
            self._calibration_gyro_bias_sum = [0.0, 0.0, 0.0]
            self._calibration_accel_bias_sq_sum = [0.0, 0.0, 0.0]
            self._calibration_gyro_bias_sq_sum = [0.0, 0.0, 0.0]
            self._calibration_sample_count = 0
            self._calibration_stationary_since = None
            self._stationary_since = None
            self._stationary_elapsed_s = 0.0
            self._calibrated = False
            self._stationary = False
            self._velocity_valid = False
            self._previous_acceleration_world = None
            self._last_estimator_update_monotonic = None
            self._last_imu_timestamp_us = None
            self._velocity_dt_s = None
            self._velocity_host_dt_s = None
            self._velocity_dt_source = "vendor_timestamp_word0_us"
            self._velocity_dt_reject_count = 0
            self._timestamp_stall_count = 0
            self._timestamp_stall_consecutive = 0
            self._zupt_count = 0
            self._velocity_estimator_error = None
            self.accelerometer_rest_sign = (
                self._configured_accelerometer_rest_sign
            )

    def __del__(self) -> None:
        try:
            self.stop()
        except Exception:
            pass


__all__ = ["IMUSDK", "calc_crc8", "calc_crc16"]
