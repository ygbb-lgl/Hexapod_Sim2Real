import math
import os
import struct
import sys
import types
import unittest


sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# The estimator math tests never open a serial port. Keep them runnable in
# development environments that do not have pyserial installed.
try:
    import serial as _serial  # noqa: F401
except ModuleNotFoundError:
    serial_stub = types.ModuleType("serial")
    serial_stub.Serial = object
    serial_stub.SerialException = Exception
    sys.modules["serial"] = serial_stub

from imu_sdk import (  # noqa: E402
    AHRS_LEN,
    GRAVITY_ACCELERATION,
    IMU_LEN,
    INSGPS_LEN,
    TYPE_AHRS,
    TYPE_IMU,
    TYPE_INSGPS,
    IMUSDK,
)


class IMUIntegrationTest(unittest.TestCase):
    def assert_vector_almost_equal(self, actual, expected, places=7):
        self.assertEqual(len(actual), len(expected))
        for actual_value, expected_value in zip(actual, expected):
            self.assertAlmostEqual(actual_value, expected_value, places=places)

    @staticmethod
    def make_ahrs_payload(qw, qx, qy, qz, timestamp_us):
        payload = struct.pack(
            "<10fq",
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            qw,
            qx,
            qy,
            qz,
            timestamp_us,
        )
        assert len(payload) == AHRS_LEN
        return payload

    @staticmethod
    def make_imu_payload(acc_x, acc_y, acc_z, timestamp_us):
        payload = struct.pack(
            "<12fq",
            0.0,
            0.0,
            0.0,
            acc_x,
            acc_y,
            acc_z,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            timestamp_us,
        )
        assert len(payload) == IMU_LEN
        return payload

    def test_physical_gravity_compensation_uses_inverse_rotation(self):
        half_angle = math.radians(30.0) * 0.5
        q = (math.cos(half_angle), 0.0, math.sin(half_angle), 0.0)
        negative_q = tuple(-value for value in q)

        gravity_for_compensation = IMUSDK._compute_gravity_vec_for_comp(q)
        gravity_for_observation = IMUSDK._compute_gravity_vec_for_obs(q)
        gravity_from_negative_q = IMUSDK._compute_gravity_vec_for_comp(negative_q)

        self.assert_vector_almost_equal(
            gravity_for_compensation,
            (0.5, 0.0, -math.sqrt(3.0) * 0.5),
        )
        self.assert_vector_almost_equal(
            gravity_for_observation,
            (-0.5, 0.0, -math.sqrt(3.0) * 0.5),
        )
        self.assert_vector_almost_equal(
            gravity_from_negative_q,
            gravity_for_compensation,
        )

    def test_hardcoded_positive_pitch_gravity_sign(self):
        pitch = math.radians(30.0)
        q = (math.cos(pitch * 0.5), 0.0, math.sin(pitch * 0.5), 0.0)

        self.assert_vector_almost_equal(
            IMUSDK._compute_gravity_vec_for_obs(q),
            (-0.5, 0.0, -math.sqrt(3.0) * 0.5),
        )

    def test_stationary_pitch_does_not_create_horizontal_velocity(self):
        sdk = IMUSDK()
        half_angle = math.radians(25.0) * 0.5
        sdk._quat_urdf = (
            math.cos(half_angle),
            0.0,
            math.sin(half_angle),
            0.0,
        )
        sdk._specific_force_urdf = list(
            sdk._rotate_world_to_body(
                sdk._quat_urdf,
                (0.0, 0.0, GRAVITY_ACCELERATION),
            )
        )

        for timestamp_us in range(1_000_000, 1_110_000, 10_000):
            sdk._update_velocity_estimation_locked(timestamp_us)

        self.assert_vector_almost_equal(sdk._estimated_velocity_world, (0.0, 0.0, 0.0))
        self.assert_vector_almost_equal(sdk.get_linear_velocity(), (0.0, 0.0, 0.0))

    def test_constant_world_acceleration_uses_device_timestamp(self):
        sdk = IMUSDK()
        sdk._quat_urdf = (1.0, 0.0, 0.0, 0.0)
        sdk._specific_force_urdf = [1.0, 0.0, GRAVITY_ACCELERATION]

        for timestamp_us in range(1_000_000, 2_010_000, 10_000):
            sdk._update_velocity_estimation_locked(timestamp_us)

        self.assertAlmostEqual(sdk._estimated_velocity_world[0], 1.0, places=7)
        self.assertAlmostEqual(sdk._last_velocity_dt_s, 0.01, places=9)
        self.assertEqual(sdk._velocity_time_source, "device_timestamp")
        self.assertEqual(sdk._velocity_update_count, 100)

    def test_velocity_is_reexpressed_in_latest_body_frame(self):
        sdk = IMUSDK()
        sdk._estimated_velocity_world = [1.0, 0.0, 0.0]

        half_yaw = math.radians(90.0) * 0.5
        sdk._quat_urdf = (math.cos(half_yaw), 0.0, 0.0, math.sin(half_yaw))

        self.assert_vector_almost_equal(sdk.get_linear_velocity(), (0.0, -1.0, 0.0))
        self.assert_vector_almost_equal(sdk._estimated_velocity_world, (1.0, 0.0, 0.0))

    def test_only_new_imu_packets_advance_integrator(self):
        sdk = IMUSDK()
        sdk._parse_payload(
            TYPE_AHRS,
            self.make_ahrs_payload(1.0, 0.0, 0.0, 0.0, 1_000_000),
        )
        sdk._parse_payload(
            TYPE_IMU,
            self.make_imu_payload(1.0, 0.0, -GRAVITY_ACCELERATION, 1_000_000),
        )
        sdk._parse_payload(
            TYPE_IMU,
            self.make_imu_payload(1.0, 0.0, -GRAVITY_ACCELERATION, 1_010_000),
        )
        sdk._parse_payload(
            TYPE_AHRS,
            self.make_ahrs_payload(1.0, 0.0, 0.0, 0.0, 1_010_000),
        )

        self.assertEqual(sdk._velocity_update_count, 1)
        self.assertAlmostEqual(sdk.get_linear_velocity()[0], 0.01, places=7)

        sdk._parse_payload(
            TYPE_AHRS,
            self.make_ahrs_payload(1.0, 0.0, 0.0, 0.0, 1_010_000),
        )
        insgps_payload = struct.pack("<16fq", *([0.0] * 16), 1_010_000)
        self.assertEqual(len(insgps_payload), INSGPS_LEN)
        sdk._parse_payload(TYPE_INSGPS, insgps_payload)

        self.assertEqual(sdk._velocity_update_count, 1)
        self.assertAlmostEqual(sdk.get_linear_velocity()[0], 0.01, places=7)

    def test_stale_ahrs_rejects_imu_integration(self):
        sdk = IMUSDK()
        sdk._parse_payload(
            TYPE_AHRS,
            self.make_ahrs_payload(1.0, 0.0, 0.0, 0.0, 1_000_000),
        )
        sdk._parse_payload(
            TYPE_IMU,
            self.make_imu_payload(1.0, 0.0, -GRAVITY_ACCELERATION, 1_100_001),
        )

        self.assertEqual(sdk._velocity_update_count, 0)
        self.assertEqual(sdk.get_linear_velocity(), [0.0, 0.0, 0.0])
        self.assertGreater(sdk._velocity_rejected_sample_count, 0)

    def test_high_rate_imu_samples_are_interpolated_not_downsampled(self):
        sdk = IMUSDK()
        sdk._parse_payload(
            TYPE_AHRS,
            self.make_ahrs_payload(1.0, 0.0, 0.0, 0.0, 1_000_000),
        )
        sdk._parse_payload(
            TYPE_IMU,
            self.make_imu_payload(1.0, 0.0, -GRAVITY_ACCELERATION, 1_000_000),
        )
        for timestamp_us in range(1_002_000, 1_012_000, 2_000):
            sdk._parse_payload(
                TYPE_IMU,
                self.make_imu_payload(
                    1.0,
                    0.0,
                    -GRAVITY_ACCELERATION,
                    timestamp_us,
                ),
            )

        self.assertEqual(sdk._velocity_update_count, 0)
        self.assertEqual(len(sdk._pending_imu_samples), 5)

        sdk._parse_payload(
            TYPE_AHRS,
            self.make_ahrs_payload(1.0, 0.0, 0.0, 0.0, 1_010_000),
        )

        self.assertEqual(sdk._velocity_update_count, 5)
        self.assertEqual(len(sdk._pending_imu_samples), 0)
        self.assertAlmostEqual(sdk.get_linear_velocity()[0], 0.01, places=7)
        self.assertTrue(sdk.get_velocity_diagnostics()["velocity_estimator_valid"])

    def test_pitching_stationary_imu_uses_timestamp_interpolated_attitude(self):
        sdk = IMUSDK()
        sdk._parse_payload(
            TYPE_AHRS,
            self.make_ahrs_payload(1.0, 0.0, 0.0, 0.0, 1_000_000),
        )
        sdk._parse_payload(
            TYPE_IMU,
            self.make_imu_payload(0.0, 0.0, -GRAVITY_ACCELERATION, 1_000_000),
        )

        pitch_mid = math.radians(15.0)
        sdk._parse_payload(
            TYPE_IMU,
            self.make_imu_payload(
                -GRAVITY_ACCELERATION * math.sin(pitch_mid),
                0.0,
                -GRAVITY_ACCELERATION * math.cos(pitch_mid),
                1_005_000,
            ),
        )

        pitch_end = math.radians(30.0)
        sdk._parse_payload(
            TYPE_AHRS,
            self.make_ahrs_payload(
                math.cos(pitch_end * 0.5),
                0.0,
                -math.sin(pitch_end * 0.5),
                0.0,
                1_010_000,
            ),
        )

        self.assertEqual(sdk._velocity_update_count, 1)
        self.assert_vector_almost_equal(
            sdk._estimated_velocity_world,
            (0.0, 0.0, 0.0),
            places=6,
        )

    def test_stationary_reset_calibrates_body_accelerometer_bias(self):
        sdk = IMUSDK()
        sdk._quat_urdf = (1.0, 0.0, 0.0, 0.0)
        sdk._gyro_urdf = [0.0, 0.0, 0.0]
        expected_bias = [0.08, -0.04, 0.03]
        sdk._specific_force_urdf = [
            expected_bias[0],
            expected_bias[1],
            GRAVITY_ACCELERATION + expected_bias[2],
        ]

        for timestamp_us in range(1_000_000, 1_300_000, 10_000):
            sdk._update_velocity_estimation_locked(timestamp_us)

        self.assertTrue(sdk.reset_linear_velocity())
        self.assert_vector_almost_equal(
            sdk._accelerometer_bias_urdf,
            expected_bias,
            places=6,
        )

        for timestamp_us in range(2_000_000, 3_010_000, 10_000):
            sdk._update_velocity_estimation_locked(timestamp_us)

        self.assert_vector_almost_equal(
            sdk._estimated_velocity_world,
            (0.0, 0.0, 0.0),
            places=6,
        )


if __name__ == "__main__":
    unittest.main()
