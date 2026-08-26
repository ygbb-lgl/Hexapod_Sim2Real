#!/usr/bin/env python3
"""Live gravity compensation and IMU-only velocity validation for DETA40."""

import argparse
import time

try:
    from .imu_sdk import IMUSDK
except ImportError:
    from imu_sdk import IMUSDK


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--baudrate", type=int, default=921600)
    parser.add_argument("--rate", type=float, default=10.0, help="print rate in Hz")
    parser.add_argument(
        "--calibration-seconds",
        type=float,
        default=1.0,
        help="startup stationary accelerometer-bias calibration duration",
    )
    parser.add_argument(
        "--accelerometer-rest-sign",
        type=int,
        choices=(-1, 0, 1),
        default=0,
        help="0=auto, +1=stationary reading follows gravity, -1=specific force",
    )
    parser.add_argument(
        "--stationary-accel-threshold",
        type=float,
        default=0.12,
        help="ZUPT acceleration threshold in m/s^2",
    )
    parser.add_argument(
        "--stationary-gyro-threshold",
        type=float,
        default=0.15,
        help="ZUPT angular-speed threshold in rad/s",
    )
    parser.add_argument(
        "--stationary-hold",
        type=float,
        default=0.25,
        help="quiet duration before zeroing velocity; 0 disables ZUPT",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.rate <= 0.0:
        raise ValueError("--rate must be positive")

    imu = IMUSDK(
        port=args.port,
        baudrate=args.baudrate,
        stationary_calibration_s=args.calibration_seconds,
        accelerometer_rest_sign=args.accelerometer_rest_sign,
        stationary_accel_threshold_mps2=args.stationary_accel_threshold,
        stationary_gyro_threshold_rad_s=args.stationary_gyro_threshold,
        stationary_hold_s=args.stationary_hold,
    )
    print("Connecting to DETA40 on {} ...".format(args.port))
    if not imu.start():
        return 1

    print("Keep the IMU still during startup calibration.")
    print("Output frame: X forward, Y left, Z up (FLU)")
    print("A level IMU should report gravity approximately [0, 0, -1].")
    print("Press Ctrl+C to stop.\n")

    try:
        while True:
            velocity = imu.get_linear_velocity()
            gravity = imu.get_gravity_acceleration()
            diagnostics = imu.get_diagnostics()
            linear_acceleration = diagnostics["linear_acceleration"]

            print(
                "Vel FLU [m/s]: [{:+7.3f}, {:+7.3f}, {:+7.3f}] | "
                "Grav FLU [unit]: [{:+7.3f}, {:+7.3f}, {:+7.3f}] | "
                "Linear acc FLU [m/s^2]: [{:+7.3f}, {:+7.3f}, {:+7.3f}] | "
                "calibrated={} stationary={} quiet={:.2f}s calib_n={} "
                "g_sign={:+d} dt={:.4f}s "
                "packets={}/{} errors={}".format(
                    velocity[0],
                    velocity[1],
                    velocity[2],
                    gravity[0],
                    gravity[1],
                    gravity[2],
                    linear_acceleration[0],
                    linear_acceleration[1],
                    linear_acceleration[2],
                    diagnostics["calibrated"],
                    diagnostics["stationary"],
                    diagnostics["stationary_elapsed_s"],
                    diagnostics["calibration_sample_count"],
                    diagnostics["accelerometer_rest_sign"],
                    diagnostics["velocity_dt_s"],
                    diagnostics["imu_packet_count"],
                    diagnostics["ahrs_packet_count"],
                    diagnostics["crc_error_count"]
                    + diagnostics["frame_error_count"],
                )
            )
            time.sleep(1.0 / args.rate)
    except KeyboardInterrupt:
        print("\nStopping ...")
    finally:
        imu.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
