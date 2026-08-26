#!/usr/bin/env python3
"""Live DETA40 IMU/AHRS validation without starting the robot controller."""

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
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.rate <= 0.0:
        raise ValueError("--rate must be positive")

    imu = IMUSDK(port=args.port, baudrate=args.baudrate)
    print("Connecting to DETA40 on {} ...".format(args.port))
    if not imu.start():
        return 1

    print("Output frame: X forward, Y left, Z up (FLU)")
    print("Press Ctrl+C to stop.\n")

    try:
        while True:
            imu_data = imu.get_imu_data()
            ahrs_data = imu.get_ahrs_data()
            diagnostics = imu.get_diagnostics()

            if imu_data is None or ahrs_data is None:
                print(
                    "Waiting for IMU/AHRS packets ... "
                    "imu={} ahrs={} crc_errors={}".format(
                        diagnostics["imu_packet_count"],
                        diagnostics["ahrs_packet_count"],
                        diagnostics["crc_error_count"],
                    )
                )
            else:
                print(
                    "Gyro FLU [rad/s]: [{:+8.4f}, {:+8.4f}, {:+8.4f}] | "
                    "Accel+g FLU [m/s^2]: [{:+8.4f}, {:+8.4f}, {:+8.4f}]".format(
                        imu_data["gyro_x"],
                        imu_data["gyro_y"],
                        imu_data["gyro_z"],
                        imu_data["acc_x"],
                        imu_data["acc_y"],
                        imu_data["acc_z"],
                    )
                )
                print(
                    "Euler FLU [rad]: roll={:+8.4f}, pitch={:+8.4f}, "
                    "heading={:+8.4f} | Quaternion FLU [wxyz]: "
                    "[{:+8.4f}, {:+8.4f}, {:+8.4f}, {:+8.4f}]".format(
                        ahrs_data["roll"],
                        ahrs_data["pitch"],
                        ahrs_data["heading"],
                        ahrs_data["qw"],
                        ahrs_data["qx"],
                        ahrs_data["qy"],
                        ahrs_data["qz"],
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
