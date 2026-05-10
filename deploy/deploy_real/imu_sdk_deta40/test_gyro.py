import os
import sys
import time

sys.path.append(os.path.dirname(os.path.abspath(__file__)))

try:
    from imu_sdk import IMUSDK
except ImportError:
    print("Failed to import IMUSDK from imu_sdk_deta40.")
    sys.exit(1)


def main():
    port = "/dev/ttyUSB0"
    if len(sys.argv) > 1:
        port = sys.argv[1]

    print(f"Connecting to IMU (deta40) on {port}...")
    imu = IMUSDK(port=port, baudrate=921600)
    if not imu.start():
        print("Failed to start IMU SDK")
        return

    try:
        print("Reading gyro/acc/quaternion (Ctrl+C to stop)...")
        while True:
            imu_data = imu.get_imu_data()
            if imu_data is None:
                print("Waiting for IMU+AHRS packets...")
                time.sleep(0.1)
                continue

            print(f"--- t={time.time():.2f} ---")
            print(
                "Gyro (rad/s): "
                f"X={imu_data['gyro_x']:+.3f}, Y={imu_data['gyro_y']:+.3f}, Z={imu_data['gyro_z']:+.3f}"
            )
            print(
                "Acc (m/s^2):  "
                f"X={imu_data['acc_x']:+.3f}, Y={imu_data['acc_y']:+.3f}, Z={imu_data['acc_z']:+.3f}"
            )
            print(
                "Quat (wxyz):  "
                f"w={imu_data['qw']:+.3f}, x={imu_data['qx']:+.3f}, y={imu_data['qy']:+.3f}, z={imu_data['qz']:+.3f}"
            )
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        imu.stop()


if __name__ == "__main__":
    main()
