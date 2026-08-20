import os
import sys
import time

# Add current directory to path so `from imu_sdk import IMUSDK` works when run directly.
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

    print(f"Initializing IMU SDK (deta40) on {port}...")
    imu = IMUSDK(port=port, baudrate=921600)
    if not imu.start():
        print("Failed to start IMU.")
        return

    print("IMU started. Selected source: IMU integration only")
    try:
        while True:
            vel = imu.get_linear_velocity()
            grav = imu.get_gravity_acceleration()
            print(
                f"Vel: [{vel[0]:+7.3f}, {vel[1]:+7.3f}, {vel[2]:+7.3f}] "
                f"| Grav: [{grav[0]:+7.3f}, {grav[1]:+7.3f}, {grav[2]:+7.3f}]"
            )
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        imu.stop()


if __name__ == "__main__":
    main()
