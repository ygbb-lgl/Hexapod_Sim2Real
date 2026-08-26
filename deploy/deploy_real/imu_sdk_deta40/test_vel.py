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

    print("IMU started. Keep the robot stationary for 1 second...")
    imu.begin_stationary_calibration()
    time.sleep(1.0)
    bias_calibrated = imu.reset_linear_velocity()
    print(
        "Selected source: corrected world-frame IMU integration; "
        f"startup bias calibrated={bias_calibrated}"
    )
    try:
        while True:
            vel = imu.get_linear_velocity()
            grav = imu.get_gravity_acceleration()
            diag = imu.get_velocity_diagnostics()
            bias = diag["accelerometer_bias_body"]
            acc = diag["compensated_acceleration"]
            print(
                f"Vel: [{vel[0]:+7.3f}, {vel[1]:+7.3f}, {vel[2]:+7.3f}] "
                f"| Grav: [{grav[0]:+7.3f}, {grav[1]:+7.3f}, {grav[2]:+7.3f}] "
                f"| Acc: [{acc[0]:+7.3f}, {acc[1]:+7.3f}, {acc[2]:+7.3f}] "
                f"| dt={diag['velocity_dt_s']:.6f}s "
                f"align={diag['imu_ahrs_timestamp_delta_s']:+.6f}s "
                f"pending={diag['pending_imu_sample_count']} "
                f"valid={diag['velocity_estimator_valid']} "
                f"| bias=[{bias[0]:+7.4f}, {bias[1]:+7.4f}, {bias[2]:+7.4f}] "
                f"samples={diag['bias_calibration_sample_count']}"
            )
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        imu.stop()


if __name__ == "__main__":
    main()
