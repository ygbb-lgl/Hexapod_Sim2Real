import time
import sys
import os

# Add current directory to path
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

try:
    from imu_sdk import IMUSDK
except ImportError:
    print("Failed to import IMUSDK.")
    sys.exit(1)

def main():
    port = '/dev/ttyUSB0'
    if len(sys.argv) > 1:
        port = sys.argv[1]
        
    print(f"Initializing IMU SDK on {port}...")
    imu = IMUSDK(port=port, baudrate=921600)
    
    if not imu.start():
        print(f"Failed to start IMU.")
        return

    print("IMU started. Reading data...")
    print("Format: Vel: [vx, vy, vz] | Grav: [gx, gy, gz]")
    
    try:
        while True:
            vel = imu.get_linear_velocity()
            grav = imu.get_gravity_acceleration()
            
            print(f"Vel: [{vel[0]:6.3f}, {vel[1]:6.3f}, {vel[2]:6.3f}] | Grav: [{grav[0]:6.3f}, {grav[1]:6.3f}, {grav[2]:6.3f}]")
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("\nStopping...")
        imu.stop()

if __name__ == "__main__":
    main()
