import time
import sys
import os

# 将当前目录添加到路径中，确保能导入 imu_sdk
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from imu_sdk import IMUSDK

def main():
    # 默认端口 /dev/ttyUSB0, 如果你的IMU在其他端口请修改
    # 默认波特率 921600 (与 C++ sdk 默认一致)
    port = "/dev/ttyUSB0"
    baud = 921600
    
    imu = IMUSDK(port, baud)
    
    print(f"Connecting to IMU on {port}...")
    if not imu.start():
        print("Failed to start IMU SDK")
        return

    try:
        print("Start reading IMU data (Press Ctrl+C to stop)...")
        while True:
            # 获取数据
            imu_data = imu.get_imu_data()
            ahrs_data = imu.get_ahrs_data()
            
            # 清屏 (可选，让输出更整洁)
            # print("\033[H\033[J", end="")
            print(f"--- Time: {time.time():.2f} ---")
            
            if imu_data:
                print(f"[IMU Raw Data]")
                print(f"  Gyro (rad/s): X={imu_data['gyro_x']:.3f}, Y={imu_data['gyro_y']:.3f}, Z={imu_data['gyro_z']:.3f}")
                print(f"  Accel (m/s²): X={imu_data['acc_x']:.3f}, Y={imu_data['acc_y']:.3f}, Z={imu_data['acc_z']:.3f}")
                # print(f"  Mag:          X={imu_data['mag_x']:.3f}, Y={imu_data['mag_y']:.3f}, Z={imu_data['mag_z']:.3f}")
            else:
                print("[IMU Raw Data] Waiting for data...")
                
            if ahrs_data:
                print(f"[AHRS Fusion Data]")
                print(f"  Euler (rad):  Roll={ahrs_data['roll']:.3f}, Pitch={ahrs_data['pitch']:.3f}, Heading={ahrs_data['heading']:.3f}")
                print(f"  Quaternion:   w={ahrs_data['qw']:.3f}, x={ahrs_data['qx']:.3f}, y={ahrs_data['qy']:.3f}, z={ahrs_data['qz']:.3f}")
            else:
                print("[AHRS Fusion Data] Waiting for data...")
            
            #print("-" * 40)
            time.sleep(0.1) # 10Hz 打印频率
            
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        imu.stop()
        print("Done.")

if __name__ == "__main__":
    main()
