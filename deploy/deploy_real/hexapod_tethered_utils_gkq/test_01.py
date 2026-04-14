# from utils.servo_motor import *
# from utils.angel_sensor import *
# from utils.angel_sensor_expenience import *


# from utils.angel_sensor import *
# if __name__ == "__main__":
#     while True:
#         angle = get_angle_value(serial_port='COM6')      
#         print(f"当前角度值为: {angle:.6f} 度")

# from utils.angel_sensor_expenience import*
# if __name__=="__main__":
#     while True:
#         angel=get_encoder_angle(port='COM7', baudrate=115200, slave_id=1, timeout=1.0)
#         print(f'当前角度值为：{angel:.6f}°')





#位置模式
from utils.servo_motor_position import *
if __name__ == "__main__":
    simple_move(position=10000, velocity=100000, acceleration=50000, deceleration=50000,
            absolute=False, node_id=1, 
            monitor_during_move=True,  # 开启运动中监控
            monitor_after_move=False,  # 关闭运动后监控
            monitor_interval=0.05)     # 更快的监控间隔


# 速度模式
# from utils.servo_motor_velocity import*
# if __name__ == "__main__":
#     success = simple_velocity_control(
#         target_velocity_pulse_per_sec=200000,  # 目标速度 2000脉冲/秒
#         acceleration_pulse_per_sec2=100000,    # 加速度 1000脉冲/秒²
#         deceleration_pulse_per_sec2=100000,    # 减速度 1000脉冲/秒²
#         max_velocity_limit_pulse_per_sec=300000,  # 最大速度限幅 3000脉冲/秒
#         duration=10,                          # 运行5秒
#         monitor_interval=0.1,                # 监控间隔0.1秒
#         node_id=1   )                         # 节点ID=1
    


# # # 转矩模式
# from utils.servo_motor_torque import*
# if __name__=='__main__':
#     simple_torque_control(target_torque_percent=10000,       
#                           torque_limit_percent=20000, #最大限制转矩300
#                           speed_limit_pulse=1000000, #最大限制速度
#                           torque_slope=500000, #转矩变化率
#                           target_velocity_pulse_per_sec=1000000,#设置目标速度
#                           duration=5,
#                           monitor_interval=0.05) #监控时间间隔
    

    #     # 减速比1：121
#     # duration: 监控时间：10秒
#  更改脉冲：get_position_info内
#     if position is not None:
        # 假设编码器分辨率10000脉冲/圈 - 请根据实际修改！
        # PULSES_PER_REV = 10000
#输出：时间，位移脉冲，圈数，角度，转矩 第244行


# 速度模式
# "时间(s)\t角度(°)\t速度(RPM)\t速度(脉冲/秒)\t状态"









# from utils.servo_motor_torque import TorqueController
# from utils.angel_sensor_expenience import get_encoder_angle
# import time

# def calculate_angle_difference():
#     """计算角度传感器和电机角度的差值"""
#     # 初始化电机控制器
#     controller = TorqueController(node_id=1)

#     controller.initialize_communication()
#     controller.set_torque_mode()
#     controller.servo_enable_sequence()
#     time.sleep(0.5)
    
#     try:
#         while True:
#                 sensor_angle = get_encoder_angle(port='COM7', baudrate=115200, slave_id=1, timeout=1.0)
#                 motor_angle = controller.get_remainder_angle()
                
#                 if sensor_angle is not None and motor_angle is not None:
#                     # 计算差值（传感器角度 - 电机角度）
#                     angle_difference = sensor_angle - motor_angle
#                     print(f'角度差值: {angle_difference:.6f}°')
                
                
#                 time.sleep(0.1) 
            
#     except KeyboardInterrupt:
#         print("\n程序被用户中断")
#     finally:
#         controller.close()
#         print("已关闭电机控制器")

# if __name__ == "__main__":
#     calculate_angle_difference()
#     # simple_move(position=100000, velocity=20000, acceleration=8000, deceleration=8000, absolute=False, node_id=1,monitor=True,duration=10)
