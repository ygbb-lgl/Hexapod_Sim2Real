from utils.servo_motor_torque_sensor import*
if __name__=='__main__':
    simple_torque_control(target_torque_percent=10000,       
                          torque_limit_percent=20000, #最大限制转矩300
                          speed_limit_pulse=5000000, #最大限制速度
                          torque_slope=500000, #转矩变化率
                          target_velocity_pulse_per_sec=1000000,#设置目标速度
                          duration=5,
                          monitor_interval=0.05) #监控时间间隔