"""
系绳控制器模块 - 封装 CableSpeedController 及所有传感器/电机依赖
"""

import numpy as np
import time

from imu_sdk.imu_sdk import IMUSDK
from hexapod_tethered_utils.cable_tension_sensor import CableTensionSensor
from hexapod_tethered_utils.cable_arm_yaw_sensor import CableArmYawSensor, get_encoder_angle
from motor_igh_sdk.deploy_real_el4090_speed_pysoem import RL_Real_Speed_PySOEM, _find_policy_index_by_motor_id


class CableSpeedController:
    """系绳速度控制器：PI反馈 + 前馈控制"""
    
    def __init__(self, config, cable_motor=None, cable_motor_policy_idx=None):
        self.config = config
        self.cable_motor = cable_motor
        self.cable_motor_policy_idx = cable_motor_policy_idx
        
        # PI + 前馈参数
        self.Kp = getattr(config, 'cable_Kp', 50.0)
        self.Ki = getattr(config, 'cable_Ki', 5.0)
        self.Kff = getattr(config, 'cable_Kff', 1.0)
        
        # 积分器
        self.integral_error = 0.0
        self.max_integral = getattr(config, 'cable_max_integral', 100.0)
        
        # 卷筒半径 (m)
        self.spool_radius = 0.07
        
        # 最大电机转速 (rpm)
        self.max_rpm = getattr(config, 'cable_max_rpm', 3000.0)
        
        # 状态变量
        self.Ft_ref = 0.0
        self.Ft_measured = 0.0
        self.e_Ft = 0.0
        self.phi = 0.0
        self.vx = 0.0
        self.vd = 0.0
        self.v_fb = 0.0
        self.v_ff = 0.0
        self.v_cmd = 0.0
    
    def update_robot_velocity(self, imu):
        """从IMU获取机器人合速度"""
        try:
            vel = imu.get_linear_velocity()
            if vel is not None:
                vx, vy, vz = vel[0], vel[1], vel[2]
                self.vx = np.sqrt(vx**2 + vy**2 + vz**2)
                if vx < 0:
                    self.vx = -self.vx
            else:
                self.vx = 0.0
        except Exception as e:
            print(f"[velocity] error: {e}")
            self.vx = 0.0
    
    def update_tension(self, tension_sensor):
        """更新测量张力"""
        try:
            tension = tension_sensor.get_cable_tension()
            self.Ft_measured = float(tension) if tension is not None else 0.0
        except Exception as e:
            print(f"[tension sensor] error: {e}")
            self.Ft_measured = 0.0
    
    def update_phi(self):
        """从编码器和电机位置计算系绳偏航角"""
        try:
            sensor_angle_deg = get_encoder_angle(
                port='/dev/ttyUSB4', baudrate=115200, slave_id=1, timeout=0.1
            )
            if sensor_angle_deg is None:
                sensor_angle_deg = 0.0
            
            motor_angle_rad = 0.0
            if self.cable_motor is not None and self.cable_motor_policy_idx is not None:
                try:
                    motor_angle_rad = self.cable_motor.motor_state_buffer.position[self.cable_motor_policy_idx]
                except Exception as e:
                    print(f"[motor angle read] error: {e}")
            
            motor_angle_deg = np.degrees(motor_angle_rad)
            diff_deg = (sensor_angle_deg + motor_angle_deg) % 360.0
            self.phi = np.radians(diff_deg)
        except Exception as e:
            print(f"[phi sensor] error: {e}")
            self.phi = 0.0
    
    def compute_feedforward(self):
        """计算前馈速度 v_ff (rpm)"""
        self.vd = np.cos(self.phi) * self.vx
        
        if self.spool_radius > 0:
            omega_rad_per_sec = self.vd / self.spool_radius
            rpm = (omega_rad_per_sec / (2 * np.pi)) * 60
        else:
            rpm = 0.0
        
        self.v_ff = self.Kff * rpm
        return self.v_ff
    
    def compute_feedback(self, dt):
        """PI反馈控制"""
        self.e_Ft = self.Ft_ref - self.Ft_measured
        
        p_term = self.Kp * self.e_Ft
        self.integral_error += self.e_Ft * dt
        self.integral_error = np.clip(self.integral_error, -self.max_integral, self.max_integral)
        i_term = self.Ki * self.integral_error
        
        self.v_fb = p_term + i_term
        return self.v_fb
    
    def update(self, cable_action, tension_sensor, imu, dt):
        """
        主更新函数
        
        Args:
            cable_action: 强化学习输出的系绳动作 [-1, 1]
            tension_sensor: 张力传感器对象
            imu: IMU SDK对象
            dt: 时间步长
        
        Returns:
            target_rpm: 目标电机转速 (rpm)
        """
        max_tension = getattr(self.config, 'cable_max_tension', 200.0)
        self.Ft_ref = cable_action * max_tension
        
        self.update_phi()
        self.update_tension(tension_sensor)
        self.update_robot_velocity(imu)
        
        self.v_ff = self.compute_feedforward()
        self.v_fb = self.compute_feedback(dt)
        
        self.v_cmd = self.v_fb + self.v_ff
        self.v_cmd = np.clip(self.v_cmd, -self.max_rpm, self.max_rpm)
        
        return self.v_cmd
    
    def reset_integrator(self):
        """重置积分器"""
        self.integral_error = 0.0


class CableControlInterface:
    """
    系绳控制接口 - 封装控制器、传感器和电机的初始化与调用
    
    用法:
        cable_iface = CableControlInterface(config)
        cable_iface.start()
        
        # 在控制循环中:
        v_cmd = cable_iface.get_cable_command(cable_action, imu, dt)
    """
    
    def __init__(self, config):
        self.config = config
        
        # 组件引用
        self.tension_sensor = None
        self.cable_motor = None
        self.controller = None
        
        # 状态标记
        self.tension_started = False
        self.cable_motor_started = False
    
    def start(self) -> bool:
        """初始化所有传感器和系绳电机"""
        # 1. 张力传感器
        self.tension_sensor = CableTensionSensor(port='/dev/ttyUSB2', baudrate=115200)
        self.tension_started = self.tension_sensor.start()
        if not self.tension_started:
            print("[CableInterface] WARNING: Tension sensor start failed.")
        
        # 2. 系绳电机
        cable_motor_id = getattr(self.config, 'cable_motor_id', 4)
        self.cable_motor = RL_Real_Speed_PySOEM('enp109s0', motor_id=cable_motor_id)
        self.cable_motor_started = self.cable_motor.start()
        if not self.cable_motor_started:
            print("[CableInterface] WARNING: Cable motor start failed.")
        
        # 3. 找到电机在 policy 中的索引
        if self.cable_motor_started:
            cable_motor_policy_idx = _find_policy_index_by_motor_id(self.cable_motor, cable_motor_id)
        else:
            cable_motor_policy_idx = None
        
        # 4. 初始化控制器
        self.controller = CableSpeedController(
            self.config,
            cable_motor=self.cable_motor if self.cable_motor_started else None,
            cable_motor_policy_idx=cable_motor_policy_idx
        )
        
        return self.tension_started and self.cable_motor_started
    
    def get_cable_command(self, cable_action, imu, dt) -> float:
        """
        获取系绳电机速度命令
        
        Args:
            cable_action: 强化学习输出 [-1, 1]
            imu: IMU SDK对象
            dt: 时间步长
        
        Returns:
            target_rpm: 目标转速 (rpm)
        """
        if self.controller is None:
            return 0.0
        
        return self.controller.update(cable_action, self.tension_sensor, imu, dt)
    
    def send_speed_command(self, target_rpm: float):
        """发送速度指令到系绳电机"""
        if self.cable_motor is None or not self.cable_motor_started:
            return
        
        self.cable_motor.motor_command_buffer.target_speed_rpm[0] = target_rpm
        self.cable_motor.motor_command_buffer.current_limit_01a[0] = 500
        self.cable_motor.motor_command_buffer.ack_status[0] = 1
    
    def get_state_dict(self) -> dict:
        """返回控制器状态字典，用于日志"""
        if self.controller is None:
            return {}
        
        cc = self.controller
        return {
            'Ft_ref': cc.Ft_ref,
            'Ft_measured': cc.Ft_measured,
            'e_Ft': cc.e_Ft,
            'phi_deg': np.degrees(cc.phi),
            'vx': cc.vx,
            'vd': cc.vd,
            'v_fb': cc.v_fb,
            'v_ff': cc.v_ff,
            'v_cmd': cc.v_cmd,
        }
    
    def stop(self):
        """停止系绳电机"""
        if self.cable_motor is not None:
            self.cable_motor.stop()