from .angel_sensor_expenience import *
import can
import time
import struct

class TorqueController:
    def __init__(self, channel='PCAN_USBBUS1', bustype='pcan', node_id=1):
        """初始化转矩控制器"""
        self.node_id = node_id
        self.bus = can.interface.Bus(channel=channel, bustype=bustype, bitrate=1000000)
        self.cob_id = 0x600 + node_id  # 标准SDO发送COB-ID
        self.nmt_id = 0x000  # NMT消息的COB-ID
        
    def send_nmt_command(self, command, node_id=None):
        """发送NMT命令启动节点"""
        if node_id is None:
            node_id = self.node_id
            
        nmt_data = bytes([command, node_id]) + bytes(6)
        message = can.Message(arbitration_id=self.nmt_id, data=nmt_data, is_extended_id=False)
        self.bus.send(message)
        print(f"NMT命令: 启动节点{node_id}进入操作模式")
        time.sleep(0.1)
    
    def send_raw_command(self, command_bytes, description=""):
        """发送原始命令字节"""
        if len(command_bytes) != 8:
            command_bytes = command_bytes + bytes(8 - len(command_bytes))
            
        message = can.Message(arbitration_id=self.cob_id, data=command_bytes, is_extended_id=False)
        self.bus.send(message)
        time.sleep(0.01)
    
    def initialize_communication(self):
        """初始化通信"""
        print("开始初始化通信...")
        
        # 配置同步周期
        cmd1 = bytes([0x23, 0x05, 0x10, 0x00, 0x80, 0x00, 0x00, 0x80])
        self.send_raw_command(cmd1, "配置同步周期")
        time.sleep(0.02)
        
        # 配置心跳
        cmd2 = bytes([0x2B, 0x17, 0x10, 0x00, 0x20, 0x03, 0x00, 0x00])
        self.send_raw_command(cmd2, "配置心跳")
        time.sleep(0.02)
        
        # 启动NMT进入OP模式
        self.send_nmt_command(0x01, self.node_id)
        time.sleep(0.1)
        
        print("通信初始化完成")
    
    def set_torque_mode(self):
        """设置转矩模式: 2F 60 60 00 04 00 00 00 (6060h=4)"""
        cmd = bytes([0x2F, 0x60, 0x60, 0x00, 0x04, 0x00, 0x00, 0x00])
        self.send_raw_command(cmd, "设置转矩模式")
    
    def servo_enable_sequence(self):
        """伺服使能序列"""
        print("开始伺服使能序列...")
        
        # 关机: 2B 40 60 00 06 00 00 00
        cmd1 = bytes([0x2B, 0x40, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00])
        self.send_raw_command(cmd1, "关机")
        time.sleep(0.02)
        
        # 上电: 2B 40 60 00 07 00 00 00
        cmd2 = bytes([0x2B, 0x40, 0x60, 0x00, 0x07, 0x00, 0x00, 0x00])
        self.send_raw_command(cmd2, "上电")
        time.sleep(0.02)
        
        # 使能运行: 2B 40 60 00 0F 00 00 00
        cmd3 = bytes([0x2B, 0x40, 0x60, 0x00, 0x0F, 0x00, 0x00, 0x00])
        self.send_raw_command(cmd3, "使能运行")
        time.sleep(0.1)
        
        print("伺服使能完成")

    def set_electronic_gear_ratio(self, numerator=131072*121, denominator=10000):
    # 10000脉冲转n圈：numerator=131072*n 15859712
        try:
            
            self.initialize_communication()
            time.sleep(0.1)
            # # # 1. 设置分子 (6091h-01)
            num_bytes = struct.pack('<I', numerator)
            cmd_numerator = bytes([0x23, 0x91, 0x60, 0x01]) + num_bytes
            self.send_raw_command(cmd_numerator, "")
            time.sleep(0.05)
            
            # 2. 设置分母 (6091h-02)
            den_bytes = struct.pack('<I', denominator)
            cmd_denominator = bytes([0x23, 0x91, 0x60, 0x02]) + den_bytes
            self.send_raw_command(cmd_denominator, "")
            time.sleep(0.05)

            return True
        except Exception as e:
            print(f"设置电子齿轮比失败: {e}")
            return False
        
    def set_target_velocity(self,velocity_pulse_per_sec):
        '''设置目标速度，单位：脉冲/秒'''
        # 转换为有符号32位整数
        velocity_raw = int(velocity_pulse_per_sec)
        # 打包为4字节小端有符号整数
        velocity_bytes = struct.pack('<i', velocity_raw)
        cmd = bytes([0x23, 0xFF, 0x60, 0x00]) + velocity_bytes
        self.send_raw_command(cmd, f"设置目标速度 {velocity_pulse_per_sec} 脉冲/秒")

    
    def set_target_torque(self, torque_percent):
        """
        设置目标转矩 (6071h)
        参数: torque_percent - 转矩百分比，范围 -100% 到 +100%
             正值为正转，负值为反转
        单位: 0.1% (手册中转矩为INT16类型，单位0.1%)
        """
        # 转换为驱动器单位 (0.1%)
        torque_raw = int(torque_percent * 10)  # 百分比转换为0.1%
        
        # 限制范围
        torque_raw = max(-1000, min(1000, torque_raw))  # ±1000对应±100%
        
        # 打包为2字节小端有符号整数
        torque_bytes = struct.pack('<h', torque_raw)
        
        # 发送命令: 2B 71 60 00 + 转矩值(2字节小端) + 00 00
        cmd = bytes([0x2B, 0x71, 0x60, 0x00]) + torque_bytes + bytes(2)
        self.send_raw_command(cmd, f"设置目标转矩 {torque_percent}%")
    
    def set_torque_limit(self, limit_percent):
        """
        设置转矩限幅 (6072h)
        参数: limit_percent - 最大转矩限幅百分比
        """
        limit_raw = int(limit_percent * 10)  # 转换为0.1%单位
        limit_raw = max(0, min(1000, limit_raw))  # 0-100%
        
        limit_bytes = struct.pack('<h', limit_raw)
        cmd = bytes([0x2B, 0x72, 0x60, 0x00]) + limit_bytes + bytes(2)
        self.send_raw_command(cmd, f"设置转矩限幅 {limit_percent}%")
    
    def set_torque_slope(self, slope):
        """
        设置转矩斜率 (6087h)
        参数: slope - 转矩变化率，单位: 0.1%/s
        """
        slope_bytes = struct.pack('<I', slope)
        cmd = bytes([0x23, 0x87, 0x60, 0x00]) + slope_bytes
        self.send_raw_command(cmd, f"设置转矩斜率 {slope} (0.1%/s)")
    
    def set_speed_limit(self, speed_limit):
        """
        设置最大速度限幅 (607Fh)
        参数: speed_limit - 最大转速限幅，单位: 脉冲/秒
        """
        speed_bytes = struct.pack('<I', speed_limit)
        cmd = bytes([0x23, 0x7F, 0x60, 0x00]) + speed_bytes
        print(f"speed_bytes (十六进制): {speed_bytes.hex()}")
        # cmd = bytes([0x23, 0x7F, 0x60, 0x00, 0x10, 0x27, 0x00, 0x00])#
        # cmd = bytes([0x23, 0x7F, 0x60, 0x00, 0x60, 0x79, 0xFE, 0xFF])#反转测试-100000
        # cmd = bytes([0x23, 0x7F, 0x60, 0x00, 0xF0, 0xD9, 0xFF, 0xFF])#-10000
        self.send_raw_command(cmd, f"设置速度限幅 {speed_limit} 脉冲/秒")
    
    def read_actual_torque(self):
        """读取实际转矩 (6077h)"""
        cmd = bytes([0x40, 0x77, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00])
        
        message = can.Message(
            arbitration_id=self.cob_id,
            data=cmd,
            is_extended_id=False
        )
        self.bus.send(message)
        
        response_id = 0x580 + self.node_id
        start_time = time.time()
        
        while time.time() - start_time < 1.0:
            msg = self.bus.recv(timeout=0.5)
            if msg and msg.arbitration_id == response_id:
                if len(msg.data) >= 8 and (msg.data[0] & 0xF0) == 0x40:
                    if msg.data[1] == 0x77 and msg.data[2] == 0x60:
                        try:
                            torque_raw = struct.unpack('<h', msg.data[4:6])[0]
                            torque_percent = torque_raw * 0.1  # 转换为百分比
                            return torque_raw, torque_percent
                        except struct.error:
                            return None, None
        
        return None, None
    
    def read_actual_velocity(self):
        """读取实际速度 (606Ch)"""
        cmd = bytes([0x40, 0x6C, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00])
        
        message = can.Message(
            arbitration_id=self.cob_id,
            data=cmd,
            is_extended_id=False
        )
        self.bus.send(message)
        
        response_id = 0x580 + self.node_id
        start_time = time.time()
        
        while time.time() - start_time < 1.0:
            msg = self.bus.recv(timeout=0.5)
            if msg and msg.arbitration_id == response_id:
                if len(msg.data) >= 8 and (msg.data[0] & 0xF0) == 0x40:
                    if msg.data[1] == 0x6C and msg.data[2] == 0x60:
                        try:
                            velocity = struct.unpack('<i', msg.data[4:8])[0]
                            return velocity
                        except struct.error:
                            return None
        
        return None
    
    def read_actual_position(self):
        """读取当前位置 (6064h)"""
        cmd = bytes([0x40, 0x64, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00])
        
        message = can.Message(
            arbitration_id=self.cob_id,
            data=cmd,
            is_extended_id=False
        )
        self.bus.send(message)
        
        response_id = 0x580 + self.node_id
        start_time = time.time()
        
        while time.time() - start_time < 1.0:
            msg = self.bus.recv(timeout=0.5)
            if msg and msg.arbitration_id == response_id:
                if len(msg.data) >= 8 and (msg.data[0] & 0xF0) == 0x40:
                    if msg.data[1] == 0x64 and msg.data[2] == 0x60:
                        try:
                            position = struct.unpack('<i', msg.data[4:8])[0]
                            return position
                        except struct.error:
                            return None
        
        return None
    
    def get_motor_info(self):
        """获取电机实时信息"""
        position = self.read_actual_position()
        velocity = self.read_actual_velocity()
        torque_raw, torque_percent = self.read_actual_torque()

        info = {}
        
        # 位置信息
        if position is not None:
            PULSES_PER_REV = 10000  # 假设编码器分辨率
            remainder_pulses = position % PULSES_PER_REV
            remainder_angle = remainder_pulses * 360.0 / PULSES_PER_REV
            
            info.update({
                'raw_position': position,
                'rotations': position / PULSES_PER_REV,
                'position_deg': remainder_angle,
                'angle_deg': remainder_angle
            })
        
        # 速度信息
        if velocity is not None:
            # 转换为RPM (假设10000脉冲/圈)
            rpm = velocity * 60.0 / 10000.0
            info.update({
                'raw_velocity': velocity,
                'velocity_rpm': rpm,
                # 'velocity_pulse_per_sec': velocity
            })
        
        # 转矩信息
        if torque_percent is not None:
            info.update({
                'torque_raw': torque_raw,
                'torque_percent': torque_percent
            })
        
        return info if info else None
    
    def get_remainder_angle(self):
        """获取电机角度"""
        info = self.get_motor_info()
        if info and 'angle_deg' in info:
            return info['angle_deg']
        return None
    
    def torque_control(self, target_torque_percent, 
                      torque_limit_percent,
                      speed_limit_pulse,
                      torque_slope,
                      target_velocity_pulse_per_sec,
                      duration,
                      monitor_interval=0.1):
        """
        执行转矩控制
        
        参数:
            target_torque_percent: 目标转矩百分比 (负值为反转)
            torque_limit_percent: 转矩限幅百分比
            speed_limit_pulse: 最大速度限幅 (脉冲/秒)
            torque_slope: 转矩斜率 (0.1%/s)
            duration: 运行时间 (秒)
            monitor_interval: 监控间隔 (秒)
        """
        try:
            print("=== 开始转矩控制 ===")
            
            # 1. 初始化通信
            self.initialize_communication()
            time.sleep(0.2)
            
            # 2. 设置转矩模式
            self.set_torque_mode()
            time.sleep(0.1)
            
            # 3. 伺服使能
            self.servo_enable_sequence()
            time.sleep(0.2)
            
            # 4. 设置转矩参数
            self.set_torque_limit(torque_limit_percent)
            time.sleep(0.05)
            
            self.set_torque_slope(torque_slope)
            time.sleep(0.05)
            
            self.set_speed_limit(speed_limit_pulse)
            time.sleep(0.05)
            
            # 5. 设置目标转矩
            self.set_target_torque(target_torque_percent)
            time.sleep(0.05)

            self.set_target_velocity(target_velocity_pulse_per_sec)
            time.sleep(0.05)
            
            print(f"转矩控制已启动: 目标转矩={target_torque_percent}%")
            print("时间(s)\t电机角度(°)\t传感器角度(°)\t角度差值(°)\t速度(RPM)\t转矩(%)")
            print("-" * 60)
            
            # 6. 监控运行
            start_time = time.time()
            while time.time() - start_time < duration:
                try:
                    info = self.get_motor_info()
                    if info:
                        elapsed = time.time() - start_time
                        
                        angle = info.get('angle_deg', 'N/A')
                        speed = info.get('velocity_rpm', 'N/A')
                        torque = info.get('torque_percent', 'N/A')
                        
                        # 格式化输出
                        angle_str = f"{angle:.2f}" if isinstance(angle, (int, float)) else str(angle)
                        speed_str = f"{speed:.1f}" if isinstance(speed, (int, float)) else str(speed)
                        torque_str = f"{torque:.1f}" if isinstance(torque, (int, float)) else str(torque)
                        
                        angel=get_encoder_angle(port='COM7', baudrate=115200, slave_id=1, timeout=1.0)
                        
                        angle_diff = (angel + angle)%360
                        if angle_diff<0:
                            angle_diff=angle_diff+360
                        
                        print(f"{elapsed:.2f}\t{angle_str}°\t{angel:.6f}°\t{angle_diff:.6f}°\t{speed_str}\t{torque_str}%\tN/A")
                        
                    else:
                        elapsed = time.time() - start_time
                        print(f"{elapsed:.2f}\t读取失败")
                        
                except Exception as e:
                    print(f"监控错误: {e}")
                
                time.sleep(monitor_interval)
            
            # 7. 停止转矩输出
            self.set_target_torque(0)
            print("转矩控制结束，转矩已归零")
            
            return True
            
        except Exception as e:
            print(f"转矩控制错误: {e}")
            return False
        finally:
            self.close()
    
    def close(self):
        """关闭CAN总线"""
        if self.bus:
            self.bus.shutdown()


def simple_torque_control(target_torque_percent, 
                         torque_limit_percent,
                         speed_limit_pulse,
                         torque_slope,
                         target_velocity_pulse_per_sec,
                         duration,
                         monitor_interval,
                         node_id=1):
    """
    简化的转矩控制函数
    
    参数:
        target_torque_percent: 目标转矩百分比 (-100 到 100)
        torque_limit_percent: 最大转矩限幅 (默认100%)
        speed_limit_pulse: 最大速度 (脉冲/秒，默认3000)
        torque_slope: 转矩变化率 (0.1%/s，默认100)
        duration: 运行时间 (秒，默认10)
        monitor_interval: 监控间隔 (秒，默认0.1)
        node_id: CAN节点ID (默认1)
    """
    controller = TorqueController(node_id=node_id)
    controller.set_electronic_gear_ratio()  # 不传参数，默认121/1
    try:
        success = controller.torque_control(
            target_torque_percent=target_torque_percent,
            torque_limit_percent=torque_limit_percent,
            speed_limit_pulse=speed_limit_pulse,
            torque_slope=torque_slope,
            target_velocity_pulse_per_sec=target_velocity_pulse_per_sec,
            duration=duration,
            monitor_interval=monitor_interval
        )
        return success
        
    except Exception as e:
        print(f"错误: {e}")
        return False
    finally:
        controller.close()

