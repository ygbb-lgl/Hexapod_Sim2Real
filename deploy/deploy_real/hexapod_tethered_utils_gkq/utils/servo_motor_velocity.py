import can
import time
import struct

class VelocityController:
    def __init__(self, channel='PCAN_USBBUS1', bustype='pcan', node_id=1):
        """初始化速度控制器"""
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
    
    def set_velocity_mode(self):
        """设置速度模式: 2F 60 60 00 03 00 00 00 (6060h=3)"""
        cmd = bytes([0x2F, 0x60, 0x60, 0x00, 0x03, 0x00, 0x00, 0x00])
        self.send_raw_command(cmd, "设置速度模式")
    
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
    
    def set_target_velocity(self, velocity_pulse_per_sec):
        """
        设置目标速度 (60FFh)
        参数: velocity_pulse_per_sec - 目标速度，单位: 脉冲/秒
              正值为正转，负值为反转
        手册范围: 0 ~ 3000 脉冲/秒
        """
        # 转换为有符号32位整数
        velocity_raw = int(velocity_pulse_per_sec)
        
        # 打包为4字节小端有符号整数
        velocity_bytes = struct.pack('<i', velocity_raw)
        
        # 发送命令: 2B FF 60 00 + 速度值(4字节小端) + 00 00
        cmd = bytes([0x23, 0xFF, 0x60, 0x00]) + velocity_bytes
        self.send_raw_command(cmd, f"设置目标速度 {velocity_pulse_per_sec} 脉冲/秒")
    
    def set_acceleration(self, acceleration_pulse_per_sec2):
        """
        设置加速度 (6083h)
        参数: acceleration_pulse_per_sec2 - 加速度，单位: 脉冲/秒²
        手册范围: 0 ~ 2000 脉冲/秒²
        """
        acceleration_bytes = struct.pack('<I', acceleration_pulse_per_sec2)
        cmd = bytes([0x23, 0x83, 0x60, 0x00]) + acceleration_bytes
        self.send_raw_command(cmd, f"设置加速度 {acceleration_pulse_per_sec2} 脉冲/秒²")
    
    def set_deceleration(self, deceleration_pulse_per_sec2):
        """
        设置减速度 (6084h)
        参数: deceleration_pulse_per_sec2 - 减速度，单位: 脉冲/秒²
        手册范围: 0 ~ 2000 脉冲/秒²
        """
        deceleration_bytes = struct.pack('<I', deceleration_pulse_per_sec2)
        cmd = bytes([0x23, 0x84, 0x60, 0x00]) + deceleration_bytes
        self.send_raw_command(cmd, f"设置减速度 {deceleration_pulse_per_sec2} 脉冲/秒²")
    
    def set_max_velocity_limit(self, max_velocity_pulse_per_sec):
        """
        设置最大速度限幅 (607Fh)
        参数: max_velocity_pulse_per_sec - 最大转速限幅，单位: 脉冲/秒
        """
        max_velocity_bytes = struct.pack('<I', max_velocity_pulse_per_sec)
        cmd = bytes([0x23, 0x7F, 0x60, 0x00]) + max_velocity_bytes
        self.send_raw_command(cmd, f"设置最大速度限幅 {max_velocity_pulse_per_sec} 脉冲/秒")
    
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
    
    def get_motor_info(self):
        """获取电机实时信息"""
        position = self.read_actual_position()
        velocity = self.read_actual_velocity()
        torque_raw, torque_percent = self.read_actual_torque()
        
        info = {}
        
        # 位置信息 - 根据手册假设编码器分辨率
        if position is not None:
            PULSES_PER_REV = 10000  # 假设编码器分辨率，请根据实际修改！
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
            rpm = velocity * 60.0 / 10000.0 if velocity >= 0 else 0
            info.update({
                'raw_velocity': velocity,
                'velocity_rpm': rpm,
                'velocity_pulse_per_sec': velocity
            })
        
        # 转矩信息
        if torque_percent is not None:
            info.update({
                'torque_raw': torque_raw,
                'torque_percent': torque_percent
            })
        
        return info if info else None
    
    def velocity_control(self, target_velocity_pulse_per_sec, 
                        acceleration_pulse_per_sec2,
                        deceleration_pulse_per_sec2,
                        max_velocity_limit_pulse_per_sec,
                        duration,
                        monitor_interval=0.1):
        """
        执行速度控制
        
        参数:
            target_velocity_pulse_per_sec: 目标速度 (脉冲/秒)
            acceleration_pulse_per_sec2: 加速度 (脉冲/秒²)
            deceleration_pulse_per_sec2: 减速度 (脉冲/秒²)
            max_velocity_limit_pulse_per_sec: 最大速度限幅 (脉冲/秒)
            duration: 运行时间 (秒)
            monitor_interval: 监控间隔 (秒)
        """
        try:
            print("=== 开始速度控制 ===")
            
            # 1. 初始化通信
            self.initialize_communication()
            time.sleep(0.2)
            
            # 2. 设置速度模式
            self.set_velocity_mode()
            time.sleep(0.1)
            
            # 3. 伺服使能
            self.servo_enable_sequence()
            time.sleep(0.2)
            
            # 4. 设置速度参数
            self.set_acceleration(acceleration_pulse_per_sec2)
            time.sleep(0.05)
            
            self.set_deceleration(deceleration_pulse_per_sec2)
            time.sleep(0.05)
            
            self.set_max_velocity_limit(max_velocity_limit_pulse_per_sec)
            time.sleep(0.05)
            
            # 5. 设置目标速度
            self.set_target_velocity(target_velocity_pulse_per_sec)
            time.sleep(0.05)
            
            print(f"速度控制已启动: 目标速度={target_velocity_pulse_per_sec} 脉冲/秒")
            print("时间(s)\t角度(°)\t速度(RPM)\t速度(脉冲/秒)\t转矩(%)")
            print("-" * 80)
            
            # 6. 监控运行
            start_time = time.time()
            while time.time() - start_time < duration:
                try:
                    info = self.get_motor_info()
                    if info:
                        elapsed = time.time() - start_time
                        
                        angle = info.get('angle_deg', 'N/A')
                        speed_rpm = info.get('velocity_rpm', 'N/A')
                        speed_pulse = info.get('velocity_pulse_per_sec', 'N/A')
                        torque = info.get('torque_percent', 'N/A')
                        
                        # 格式化输出
                        angle_str = f"{angle:.2f}" if isinstance(angle, (int, float)) else str(angle)
                        speed_rpm_str = f"{speed_rpm:.1f}" if isinstance(speed_rpm, (int, float)) else str(speed_rpm)
                        speed_pulse_str = f"{speed_pulse:.0f}" if isinstance(speed_pulse, (int, float)) else str(speed_pulse)
                        torque_str = f"{torque:.1f}" if isinstance(torque, (int, float)) else str(torque)
                        
                        print(f"{elapsed:.2f}\t{angle_str}°\t{speed_rpm_str}\t{speed_pulse_str}\t{torque_str}%")
                    else:
                        elapsed = time.time() - start_time
                        print(f"{elapsed:.2f}\t读取失败")
                        
                except Exception as e:
                    print(f"监控错误: {e}")
                
                time.sleep(monitor_interval)
            
            # 7. 停止速度输出
            self.set_target_velocity(0)
            print("速度控制结束，速度已归零")
            
            return True
            
        except Exception as e:
            print(f"速度控制错误: {e}")
            return False
        finally:
            self.close()
    
    def close(self):
        """关闭CAN总线"""
        if self.bus:
            self.bus.shutdown()


def simple_velocity_control(target_velocity_pulse_per_sec,
                           acceleration_pulse_per_sec2,
                           deceleration_pulse_per_sec2,
                           max_velocity_limit_pulse_per_sec,
                           duration,
                           monitor_interval,
                           node_id=1):
    """
    简化的速度控制函数
    
    参数:
        target_velocity_pulse_per_sec: 目标速度 (脉冲/秒，默认1000)
        acceleration_pulse_per_sec2: 加速度 (脉冲/秒²，默认1000)
        deceleration_pulse_per_sec2: 减速度 (脉冲/秒²，默认1000)
        max_velocity_limit_pulse_per_sec: 最大速度限幅 (脉冲/秒，默认3000)
        duration: 运行时间 (秒，默认10)
        monitor_interval: 监控间隔 (秒，默认0.1)
        node_id: CAN节点ID (默认1)
    """
    controller = VelocityController(node_id=node_id)
    
    try:
        success = controller.velocity_control(
            target_velocity_pulse_per_sec=target_velocity_pulse_per_sec,
            acceleration_pulse_per_sec2=acceleration_pulse_per_sec2,
            deceleration_pulse_per_sec2=deceleration_pulse_per_sec2,
            max_velocity_limit_pulse_per_sec=max_velocity_limit_pulse_per_sec,
            duration=duration,
            monitor_interval=monitor_interval
        )
        return success
        
    except Exception as e:
        print(f"错误: {e}")
        return False
    finally:
        controller.close()