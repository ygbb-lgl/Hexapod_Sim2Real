import can
import time
import struct

class ServoController:
    def __init__(self, channel='PCAN_USBBUS1', bustype='pcan', node_id=1):
        self.node_id = node_id
        self.bus = can.interface.Bus(channel=channel, bustype=bustype, bitrate=1000000)
        # 注意：图片中所有命令的CANID都是601，对应节点ID=1
        self.cob_id = 0x600 + node_id  # 标准SDO发送COB-ID
        self.nmt_id = 0x000  # NMT消息的COB-ID

    def send_nmt_command(self, command, node_id=None):
        """发送NMT命令启动节点"""
        if node_id is None:
            node_id = self.node_id
            
        # NMT消息格式：命令字节 + 节点ID
        nmt_data = bytes([command, node_id]) + bytes(6)  # 填充到8字节
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
        """初始化通信（严格按照第一张图片）"""
        print("开始初始化通信...")
        
        # 1. 配置同步周期: 23 05 10 00 80 00 00 80
        cmd1 = bytes([0x23, 0x05, 0x10, 0x00, 0x80, 0x00, 0x00, 0x80])
        self.send_raw_command(cmd1, "配置同步周期")
        time.sleep(0.02)
        
        # 2. 配置心跳: 2B 17 10 00 20 03 00 00
        cmd2 = bytes([0x2B, 0x17, 0x10, 0x00, 0x20, 0x03, 0x00, 0x00])
        self.send_raw_command(cmd2, "配置心跳")
        time.sleep(0.02)
        
        # 3. 启动NMT进入OP模式: CANID=00, 报文=01 00
        self.send_nmt_command(0x01, self.node_id)  # 01=启动命令
        time.sleep(0.1)
        
        print("通信初始化完成")
# ###################
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


    def set_position_mode(self):
        """设置位置模式: 2F 60 60 00 01 00 00 00"""
        cmd = bytes([0x2F, 0x60, 0x60, 0x00, 0x01, 0x00, 0x00, 0x00])
        self.send_raw_command(cmd, "设置位置模式")

    def servo_enable_sequence(self):
        """伺服使能序列（严格按照第一张图片）"""
        print("开始伺服使能序列...")
        
        # 关机: 2B 40 60 00 06 00 00 00
        cmd1 = bytes([0x2B, 0x40, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00])
        self.send_raw_command(cmd1, "关机")
        time.sleep(0.02)  # 图片建议间隔10ms以上
        
        # 上电: 2B 40 60 00 07 00 00 00
        cmd2 = bytes([0x2B, 0x40, 0x60, 0x00, 0x07, 0x00, 0x00, 0x00])
        self.send_raw_command(cmd2, "上电")
        time.sleep(0.02)
        
        # 使能运行: 2B 40 60 00 0F 00 00 00
        cmd3 = bytes([0x2B, 0x40, 0x60, 0x00, 0x0F, 0x00, 0x00, 0x00])
        self.send_raw_command(cmd3, "使能运行")
        time.sleep(0.1)
        
        print("伺服使能完成")

    def set_target_position(self, position):
        """设置目标位置: 23 7A 60 00 + 位置值(4字节小端)"""
        pos_bytes = struct.pack('<I', position)
        cmd = bytes([0x23, 0x7A, 0x60, 0x00]) + pos_bytes
        self.send_raw_command(cmd, f"设置目标位置 {position}")

    def set_target_velocity(self, velocity):
        """设置目标速度: 23 81 60 00 + 速度值(4字节小端)"""
        vel_bytes = struct.pack('<I', velocity)
        cmd = bytes([0x23, 0x81, 0x60, 0x00]) + vel_bytes
        self.send_raw_command(cmd, f"设置目标速度 {velocity}")

    def set_acceleration(self, acceleration):
        """设置加速度: 23 83 60 00 + 加速度值(4字节小端)"""
        acc_bytes = struct.pack('<I', acceleration)
        cmd = bytes([0x23, 0x83, 0x60, 0x00]) + acc_bytes
        self.send_raw_command(cmd, f"设置加速度 {acceleration}")

    def set_deceleration(self, deceleration):
        """设置减速度: 23 84 60 00 + 减速度值(4字节小端)"""
        dec_bytes = struct.pack('<I', deceleration)
        cmd = bytes([0x23, 0x84, 0x60, 0x00]) + dec_bytes
        self.send_raw_command(cmd, f"设置减速度 {deceleration}")

    def start_absolute_move(self):
        """绝对定位启动: 2B 40 60 00 1F 00 00 00"""
        cmd = bytes([0x2B, 0x40, 0x60, 0x00, 0x1F, 0x00, 0x00, 0x00])
        self.send_raw_command(cmd, "绝对定位启动")

    def start_relative_move(self):
        """相对定位启动: 2B 40 60 00 5F 00 00 00"""
        cmd = bytes([0x2B, 0x40, 0x60, 0x00, 0x5F, 0x00, 0x00, 0x00])
        self.send_raw_command(cmd, "相对定位启动")

    def read_status_word(self):
        """读取状态字: 40 41 60 00 00 00 00 00"""
        cmd = bytes([0x40, 0x41, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00])
        self.send_raw_command(cmd, "读取状态字")

        
        # 
        # 等待响应
        start_time = time.time()
        while time.time() - start_time < 1.0:
            msg = self.bus.recv(timeout=0.5)
            if msg and msg.arbitration_id == (0x580 + self.node_id):
                status = msg.data[4] | (msg.data[5] << 8)
                # print(f"状态字响应: 0x{status:04X}")
                return status
        # print("读取状态字超时")
        return None

    def complete_position_move(self, position, velocity, acceleration, deceleration, absolute=True, monitor_during_move=False, monitor_interval=0.1, monitor_duration=0):
        """执行位置移动，可选择在运动过程中监控
            
            Args:
                position: 目标位置
                velocity: 目标速度
                acceleration: 加速度
                deceleration: 减速度
                absolute: 是否绝对定位
                monitor_during_move: 是否在运动过程中监控
                monitor_interval: 监控间隔（秒）
                monitor_duration: 监控持续时间（0表示只监控到运动完成）
            """
        try:
            print("=== 开始位置控制流程 ===")
            
            # 1. 初始化通信
            self.initialize_communication()
            time.sleep(0.2)
            
            # 2. 设置位置模式
            self.set_position_mode()
            time.sleep(0.1)
            
            # 3. 伺服使能
            self.servo_enable_sequence()
            time.sleep(0.2)
            
            # 4. 设置运动参数
            self.set_target_position(position)
            time.sleep(0.05)
            self.set_target_velocity(velocity)
            time.sleep(0.05)
            self.set_acceleration(acceleration)
            time.sleep(0.05)
            self.set_deceleration(deceleration)
            time.sleep(0.05)
            
            # 5. 启动运动
            if absolute:
                self.start_absolute_move()
            else:
                self.start_relative_move()
            
            print("运动指令已发送，开始监控..." if monitor_during_move else "运动指令已发送，等待完成...")
            
            # 6. 等待运动完成，可选择监控
            start_time = time.time()
            if monitor_during_move:
                print("时间(s)\t位置(脉冲)\t圈数\t角度(度)\t转矩(%)")
                print("-" * 70)
            
            while True:
                # 如果启用运动过程中监控
                if monitor_during_move:
                    try:
                        info = self.get_position_info()
                        elapsed = time.time() - start_time
                        if info:
                            torque_str = f"{info.get('torque_percent', 'N/A'):.2f}" if 'torque_percent' in info else "N/A"
                            print(f"{elapsed:.3f}\t{info.get('raw_position', 'N/A')}\t{info.get('rotations', 'N/A'):.3f}\t{info.get('position_deg', 'N/A'):.2f}\t{torque_str}")
                            # print(f"{info.get('position_deg', 'N/A'):.2f}\t")
                    #     else:
                            
                    #         print(f"{elapsed:.3f}\t读取失败")
                    # except Exception as e:
                    #     print(f"监控错误: {e}")
                    except Exception:
                        pass
                
                # 检查状态字
                status = self.read_status_word()
                if status is not None:
                    # 检查目标到达位（bit10）和定位完成位（bit15）
                    if status & (1 << 10) or status & (1 << 15):
                        if monitor_during_move:
                            print("运动完成！")
                        else:
                            print("运动完成！")
                        return True
                
                # 如果设置了监控持续时间，并且时间已到，则停止监控但不停止运动
                if monitor_duration > 0 and (time.time() - start_time) > monitor_duration:
                    if monitor_during_move:
                        print(f"监控时间 {monitor_duration} 秒已到，停止监控，运动继续...")
                        monitor_during_move = False
                
                time.sleep(monitor_interval if monitor_during_move else 0.2)
            
        except Exception as e:
            print(f"运动控制错误: {e}")
            return False



    def read_actual_position_direct(self):
        '''直接读取当前位置(索引6064h)'''
        # 使用SDO读取当前位置：40 64 60 00 00 00 00 00
        cmd = bytes([0x40, 0x64, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00])
        self.send_raw_command(cmd, "读取当前位置")
    # 
        # 发送SDO读取请求
        message = can.Message(
            arbitration_id=self.cob_id, 
            data=cmd, 
            is_extended_id=False
        )
        self.bus.send(message)
        # print(f"发送SDO读取: {' '.join(f'{b:02X}' for b in cmd)}")
            # 等待响应（响应COB-ID = 0x580 + node_id）
        response_id = 0x580 + self.node_id
    # 
        start_time = time.time()
        while time.time() - start_time < 1.0:  # 超时1秒
            msg = self.bus.recv(timeout=0.5)
            if msg and msg.arbitration_id == response_id:
                # print(f"收到响应: {' '.join(f'{b:02X}' for b in msg.data)}")
                
                # 修正：检查SDO响应格式
                # 第一个字节应该是0x4x的形式（4表示读取响应）
                if len(msg.data) >= 8 and (msg.data[0] & 0xF0) == 0x40:
                    # 检查索引是否匹配（0x6064）
                    if msg.data[1] == 0x64 and msg.data[2] == 0x60:
                        # 提取位置值（小端格式）
                        try:
                            position = struct.unpack('<i', msg.data[4:8])[0]
                            # print(f"读取到当前位置: {position} 脉冲")
                            return position
                        except struct.error as e:
                            
                            print(f"解析位置数据错误: {e}")
                            print(f"响应数据: {msg.data.hex()}")
                else:
                    # return None
                    print(f"非预期的SDO响应格式: {msg.data[0]:02X}")
                    print(f"完整响应: {msg.data.hex()}")
            # if msg and msg.arbitration_id == (0x580 + self.node_id):
            #     print(f"msg={msg}")
            #     # 检查是否是当前位置的响应（0x4B 0x64 0x60 0x00）
            #     if len(msg.data) >= 8 and msg.data[0] == 0x4B and msg.data[1] == 0x64:
            #         # 提取位置值（小端格式）
            #         position = struct.unpack('<i', msg.data[4:8])[0]
            #         return position
        print("读取当前位置超时")
        return None
    # 只监控位移：
    # def get_position_info(self):
    #     """获取当前位置信息"""
    #     position = self.read_actual_position_direct()
    #     if position is not None:
    #         return {
    #             'raw_position': position,  # 原始脉冲数
    #             'rotations': position / 10000.0,  # 转换为圈数（假设10000脉冲/圈）
    #             'position_deg': position * 360.0 / 10000.0  # 转换为角度
    #         }
    #     return None

    # 监控位移和转矩
    def get_position_info(self):
        """获取当前位置信息"""
        position = self.read_actual_position_direct()
        torque_raw, torque_percent = self.read_actual_torque()
        
        info = {}
        if position is not None:
            PULSES_PER_REV = 10000
            
            # 正确计算规范化角度
            remainder_pulses = position % PULSES_PER_REV  # 剩余脉冲
            remainder_angle = remainder_pulses * 360.0 / PULSES_PER_REV  # 规范化角度（0-360°）
            
            info.update({
                'raw_position': position,           # 原始脉冲数
                'rotations': position / PULSES_PER_REV,  # 转换为圈数
                'position_deg': remainder_angle,    # 修正：使用规范化角度
                'angle_deg': remainder_angle        # 角度别名
            })
        
        if torque_raw is not None:
            info.update({
                'torque_raw': torque_raw,
                'torque_percent': torque_percent
            })
        
        return info if info else None
    # 位置监控
    # def monitor_position_continuous(self, duration=10, interval=0.1):
    #     """连续监控位置"""
    #     print(f"开始监控位置，持续时间：{duration}秒，间隔：{interval}秒")
    #     print("时间(s)\t位置(脉冲)\t圈数\t角度(度)")
    #     print("-" * 50)

    #     start_time = time.time()
    #     while time.time() - start_time < duration:
    #         try:
    #             info = self.get_position_info()
    #             if info:
    #                 elapsed = time.time() - start_time
    #                 print(f"{elapsed:.2f}\t{info['raw_position']}\t{info['rotations']:.3f}\t{info['position_deg']:.2f}")
    #             else:
    #                 elapsed = time.time() - start_time
    #                 print(f"{elapsed:.2f}\t读取失败")
    #         except Exception as e:
    #             print(f"监控错误: {e}")
        
    #         time.sleep(interval)

    #     print("位置监控结束")
    def monitor_position_continuous(self, duration=10, interval=0.1):
        """连续监控位置、角度和转矩"""
        print(f"开始监控，持续时间：{duration}秒，间隔：{interval}秒")
        print("时间(s)\t位置(脉冲)\t整圈数\t剩余角度(°)\t转矩(%)")
        print("-" * 70)
        
        start_time = time.time()
        while time.time() - start_time < duration:
            try:
                info = self.get_position_info()
                if info:
                    elapsed = time.time() - start_time
                    
                    # 从原始位置计算规范化角度
                    PULSES_PER_REV = 10000
                    position = info.get('raw_position', 0)
                    
                    # 计算整圈数和剩余脉冲
                    full_turns = position // PULSES_PER_REV  # 整圈数
                    remainder_pulses = position % PULSES_PER_REV  # 剩余脉冲
                    
                    # 计算剩余角度
                    remainder_angle = remainder_pulses * 360.0 / PULSES_PER_REV
                    
                    # 转矩
                    torque_str = f"{info.get('torque_percent', 'N/A'):.2f}" if 'torque_percent' in info else "N/A"
                    
                    # 打印信息
                    print(f"{elapsed:.3f}\t"
                        f"{position:8}\t"
                        f"{full_turns:3}\t"
                        f"{remainder_angle:8.2f}°\t"
                        f"{torque_str}")
                else:
                    elapsed = time.time() - start_time
                    # print(f"{elapsed:.3f}\t读取失败")
            except Exception as e:
                print(f"监控错误: {e}")
            
            time.sleep(interval)
        
        # print("监控结束")
    def close(self):
        if self.bus:
            self.bus.shutdown()

    def read_actual_velocity(self):
        """读取实际转速"""
        cmd = bytes([0x40, 0x6C, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00])
        self.send_raw_command(cmd, "读取实际转速")
        
        # 等待响应
        start_time = time.time()
        while time.time() - start_time < 1.0:
            msg = self.bus.recv(timeout=0.5)
            if msg and msg.arbitration_id == (0x580 + self.node_id):
                if len(msg.data) >= 8 and msg.data[0] == 0x4B and msg.data[1] == 0x6C:
                    # 提取转速值（小端格式）
                    velocity = struct.unpack('<i', msg.data[4:8])[0]
                    # 注意：这里的单位可能是脉冲/秒，需要根据手册转换
                    return velocity
        return None
    def read_actual_torque(self):
        """读取实际转矩（6077h），单位：0.1%"""
        # SDO读取请求：40 77 60 00 00 00 00 00
        cmd = bytes([0x40, 0x77, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00])
        
        # 发送SDO读取请求
        message = can.Message(
            arbitration_id=self.cob_id,
            data=cmd,
            is_extended_id=False
        )
        self.bus.send(message)
        # print(f"发送SDO读取转矩: {' '.join(f'{b:02X}' for b in cmd)}")
        
        # 等待响应（响应COB-ID = 0x580 + node_id）
        response_id = 0x580 + self.node_id
        
        start_time = time.time()
        while time.time() - start_time < 1.0:  # 超时1秒
            msg = self.bus.recv(timeout=0.5)
            if msg and msg.arbitration_id == response_id:
                # print(f"收到转矩响应: {' '.join(f'{b:02X}' for b in msg.data)}")
                
                # 检查SDO响应格式
                if len(msg.data) >= 8 and (msg.data[0] & 0xF0) == 0x40:
                    # 检查索引是否匹配（0x6077）
                    if msg.data[1] == 0x77 and msg.data[2] == 0x60:
                        # 提取转矩值（小端格式）
                        try:
                            # 根据手册，转矩为INT16类型（2字节）
                            torque_raw = struct.unpack('<h', msg.data[4:6])[0]
                            torque_percent = torque_raw * 0.1  # 转换为百分比
                            # print(f"读取到实际转矩: {torque_raw} (0.1%)，即 {torque_percent}%")
                            return torque_raw, torque_percent
                        except struct.error as e:
                            print(f"解析转矩数据错误: {e}")
                            print(f"响应数据: {msg.data.hex()}")
                else:
                    # return None,None
                    print(f"非预期的SDO响应格式: {msg.data[0]:02X}")
                    print(f"完整响应: {msg.data.hex()}")
        
        # print("读取转矩超时")
        return None, None



# 简化使用函数
def simple_move(position, velocity, acceleration, deceleration, absolute, node_id=1,monitor_during_move=True, monitor_after_move=True,monitor_interval=0.1, duration=10):

    servo = ServoController(channel='PCAN_USBBUS1', bustype='pcan', node_id=node_id)
    servo.set_electronic_gear_ratio()  # 不传参数，默认121/1

    # servo.set_electronic_gear_ratio(50, 1)
    
    try:
        success=servo.complete_position_move(
        # success = complete_position_move(
            position=position,
            velocity=velocity,
            acceleration=acceleration,
            deceleration=deceleration,
            absolute=absolute,
            monitor_during_move=monitor_during_move,
            monitor_interval=monitor_interval,
            monitor_duration=0  # 0表示监控到运动完成
        )
        if monitor_after_move and success:
            print("\n=== 运动完成，继续监控状态 ===")
            # monitor_position_continuous(duration=duration, interval=monitor_interval)
            servo.monitor_position_continuous(duration=duration, interval=monitor_interval)
        return success
    except Exception as e:
        print(f"错误: {e}")
        return False
    finally:
        servo.close()
    