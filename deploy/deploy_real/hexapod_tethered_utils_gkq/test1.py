import serial
import threading
import time
import struct
import numpy as np

# 由 Fz 计算“拉力值”的线性标定参数：pull_force = (-Fz - offset) / scale
PULL_FORCE_OFFSET = 2.1008
PULL_FORCE_SCALE = 0.6057

class SensorDataParser:
    """
    解析来自传感器的 M8218 数据帧 (0xAA 0x55 开头)
    支持手动零点校准
    """
    def __init__(self):
        self.buffer = bytearray()
        # 零点偏移值
        self.zero_offsets = np.zeros(6)  # [Fx, Fy, Fz, Mx, My, Mz]
        self.is_zero_calibrated = False
        self.calibration_samples = []
        self.calibrating = False
        
    def start_zero_calibration(self, num_samples=50):
        """开始零点校准（应在无负载时调用）"""
        print(f"开始零点校准，采集{num_samples}个样本...")
        self.calibrating = True
        self.calibration_samples = []
        # 这里简化为计数，实际可能需要定时采集
        
    def process_calibration(self):
        """处理采集的校准数据"""
        if len(self.calibration_samples) == 0:
            return
            
        # 计算平均值作为零点偏移
        samples_array = np.array(self.calibration_samples)
        self.zero_offsets = np.mean(samples_array, axis=0)
        self.is_zero_calibrated = True
        print(f"零点校准完成，偏移值：{self.zero_offsets}")
        self.calibrating = False
        self.calibration_samples = []

    def parse(self, data):
        self.buffer.extend(data)
        
        while True:
            # 查找帧头 0xAA 0x55
            head_index = self.buffer.find(b'\xAA\x55')
            if head_index == -1:
                if len(self.buffer) > 1:
                    self.buffer = self.buffer[-1:]
                break

            if len(self.buffer) < head_index + 31:
                if head_index > 0:
                    self.buffer = self.buffer[head_index:]
                break

            frame = self.buffer[head_index : head_index + 31]
            frame_len = int.from_bytes(frame[2:4], 'big')
            if frame_len != 27:
                self.buffer = self.buffer[head_index + 2:]
                continue

            checksum_from_sensor = frame[30]
            calculated_checksum = sum(frame[6:30]) & 0xFF
            
            if checksum_from_sensor == calculated_checksum:
                try:
                    # 解析原始数据
                    raw_values = np.array(struct.unpack('<ffffff', frame[6:30]))
                    
                    # 如果正在校准，收集样本
                    if self.calibrating:
                        self.calibration_samples.append(raw_values)
                        if len(self.calibration_samples) >= 50:  # 采集50个样本
                            self.process_calibration()
                    
                    # 应用零点补偿
                    if self.is_zero_calibrated:
                        compensated_values = raw_values - self.zero_offsets
                    else:
                        compensated_values = raw_values
                    
                    fx, fy, fz, mx, my, mz = compensated_values
                    
                    # 可选：添加滤波或死区处理
                    # 死区处理：非常小的值视为零
                    dead_zone = 0.001
                    if abs(fx) < dead_zone: fx = 0.0
                    if abs(fy) < dead_zone: fy = 0.0
                    if abs(fz) < dead_zone: fz = 0.0
                    if abs(mx) < dead_zone: mx = 0.0
                    if abs(my) < dead_zone: my = 0.0
                    if abs(mz) < dead_zone: mz = 0.0
                    
                    # print(f"M8218 = {fx:10.6f}, {fy:10.6f}, {fz:10.6f},   {mx:10.6f}, {my:10.6f}, {mz:10.6f}")
                    # print(f"{fz:10.6f}")
                    cable_tension = (-fz - PULL_FORCE_OFFSET) / PULL_FORCE_SCALE
                    # print(cable_tension)  # 输出拉力计算值
                except struct.error as e:
                    print(f"数据解析错误: {e}")

                self.buffer = self.buffer[head_index + 31:]
            else:
                self.buffer = self.buffer[head_index + 1:]


class SerialManager:
    """
    管理串口连接、发送指令和接收数据
    """
    def __init__(self, port, baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self.is_running = False
        self.read_thread = None
        self.parser = SensorDataParser()

    def start(self):
        try:
            self.ser = serial.Serial(
                self.port,
                self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_TWO,
                timeout=1
            )
            print(f"成功打开串口 {self.port}")
        except serial.SerialException as e:
            print(f"打开串口 {self.port} 失败: {e}")
            return False

        self.is_running = True
        self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
        self.read_thread.start()
        
        # 配置传感器
        self._configure_sensor()
        
        return True

    def _send_command(self, command, params=""):
        at_command = f"AT+{command}={params}\r\n"
        print(f"发送指令: {at_command.strip()}")
        try:
            self.ser.write(at_command.encode('ascii'))
            time.sleep(0.2)
        except serial.SerialException as e:
            print(f"发送指令失败: {e}")

    def _configure_sensor(self):
        # 设置采样率
        self._send_command("SMPF", "100")
        # 设置数据校验模式
        self._send_command("DCKMD", "SUM")
        # 开始持续上传数据
        self._send_command("GSD", "")
        print("传感器配置完成，开始接收数据...")
        print("等待3秒让传感器稳定...")
        time.sleep(3)
        
        # 可选：自动零点校准
        print("开始自动零点校准...")
        self.parser.start_zero_calibration()

    def _read_loop(self):
        while self.is_running:
            try:
                if self.ser and self.ser.in_waiting > 0:
                    data = self.ser.read(self.ser.in_waiting)
                    self.parser.parse(data)
            except serial.SerialException as e:
                print(f"读取串口时发生错误: {e}")
                self.is_running = False
                break
            time.sleep(0.01)

    def stop(self):
        self.is_running = False
        if self.read_thread:
            self.read_thread.join()
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("串口已关闭")


if __name__ == "__main__":
    SERIAL_PORT = "COM3"
    
    manager = SerialManager(port=SERIAL_PORT, baudrate=115200)
    
    if manager.start():
        print("程序正在运行，按 Ctrl+C 停止。")
        print("确保传感器在启动时无负载，以进行正确零点校准。")
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("\n正在停止程序...")
            manager.stop()
            print("程序已退出。")