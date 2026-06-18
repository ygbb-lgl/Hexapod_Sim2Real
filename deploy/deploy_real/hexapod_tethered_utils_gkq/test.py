import serial
import threading
import time
import struct

class SensorDataParser:
    """
    解析来自传感器的 M8218 数据帧 (0xAA 0x55 开头)
    """
    def __init__(self):
        self.buffer = bytearray()

    def parse(self, data):
        self.buffer.extend(data)
        
        while True:
            # 查找帧头 0xAA 0x55
            head_index = self.buffer.find(b'\xAA\x55')
            if head_index == -1:
                # 没找到帧头，为了防止buffer无限增长，只保留最后1个字节
                if len(self.buffer) > 1:
                    self.buffer = self.buffer[-1:]
                break

            # 检查数据是否足够长 (帧头2 + 长度2 + 序列号2 + 数据24 + 校验1 = 31字节)
            if len(self.buffer) < head_index + 31:
                # 数据不完整，等待更多数据
                # 如果帧头不在buffer开头，说明前面是无效数据，可以丢弃
                if head_index > 0:
                    self.buffer = self.buffer[head_index:]
                break

            # 提取可能是一个完整帧的数据
            frame = self.buffer[head_index : head_index + 31]

            # 校验数据长度字段
            frame_len = int.from_bytes(frame[2:4], 'big')
            if frame_len != 27:
                # 长度不对，说明这不是一个有效帧的开始，丢弃帧头并继续搜索
                self.buffer = self.buffer[head_index + 2:]
                continue

            # 校验和
            checksum_from_sensor = frame[30]
            calculated_checksum = sum(frame[6:30]) & 0xFF # 校验和是从第6个字节开始到数据结束
            
            if checksum_from_sensor == calculated_checksum:
                # 校验成功，解析数据
                # 数据从第6个字节开始，每个浮点数4字节
                try:
                    fx, fy, fz, mx, my, mz = struct.unpack('<ffffff', frame[6:30])
                    print(f"M8218 = {fx:10.6f}, {fy:10.6f}, {fz:10.6f},   {mx:10.6f}, {my:10.6f}, {mz:10.6f}")
                    # print(f'{fz:10.6f}')
                except struct.error as e:
                    print(f"数据解析错误: {e}")

                # 处理完一帧，从缓冲区删除
                self.buffer = self.buffer[head_index + 31:]
            else:
                # 校验失败，丢弃帧头并继续搜索
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
            # C++版本有复杂的ACK等待，这里简化为简单延时
            # 因为主要功能是接收GSD数据，ACK不是关键路径
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

    def _read_loop(self):
        while self.is_running:
            try:
                if self.ser and self.ser.in_waiting > 0:
                    data = self.ser.read(self.ser.in_waiting)
                    # 将收到的数据送给解析器处理
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
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("\n正在停止程序...")
            manager.stop()
            print("程序已退出。")
