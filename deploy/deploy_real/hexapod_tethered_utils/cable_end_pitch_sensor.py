import atexit
import threading
import time
import struct

import serial


CABLE_PITCH_SERIAL_PORT = "/dev/ttyUSB1"

class AbsoluteAngleSensor:
    def __init__(self, port, baudrate=9600, slave_id=2, timeout=1):
        self.ser = serial.Serial(port, baudrate, timeout=timeout)
        self.slave_id = slave_id

    def calculate_crc(self, data):
        wcrc = 0xFFFF
        for byte in data:
            wcrc ^= byte
            for _ in range(8):
                if wcrc & 0x0001:
                    wcrc >>= 1
                    wcrc ^= 0xA001
                else:
                    wcrc >>= 1
        return bytes([wcrc & 0xFF, (wcrc >> 8) & 0xFF])

    def read_angle_float(self):
        # 1. 构建请求帧 (读取浮点数寄存器地址 0x0008)
        # 功能码03：读寄存器
        # 起始地址 0x0008：浮点数角度值的寄存器地址
        # 寄存器数量 0x0002：读取2个寄存器（共4字节，构成一个浮点数）
        request_data = [
            self.slave_id,      # 站号
            0x03,               # 功能码
            0x00, 0x08,         # 寄存器起始地址 (0x0008)
            0x00, 0x02          # 要读取的寄存器数量
        ]
        request_bytes = bytes(request_data)
        crc = self.calculate_crc(request_bytes)
        full_request = request_bytes + crc

        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        self.ser.write(full_request)
        
        time.sleep(0.05)  # 等待传感器响应
        response = self.ser.read(9)  # 预期响应长度为9字节

        if len(response) != 9:
            print(f"错误：响应数据长度不匹配。期望9字节，收到{len(response)}字节: {response.hex()}")
            return None

        # 5. 验证响应头与CRC
        if response[0] != self.slave_id or response[1] != 0x03:
            print("错误：响应帧站号或功能码不匹配。")
            return None

        # 验证响应数据的CRC
        received_crc = response[-2:]
        calculated_crc = self.calculate_crc(response[:-2])
        if received_crc != calculated_crc:
            print("错误：响应数据CRC校验失败。")
            return None

        # 响应数据格式: [站号][03][字节数][4字节浮点数][2字节CRC]
        float_bytes = response[3:7] 
        try:
            angle_value = struct.unpack('>f', float_bytes)[0]
            return angle_value
        except Exception as e:
            print(f"解析浮点数时发生错误: {e}")
            return None
class CableEndPitchSensor:
    """后台线程持续读取角度，并缓存最新值。提供简单的 start/get_angle/stop 接口。"""
    def __init__(self, port, baudrate=9600, slave_id=2, timeout=0.2, poll_hz=10.0):
        self.port = port
        self.baudrate = baudrate
        self.slave_id = slave_id
        self.timeout = timeout
        self.poll_period = 1.0 / float(poll_hz) if poll_hz and poll_hz > 0 else 0.1

        self._lock = threading.Lock()
        self._last_angle = None
        self._last_ts = 0.0

        self.is_running = False
        self.read_thread = None
        self.sensor = None

    def start(self) -> bool:
        try:
            self.sensor = AbsoluteAngleSensor(
                port=self.port,
                baudrate=self.baudrate,
                slave_id=self.slave_id,
                timeout=self.timeout,
            )
            print(f"成功打开末端角度传感器串口 {self.port}")
        except serial.SerialException as e:
            print(f"打开末端角度传感器串口 {self.port} 失败: {e}")
            return False

        self.is_running = True
        self.read_thread = threading.Thread(
            target=self._read_loop,
            name="cable_end_pitch_reader",
            daemon=True,
        )
        self.read_thread.start()
        return True

    def _read_loop(self):
        while self.is_running:
            angle = None
            try:
                if self.sensor is not None:
                    angle = self.sensor.read_angle_float()
            except Exception as e:
                print(f"末端角度传感器读取发生异常: {e}")
                self.is_running = False
                break

            if angle is not None:
                with self._lock:
                    self._last_angle = float(angle)
                    self._last_ts = time.time()

            time.sleep(self.poll_period)

    def get_angle(self):
        """主控制循环调用接口，非阻塞返回最近一次的有效角度"""
        with self._lock:
            return self._last_angle

    def get_angle_and_timestamp(self):
        with self._lock:
            return self._last_angle, self._last_ts

    def stop(self):
        self.is_running = False
        if self.read_thread and self.read_thread.is_alive():
            self.read_thread.join()
        if self.sensor is not None and getattr(self.sensor, "ser", None) is not None:
            try:
                self.sensor.ser.close()
                print("末端角度传感器串口已关闭")
            except Exception:
                pass


if __name__ == "__main__":
    CABLE_PITCH_SERIAL_PORT = "/dev/ttyUSB0"
    
    sensor = CableEndPitchSensor(port=CABLE_PITCH_SERIAL_PORT, baudrate=9600)
    
    if sensor.start():
        print("角度传感器读数线程已启动，按 Ctrl+C 停止。")
        try:
            while True:
                latest = sensor.get_angle()
                if latest is not None:
                    print(f"cable_end_pitch_angle = {latest:.6f} 度")
                time.sleep(0.1) # 模拟主控制循环抓取数据
        except KeyboardInterrupt:
            print("\n正在停止程序...")
            sensor.stop()
            print("程序已退出。")