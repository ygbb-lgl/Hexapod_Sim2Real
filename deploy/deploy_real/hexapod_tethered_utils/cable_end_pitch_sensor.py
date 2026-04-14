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
class AngleManager:
    """后台线程持续读取角度，并缓存最新值（主循环里只做 getter，不阻塞）。"""

    def __init__(
        self,
        port=None,
        baudrate=9600,
        slave_id=2,
        timeout=0.2,
        poll_hz=10.0,
    ):
        self._port = port
        self._baudrate = baudrate
        self._slave_id = slave_id
        self._timeout = timeout
        self._poll_period = 1.0 / float(poll_hz) if poll_hz and poll_hz > 0 else 0.1

        self._lock = threading.Lock()
        self._last_angle = None
        self._last_ts = 0.0
        self._failed = False

        self._stop_evt = threading.Event()
        self._thread = None
        self._sensor = None

    def start(self):
        if self._thread is not None and self._thread.is_alive():
            return
        if self._failed:
            return

        resolved_port = self._port or CABLE_PITCH_SERIAL_PORT
        self._sensor = AbsoluteAngleSensor(
            port=resolved_port,
            baudrate=self._baudrate,
            slave_id=self._slave_id,
            timeout=self._timeout,
        )

        self._stop_evt.clear()
        self._thread = threading.Thread(
            target=self._read_loop,
            name="cable_end_pitch_reader",
            daemon=True,
        )
        self._thread.start()

    def _read_loop(self):
        try:
            while not self._stop_evt.is_set():
                angle = None
                try:
                    if self._sensor is not None:
                        angle = self._sensor.read_angle_float()
                except Exception:
                    # 传感器读异常：直接标记失败并退出线程，避免刷屏
                    self._failed = True
                    return

                if angle is not None:
                    with self._lock:
                        self._last_angle = float(angle)
                        self._last_ts = time.time()

                time.sleep(self._poll_period)
        finally:
            self.close()

    def get_angle(self):
        self.start()
        with self._lock:
            return self._last_angle

    def get_angle_and_timestamp(self):
        self.start()
        with self._lock:
            return self._last_angle, self._last_ts

    def close(self):
        self._stop_evt.set()
        try:
            if self._sensor is not None and getattr(self._sensor, "ser", None) is not None:
                try:
                    self._sensor.ser.close()
                except Exception:
                    pass
        finally:
            self._sensor = None


_default_manager = None


def get_default_manager():
    global _default_manager
    if _default_manager is None:
        _default_manager = AngleManager()
        atexit.register(_default_manager.close)
    return _default_manager


def get_cable_end_pitch_angle():
    """非阻塞：返回后台线程缓存的最新角度（度）。

    - 首次调用会自动启动后台线程。
    - 串口通过本文件的 SERIAL_PORT 指定。
    """
    mgr = get_default_manager()
    return mgr.get_angle()

# 封装的函数：在main函数中调用此函数即可获取角度
def get_angle_value(serial_port=CABLE_PITCH_SERIAL_PORT):
    sensor = None
    try:
        # 初始化传感器对象
        sensor = AbsoluteAngleSensor(port=serial_port)
        # 读取角度值
        angle = sensor.read_angle_float()
        return angle
    except Exception as e:
        print(f"error: {e}")
        return None

def continuous_read_angle(serial_port=CABLE_PITCH_SERIAL_PORT, interval=0.1):
    sensor = AbsoluteAngleSensor(port=serial_port)
    try:
        while True:
            angle = sensor.read_angle_float()
            print(f"当前角度值为: {angle:.6f} 度")
            time.sleep(interval)
    except KeyboardInterrupt:
        print("停止连续读取。")
    finally:
        sensor.ser.close()

def angle_stream(serial_port, interval=0.1):
    sensor = AbsoluteAngleSensor(port=serial_port)
    try:
        while True:
            angle = sensor.read_angle_float()
            yield angle
            time.sleep(interval)
    finally:
        sensor.ser.close()