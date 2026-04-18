import serial
import struct
import time
import threading

class CableArmYawSensor:
    """后台线程持续读取臂Yaw角度，并缓存最新值。提供非阻塞接口。"""
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200, slave_id=1, timeout=0.2, poll_hz=50.0):
        self.port = port
        self.baudrate = baudrate
        self.slave_id = slave_id
        self.timeout = timeout
        self.poll_period = 1.0 / float(poll_hz) if poll_hz and poll_hz > 0 else 0.02
        
        self._lock = threading.Lock()
        self._last_angle = None
        
        self.is_running = False
        self.read_thread = None
        self.ser = None

    def start(self) -> bool:
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=self.timeout
            )
            print(f"成功打开臂Yaw角度传感器串口: {self.port}")
        except serial.SerialException as e:
            print(f"打开臂Yaw角度传感器串口 {self.port} 失败: {e}")
            return False

        self.is_running = True
        self.read_thread = threading.Thread(
            target=self._read_loop,
            name="cable_arm_yaw_reader",
            daemon=True
        )
        self.read_thread.start()
        return True

    def _calculate_crc(self, data):
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x0001:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc >>= 1
        return bytes([crc & 0xFF, (crc >> 8) & 0xFF])

    def _read_loop(self):
        # 构建读取位置信息的Modbus帧
        request_data = bytes([
            self.slave_id,   # 从机号
            0x03,            # 功能码：读取寄存器
            0x00, 0x00,      # 起始地址：0x0000
            0x00, 0x02       # 寄存器数量：2个
        ])
        request_data += self._calculate_crc(request_data)

        while self.is_running:
            try:
                if self.ser is not None and self.ser.is_open:
                    self.ser.reset_input_buffer()
                    self.ser.write(request_data)
                    
                    # 接收响应
                    time.sleep(0.01)
                    response = self.ser.read(9)  # 响应帧总长9字节
                    
                    if len(response) == 9:
                        if response[0] == self.slave_id and response[1] == 0x03 and response[2] == 0x04:
                            # 验证CRC
                            r_crc = response[-2:]
                            c_crc = self._calculate_crc(response[:-2])
                            if r_crc == c_crc:
                                position_low = (response[3] << 8) | response[4]
                                position_high = (response[5] << 8) | response[6]
                                position = (position_high << 16) | position_low
                                
                                angle = (position / 2097152.0) * 360.0
                                
                                with self._lock:
                                    self._last_angle = float(angle)
            except Exception as e:
                print(f"读取臂Yaw角度异常: {e}")
                self.is_running = False
                break
            
            time.sleep(self.poll_period)

    def get_angle(self):
        """主控制循环调用接口，非阻塞返回最近一次的有效角度"""
        with self._lock:
            return self._last_angle

    def stop(self):
        self.is_running = False
        if self.read_thread and self.read_thread.is_alive():
            self.read_thread.join()
        if self.ser is not None and self.ser.is_open:
            try:
                self.ser.close()
                print("臂Yaw角度传感器串口已关闭")
            except Exception:
                pass


if __name__ == "__main__":
    TEST_PORT = '/dev/ttyUSB0'  # 依实际情况修改
    sensor = CableArmYawSensor(port=TEST_PORT, baudrate=115200)
    
    if sensor.start():
        print("臂Yaw角度传感器读取线程已启动，按 Ctrl+C 停止。")
        try:
            while True:
                ang = sensor.get_angle()
                if ang is not None:
                    print(f"Arm Yaw Angle = {ang:.4f} 度")
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("\n正在停止程序...")
            sensor.stop()
            print("程序已退出。")