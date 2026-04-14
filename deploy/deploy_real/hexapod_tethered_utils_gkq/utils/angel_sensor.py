import serial
import struct
import time

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

# 封装的函数：在main函数中调用此函数即可获取角度
def get_angle_value(serial_port):
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

def continuous_read_angle(serial_port, interval=0.1):
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