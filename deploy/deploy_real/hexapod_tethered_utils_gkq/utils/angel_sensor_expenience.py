import serial
import struct
import time

def get_encoder_angle(port='COM3', baudrate=115200, slave_id=1, timeout=1.0):
    """
    获取绝对值编码器的角度值
    
    Args:
        port: 串口号，默认COM3
        baudrate: 波特率，默认115200
        slave_id: 从机地址，默认1
        timeout: 超时时间，默认1秒
    
    Returns:
        float: 角度值（度数），如果读取失败返回None
    """
    try:
        # 连接串口
        ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=timeout
        )
        
        # 构建读取位置信息的Modbus帧 (读取2个寄存器：0x0000和0x0001)
        frame = bytes([
            slave_id,        # 从机号
            0x03,            # 功能码：读取寄存器
            0x00, 0x00,      # 起始地址：0x0000
            0x00, 0x02       # 寄存器数量：2个
        ])
        
        # 计算CRC16校验
        crc = 0xFFFF
        for byte in frame:
            crc ^= byte
            for _ in range(8):
                if crc & 0x0001:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc >>= 1
        
        frame += bytes([crc & 0xFF, (crc >> 8) & 0xFF])
        
        # 发送请求
        ser.write(frame)
        
        # 接收响应
        response = ser.read(9)  # 响应帧总长9字节
        
        if len(response) < 9:
            ser.close()
            return None
        
        # 解析响应数据
        # 响应格式: [从机号, 0x03, 字节数, 数据1高, 数据1低, 数据2高, 数据2低, CRC低, CRC高]
        if response[0] == slave_id and response[1] == 0x03 and response[2] == 0x04:
            # 提取位置数据 (低16位 + 高16位)
            position_low = (response[3] << 8) | response[4]    # 低16位寄存器值
            position_high = (response[5] << 8) | response[6]   # 高16位寄存器值
            position = (position_high << 16) | position_low   # 组合成32位位置值
            
            # 计算角度 (位置值 / 总计数2097152 * 360°)
            angle = (position / 2097152) * 360.0
            
            ser.close()
            return angle
        else:
            ser.close()
            return None
            
    except Exception as e:
        print(f"读取编码器角度错误: {e}")
        return None