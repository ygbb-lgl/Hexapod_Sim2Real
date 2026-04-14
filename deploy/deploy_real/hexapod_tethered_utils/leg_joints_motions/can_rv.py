"""can_rv.py

这是对 `can_rv.c/.h` 的 Python 版翻译（协议打包/解包逻辑保持一致）。

你不再需要单片机来“编译/运行”这份协议逻辑，但要真正控制电机仍需要：
- 一条 CAN 总线硬件接口（例如 USB-CAN、CAN 网卡、树莓派 CAN HAT 等）
- Linux 下通常用 SocketCAN；Python 可选用 `python-can` 来发送/接收帧。

本文件提供两层：
1) 纯协议层：负责把控制量打包成 8 字节（或 7 字节）payload，以及把回传帧解包。
2) 传输层接口：`CanInterface`，你可以用自己的方式把帧发到 CAN 上。

为了便于对照学习，函数名尽量与 C 版一致（驼峰/下划线也保持原样）。
"""

from __future__ import annotations

from dataclasses import dataclass, field
import struct
from typing import List, Optional, Protocol, Sequence

try:
    # Package import (preferred)
    from .math_ops import float_to_uint, uint_to_float
except ImportError:  # pragma: no cover
    # Fallback when running from this directory directly.
    from math_ops import float_to_uint, uint_to_float  # type: ignore

# -----------------------------
# 常量（来自 can_rv.c）
# -----------------------------

KP_MIN = 0.0
KP_MAX = 500.0
KD_MIN = 0.0
KD_MAX = 5.0
POS_MIN = -12.5
POS_MAX = 12.5
SPD_MIN = -18.0
SPD_MAX = 18.0
T_MIN = -30.0
T_MAX = 30.0
I_MIN = -30.0
I_MAX = 30.0

# 参数读取指令（来自 can_rv.h）
param_get_pos = 0x01
param_get_spd = 0x02
param_get_cur = 0x03
param_get_pwr = 0x04
param_get_acc = 0x05
param_get_lkgKP = 0x06
param_get_spdKI = 0x07
param_get_fdbKP = 0x08
param_get_fdbKD = 0x09

# 通信模式（来自 can_rv.h）
comm_ack = 0x00
comm_auto = 0x01


# -----------------------------
# CAN 帧的最小抽象
# -----------------------------

@dataclass(frozen=True)
class CanFrame:
    """一个最简 CAN 帧结构。

    - `arbitration_id` 对应 C 里的 StdId
    - `data` 为 bytes（长度 0~8）
    - 本 demo 只用标准帧（11-bit ID），所以默认 `is_extended_id=False`
    """

    arbitration_id: int
    data: bytes
    is_extended_id: bool = False

    @property
    def dlc(self) -> int:
        return len(self.data)


class CanInterface(Protocol):
    """CAN 发送接口：你可以用 SocketCAN / python-can / 自己的串口网关来实现。"""

    def send(self, frame: CanFrame) -> None:  # pragma: no cover
        raise NotImplementedError


@dataclass
class PrintCanInterface:
    """默认的“演示接口”：只打印要发送的帧，不真正发到 CAN 总线上。"""

    def send(self, frame: CanFrame) -> None:
        data_hex = frame.data.hex(" ")
        print(f"CAN TX id=0x{frame.arbitration_id:X} dlc={frame.dlc} data=[{data_hex}]")


# -----------------------------
# 数据结构（对应 C 的 struct）
# -----------------------------

@dataclass
class MotorCommFbd:
    motor_id: int = 0
    INS_code: int = 0
    motor_fbd: int = 0


@dataclass
class ODMotorMsg:
    # 原 C 结构体字段很多；这里保持同名，便于对照。
    angle_actual_int: int = 0
    angle_desired_int: int = 0
    speed_actual_int: int = 0
    speed_desired_int: int = 0
    current_actual_int: int = 0
    current_desired_int: int = 0

    speed_actual_rad: float = 0.0
    speed_desired_rad: float = 0.0
    angle_actual_rad: float = 0.0
    angle_desired_rad: float = 0.0

    motor_id: int = 0
    temperature: float = 0.0
    error: int = 0

    angle_actual_float: float = 0.0
    speed_actual_float: float = 0.0
    current_actual_float: float = 0.0

    angle_desired_float: float = 0.0
    speed_desired_float: float = 0.0
    current_desired_float: float = 0.0

    power: float = 0.0
    acceleration: int = 0
    linkage_KP: int = 0
    speed_KI: int = 0
    feedback_KP: int = 0
    feedback_KD: int = 0


# 对应 C 的全局变量
motor_comm_fbd = MotorCommFbd()
rv_motor_msg: List[ODMotorMsg] = [ODMotorMsg() for _ in range(8)]
motor_id_check: int = 0


# -----------------------------
# 一些小工具：字节/有符号数处理
# -----------------------------

def _u16_be(msb: int, lsb: int) -> int:
    return ((msb & 0xFF) << 8) | (lsb & 0xFF)


def _s16_be(msb: int, lsb: int) -> int:
    return int.from_bytes(bytes([msb & 0xFF, lsb & 0xFF]), byteorder="big", signed=True)


def _pack_float_le(value: float) -> bytes:
    """按 C/ARM 常见的小端序把 float 打包为 4 字节。

    重要：原 C 代码通过 union 的 buf[0..3] 取字节，默认运行在 little-endian。
    """

    return struct.pack("<f", float(value))


# -----------------------------
# 发送侧（对应 can_rv.c 的打包函数）
# -----------------------------

class RVCanMotor:
    """把 C 里的“全局函数 + CAN_Transmit”封装成一个 Python 对象。

    使用方法：
        motor = RVCanMotor(can=PrintCanInterface())
        motor.send_motor_ctrl_cmd(motor_id=1, kp=10, kd=0.5, pos=0, spd=0, tor=0)

    说明：
    - 这里把 CAN 发送动作抽象为 `self.can.send(frame)`。
    - 如果你接了真实 CAN 设备，把 `can` 换成你的实现即可。
    """

    def __init__(self, can: Optional[CanInterface] = None):
        self.can: CanInterface = can if can is not None else PrintCanInterface()

    # cmd:
    # 0x00: NON
    # 0x01: set comm mode auto feedback
    # 0x02: set comm mode response
    # 0x03: set current position to zero
    def MotorSetting(self, motor_id: int, cmd: int) -> None:
        if cmd == 0:
            return
        data = bytes([
            (motor_id >> 8) & 0xFF,
            motor_id & 0xFF,
            0x00,
            cmd & 0xFF,
        ])
        self.can.send(CanFrame(arbitration_id=0x7FF, data=data))

    def MotorIDReset(self) -> None:
        data = bytes([0x7F, 0x7F, 0x00, 0x05, 0x7F, 0x7F])
        self.can.send(CanFrame(arbitration_id=0x7FF, data=data))

    def MotorIDSetting(self, motor_id: int, motor_id_new: int) -> None:
        data = bytes([
            (motor_id >> 8) & 0xFF,
            motor_id & 0xFF,
            0x00,
            0x04,
            (motor_id_new >> 8) & 0xFF,
            motor_id_new & 0xFF,
        ])
        self.can.send(CanFrame(arbitration_id=0x7FF, data=data))

    def MotorCommModeReading(self, motor_id: int) -> None:
        data = bytes([
            (motor_id >> 8) & 0xFF,
            motor_id & 0xFF,
            0x00,
            0x81,
        ])
        self.can.send(CanFrame(arbitration_id=0x7FF, data=data))

    def MotorIDReading(self) -> None:
        data = bytes([0xFF, 0xFF, 0x00, 0x82])
        self.can.send(CanFrame(arbitration_id=0x7FF, data=data))

    def set_motors_current(self, motor_msgs: Sequence[ODMotorMsg], motor_quantity: int) -> None:
        """自动反馈模式下的多电机电流控制打包。

        C 版会检查发送邮箱状态并 busy-wait；Python 版这里不做忙等，只直接发两帧。
        """

        if motor_quantity < 1:
            return

        # 帧 1：前 4 个电机 -> StdId 0x1FF
        payload = bytearray(8)
        for i in range(4):
            cur = int(motor_msgs[i].current_desired_int) & 0xFFFF
            payload[2 * i] = (cur >> 8) & 0xFF
            payload[2 * i + 1] = cur & 0xFF
        self.can.send(CanFrame(arbitration_id=0x1FF, data=bytes(payload)))

        if motor_quantity < 5:
            return

        # 帧 2：后 4 个电机 -> StdId 0x2FF
        payload = bytearray(8)
        for i in range(4):
            cur = int(motor_msgs[i + 4].current_desired_int) & 0xFFFF
            payload[2 * i] = (cur >> 8) & 0xFF
            payload[2 * i + 1] = cur & 0xFF
        self.can.send(CanFrame(arbitration_id=0x2FF, data=bytes(payload)))

    def send_motor_ctrl_cmd(self, motor_id: int, kp: float, kd: float, pos: float, spd: float, tor: float) -> None:
        """“问答/响应”模式下的力控指令（KP/KD/位置/速度/力矩）。

        这就是 C 版 `send_motor_ctrl_cmd` 的 8 字节打包。
        """

        # 1) 先按原代码做范围裁剪
        if kp > KP_MAX:
            kp = KP_MAX
        elif kp < KP_MIN:
            kp = KP_MIN

        if kd > KD_MAX:
            kd = KD_MAX
        elif kd < KD_MIN:
            kd = KD_MIN

        if pos > POS_MAX:
            pos = POS_MAX
        elif pos < POS_MIN:
            pos = POS_MIN

        if spd > SPD_MAX:
            spd = SPD_MAX
        elif spd < SPD_MIN:
            spd = SPD_MIN

        if tor > T_MAX:
            tor = T_MAX
        elif tor < T_MIN:
            tor = T_MIN

        # 2) 浮点 -> 定点整数
        kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12)
        kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 9)
        pos_int = float_to_uint(pos, POS_MIN, POS_MAX, 16)
        spd_int = float_to_uint(spd, SPD_MIN, SPD_MAX, 12)
        tor_int = float_to_uint(tor, T_MIN, T_MAX, 12)

        # 3) 按位拼成 8 字节
        data0 = (0x00 | (kp_int >> 7)) & 0xFF
        data1 = (((kp_int & 0x7F) << 1) | ((kd_int & 0x100) >> 8)) & 0xFF
        data2 = kd_int & 0xFF
        data3 = (pos_int >> 8) & 0xFF
        data4 = pos_int & 0xFF
        data5 = (spd_int >> 4) & 0xFF
        data6 = (((spd_int & 0x0F) << 4) | ((tor_int >> 8) & 0x0F)) & 0xFF
        data7 = tor_int & 0xFF

        self.can.send(CanFrame(arbitration_id=motor_id & 0x7FF, data=bytes([data0, data1, data2, data3, data4, data5, data6, data7])))

    def set_motor_position(self, motor_id: int, pos: float, spd: int, cur: int, ack_status: int) -> None:
        """设置目标位置（协议: 0x20...）。"""

        if ack_status > 3:
            return

        b = _pack_float_le(pos)  # b[0] LSB, b[3] MSB
        data = bytearray(8)
        data[0] = (0x20 | (b[3] >> 3)) & 0xFF
        data[1] = ((b[3] << 5) | (b[2] >> 3)) & 0xFF
        data[2] = ((b[2] << 5) | (b[1] >> 3)) & 0xFF
        data[3] = ((b[1] << 5) | (b[0] >> 3)) & 0xFF
        data[4] = ((b[0] << 5) | ((int(spd) >> 10) & 0x1F)) & 0xFF
        data[5] = ((int(spd) & 0x3FC) >> 2) & 0xFF
        data[6] = (((int(spd) & 0x03) << 6) | ((int(cur) >> 6) & 0x3F)) & 0xFF
        data[7] = (((int(cur) & 0x3F) << 2) | (ack_status & 0x03)) & 0xFF

        self.can.send(CanFrame(arbitration_id=motor_id & 0x7FF, data=bytes(data)))

    def set_motor_speed(self, motor_id: int, spd: float, cur: int, ack_status: int) -> None:
        """设置目标速度（协议: 0x40...，DLC=7）。"""

        b = _pack_float_le(spd)
        data = bytearray(7)
        data[0] = (0x40 | (ack_status & 0x03)) & 0xFF
        data[1] = b[3]
        data[2] = b[2]
        data[3] = b[1]
        data[4] = b[0]
        data[5] = (int(cur) >> 8) & 0xFF
        data[6] = int(cur) & 0xFF
        self.can.send(CanFrame(arbitration_id=motor_id & 0x7FF, data=bytes(data)))

    def set_motor_cur_tor(self, motor_id: int, cur_tor: int, ctrl_status: int, ack_status: int) -> None:
        """设置电流/力矩/刹车控制（协议: 0x60...，DLC=3）。"""

        if ack_status > 3:
            return
        if ctrl_status > 7:
            return

        # 原 C 代码：根据 ctrl_status 对范围不同限幅
        if ctrl_status != 0:
            if cur_tor > 3000:
                cur_tor = 3000
            elif cur_tor < -3000:
                cur_tor = -3000
        else:
            if cur_tor > 2000:
                cur_tor = 2000
            elif cur_tor < -2000:
                cur_tor = -2000

        data0 = (0x60 | ((ctrl_status & 0x07) << 2) | (ack_status & 0x03)) & 0xFF
        cur_tor_u16 = int(cur_tor) & 0xFFFF
        data1 = (cur_tor_u16 >> 8) & 0xFF
        data2 = cur_tor_u16 & 0xFF

        self.can.send(CanFrame(arbitration_id=motor_id & 0x7FF, data=bytes([data0, data1, data2])))

    def set_motor_acceleration(self, motor_id: int, acc: int, ack_status: int) -> None:
        """设置加速度（协议: 0xC0 0x01 ...，DLC=4）。"""

        if ack_status > 2:
            return
        if acc > 2000:
            acc = 2000

        data = bytes([
            (0xC0 | (ack_status & 0x03)) & 0xFF,
            0x01,
            (int(acc) >> 8) & 0xFF,
            int(acc) & 0xFF,
        ])
        self.can.send(CanFrame(arbitration_id=motor_id & 0x7FF, data=data))

    def set_motor_linkage_speedKI(self, motor_id: int, linkage: int, speedKI: int, ack_status: int) -> None:
        """设置 linkage 和 speedKI（协议: 0xC0 0x02 ...，DLC=6）。"""

        if ack_status > 2:
            return
        linkage = min(int(linkage), 10000)
        speedKI = min(int(speedKI), 10000)

        data = bytes([
            (0xC0 | (ack_status & 0x03)) & 0xFF,
            0x02,
            (linkage >> 8) & 0xFF,
            linkage & 0xFF,
            (speedKI >> 8) & 0xFF,
            speedKI & 0xFF,
        ])
        self.can.send(CanFrame(arbitration_id=motor_id & 0x7FF, data=data))

    # 原 C 源码里提供的是 set_motor_feedbackKP_KD；头文件只声明了 KP。
    # 为了方便使用，这里提供：
    # - set_motor_feedbackKP_KD：一次设置 KP + KD（与 can_rv.c 一致）
    # - set_motor_feedbackKP / set_motor_feedbackKD：单独设置（用同一条协议 0x03，另一项传 0）

    def set_motor_feedbackKP_KD(self, motor_id: int, fdbKP: int, fdbKD: int, ack_status: int) -> None:
        """设置反馈环 KP/KD（协议: 0xC0 0x03 ...，DLC=6）。"""

        if ack_status > 2:
            return
        fdbKP = min(int(fdbKP), 10000)
        fdbKD = min(int(fdbKD), 10000)

        data = bytes([
            (0xC0 | (ack_status & 0x03)) & 0xFF,
            0x03,
            (fdbKP >> 8) & 0xFF,
            fdbKP & 0xFF,
            (fdbKD >> 8) & 0xFF,
            fdbKD & 0xFF,
        ])
        self.can.send(CanFrame(arbitration_id=motor_id & 0x7FF, data=data))

    def set_motor_feedbackKP(self, motor_id: int, fdbKP: int, ack_status: int) -> None:
        self.set_motor_feedbackKP_KD(motor_id=motor_id, fdbKP=fdbKP, fdbKD=0, ack_status=ack_status)

    def set_motor_feedbackKD(self, motor_id: int, fdbKD: int, ack_status: int) -> None:
        self.set_motor_feedbackKP_KD(motor_id=motor_id, fdbKP=0, fdbKD=fdbKD, ack_status=ack_status)

    def get_motor_parameter(self, motor_id: int, param_cmd: int) -> None:
        """读取电机参数（协议: 0xE0 param_cmd，DLC=2）。"""

        data = bytes([0xE0, param_cmd & 0xFF])
        self.can.send(CanFrame(arbitration_id=motor_id & 0x7FF, data=data))


# -----------------------------
# 接收侧（对应 can_rv.c 的 RV_can_data_repack）
# -----------------------------

def RV_can_data_repack(rx: CanFrame, comm_mode: int) -> None:
    """解析电机回传 CAN 帧，更新全局 `rv_motor_msg`、`motor_comm_fbd`。

    参数：
    - rx: 接收到的 CAN 帧（StdId + Data[]）
    - comm_mode: `comm_ack`(0x00) 响应模式 / `comm_auto`(0x01) 自动反馈模式

    这段逻辑基本按原 C 代码逐行翻译。
    """

    global motor_id_check

    data = list(rx.data)

    if rx.arbitration_id == 0x7FF:
        # 0x7FF 是“电机设置/ID查询”类的反馈帧
        if len(data) < 4:
            return
        if data[2] != 0x01:
            return

        if data[0] == 0xFF and data[1] == 0xFF:
            if len(data) >= 5:
                motor_comm_fbd.motor_id = _u16_be(data[3], data[4])
                motor_comm_fbd.motor_fbd = 0x01
        elif data[0] == 0x80 and data[1] == 0x80:
            motor_comm_fbd.motor_id = 0
            motor_comm_fbd.motor_fbd = 0x80
        elif data[0] == 0x7F and data[1] == 0x7F:
            motor_comm_fbd.motor_id = 1
            motor_comm_fbd.motor_fbd = 0x05
        else:
            motor_comm_fbd.motor_id = _u16_be(data[0], data[1])
            motor_comm_fbd.motor_fbd = data[3] if len(data) >= 4 else 0
        return

    # --------------------
    # comm_ack：响应模式
    # --------------------
    if comm_mode == comm_ack:
        if len(data) < 1:
            return

        ack_status = (data[0] >> 5) & 0x07
        motor_id_t = (rx.arbitration_id - 1) & 0xFF
        motor_id_check = rx.arbitration_id

        if motor_id_t >= len(rv_motor_msg):
            return

        msg = rv_motor_msg[motor_id_t]
        msg.motor_id = motor_id_t
        msg.error = data[0] & 0x1F

        if ack_status == 1:
            # response frame 1
            if len(data) < 7:
                return
            pos_int = _u16_be(data[1], data[2])
            spd_int = ((data[3] & 0xFF) << 4) | ((data[4] & 0xF0) >> 4)
            cur_int = ((data[4] & 0x0F) << 8) | (data[5] & 0xFF)

            msg.angle_actual_rad = uint_to_float(pos_int, POS_MIN, POS_MAX, 16)
            msg.speed_actual_rad = uint_to_float(spd_int, SPD_MIN, SPD_MAX, 12)
            msg.current_actual_float = uint_to_float(cur_int, I_MIN, I_MAX, 12)
            msg.temperature = (float(data[6]) - 50.0) / 2.0

        elif ack_status == 2:
            # response frame 2
            if len(data) < 8:
                return
            b = bytes([data[4], data[3], data[2], data[1]])
            msg.angle_actual_float = struct.unpack("<f", b)[0]
            msg.current_actual_int = _u16_be(data[5], data[6])
            msg.temperature = (float(data[7]) - 50.0) / 2.0
            msg.current_actual_float = msg.current_actual_int / 100.0

        elif ack_status == 3:
            # response frame 3
            if len(data) < 8:
                return
            b = bytes([data[4], data[3], data[2], data[1]])
            msg.speed_actual_float = struct.unpack("<f", b)[0]
            msg.current_actual_int = _u16_be(data[5], data[6])
            msg.temperature = (float(data[7]) - 50.0) / 2.0
            msg.current_actual_float = msg.current_actual_int / 100.0

        elif ack_status == 4:
            # response frame 4
            if rx.dlc != 3 or len(data) < 3:
                return
            motor_comm_fbd.INS_code = data[1]
            motor_comm_fbd.motor_fbd = data[2]

        elif ack_status == 5:
            # response frame 5：读取参数的回包
            if len(data) < 2:
                return
            motor_comm_fbd.INS_code = data[1]

            # 注意：原 C 代码写法是 `INS_code==1 & DLC==6`，这是按位 `&`，
            # 效果等价于对两个布尔表达式做“非短路”的 AND（更接近“原封不动”）。
            if ((motor_comm_fbd.INS_code == 1) & (rx.dlc == 6)) and len(data) >= 6:
                b = bytes([data[5], data[4], data[3], data[2]])
                msg.angle_actual_float = struct.unpack("<f", b)[0]
            elif ((motor_comm_fbd.INS_code == 2) & (rx.dlc == 6)) and len(data) >= 6:
                b = bytes([data[5], data[4], data[3], data[2]])
                msg.speed_actual_float = struct.unpack("<f", b)[0]
            elif ((motor_comm_fbd.INS_code == 3) & (rx.dlc == 6)) and len(data) >= 6:
                b = bytes([data[5], data[4], data[3], data[2]])
                msg.current_actual_float = struct.unpack("<f", b)[0]
            elif ((motor_comm_fbd.INS_code == 4) & (rx.dlc == 6)) and len(data) >= 6:
                b = bytes([data[5], data[4], data[3], data[2]])
                msg.power = struct.unpack("<f", b)[0]
            elif ((motor_comm_fbd.INS_code == 5) & (rx.dlc == 4)) and len(data) >= 4:
                msg.acceleration = _u16_be(data[2], data[3])
            elif ((motor_comm_fbd.INS_code == 6) & (rx.dlc == 4)) and len(data) >= 4:
                msg.linkage_KP = _u16_be(data[2], data[3])
            elif ((motor_comm_fbd.INS_code == 7) & (rx.dlc == 4)) and len(data) >= 4:
                msg.speed_KI = _u16_be(data[2], data[3])
            elif ((motor_comm_fbd.INS_code == 8) & (rx.dlc == 4)) and len(data) >= 4:
                msg.feedback_KP = _u16_be(data[2], data[3])
            elif ((motor_comm_fbd.INS_code == 9) & (rx.dlc == 4)) and len(data) >= 4:
                msg.feedback_KD = _u16_be(data[2], data[3])

        return

    # --------------------
    # comm_auto：自动反馈模式
    # --------------------
    if comm_mode == comm_auto:
        motor_id_t = rx.arbitration_id - 0x205
        if motor_id_t < 0 or motor_id_t >= len(rv_motor_msg):
            return
        if len(data) < 8:
            return

        msg = rv_motor_msg[motor_id_t]
        msg.angle_actual_int = _u16_be(data[0], data[1])
        msg.speed_actual_int = _s16_be(data[2], data[3])
        msg.current_actual_int = _u16_be(data[4], data[5])
        msg.temperature = float(data[6])
        msg.error = data[7]

        return
