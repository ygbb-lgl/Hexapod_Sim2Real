"""run_can_rv_demo.py

你说“不用终端接口”，所以这个文件现在主要提供一个可直接 import 的封装类：`RVMotorDriver`。

它做的事情很简单：
- 你传入控制参数（motor_id/kp/kd/pos/spd/tor 等）
- 它按电机协议打包成 CAN 帧，并交给你提供的发送接口发出去

默认不依赖任何硬件：不传接口时只打印帧，方便你先验证协议打包是否正确。

如果你后续确认自己用的是 SocketCAN（Linux 上的 can0），也可以用 `RVMotorDriver.socketcan()`。
如果你用的是 PCAN-USB（和你 servo 例子一样），可以用 `RVMotorDriver.pcan()`。

注意：
- 这份封装只负责“协议层”。真正让电机动起来还取决于：电机上电、CAN 波特率、终端电阻、ID、使能流程等。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Optional
import time

try:
    # Package import (e.g. imported as hexapod_tethered_utils.leg_joints_motions.run_can_rv)
    from .can_rv import *
except ImportError:  # pragma: no cover
    # Fallback for running this file directly.
    from can_rv import *




SendFn = Callable[[CanFrame], None]


class PythonCanInterface:
    """把 ` CanInterface` 适配到 python-can（SocketCAN）。

    只有在你确实有 `can0` 并且安装了 `python-can` 时才需要。
    """

    def __init__(self, channel: str = "can0", bustype: str = "socketcan"):
        try:
            import can  # type: ignore
        except Exception as exc:  # pragma: no cover
            raise RuntimeError("未安装 python-can。请先执行: pip install python-can") from exc

        self._can = can
        # python-can v4.2+ 推荐使用 interface=；bustype= 已弃用。
        self._bus = can.interface.Bus(channel=channel, interface=bustype)

    def send(self, frame:  CanFrame) -> None:
        msg = self._can.Message(
            arbitration_id=frame.arbitration_id,
            is_extended_id=frame.is_extended_id,
            data=frame.data,
        )
        self._bus.send(msg)


class PCANInterface:
    """把 ` CanInterface` 适配到 python-can（PCAN）。

    对应你给的 servo 示例：
        can.interface.Bus(channel='PCAN_USBBUS1', bustype='pcan', bitrate=1000000)

    说明：
    - PCAN 这类驱动通常允许在打开设备时设置 `bitrate`。
    - channel 常见是 'PCAN_USBBUS1'（也可能是 USBBUS2 等）。
    """

    def __init__(self, channel: str = "PCAN_USBBUS1", bitrate: int = 1_000_000):
        try:
            import can  # type: ignore
        except Exception as exc:  # pragma: no cover
            raise RuntimeError("未安装 python-can。请先执行: pip install python-can") from exc

        self._can = can
        # python-can v4.2+ 推荐使用 interface=；bustype= 已弃用。
        self._bus = can.interface.Bus(channel=channel, interface="pcan", bitrate=int(bitrate))

    def send(self, frame:  CanFrame) -> None:
        msg = self._can.Message(
            arbitration_id=frame.arbitration_id,
            is_extended_id=frame.is_extended_id,
            data=frame.data,
        )
        self._bus.send(msg)


@dataclass
class RVMotorDriver:
    """给主程序调用的封装类。

    - 不需要命令行参数。
    - 主程序只管传入控制参数。
    - 发送方式可插拔：默认打印；也可以换成 SocketCAN 或你自己的发送函数。
    """

    can_iface: Optional[ CanInterface] = None

    def __post_init__(self) -> None:
        if self.can_iface is None:
            self.can_iface =  PrintCanInterface()
        self._motor =  RVCanMotor(can=self.can_iface)

    # ---------- 构造快捷方式 ----------

    @classmethod
    def print_only(cls) -> "RVMotorDriver":
        """不接硬件：只打印将要发送的帧。"""

        return cls(can_iface= PrintCanInterface())

    @classmethod
    def socketcan(cls, channel: str = "can0") -> "RVMotorDriver":
        """使用 SocketCAN（需要 can0 + python-can）。"""

        return cls(can_iface=PythonCanInterface(channel=channel))

    @classmethod
    def pcan(cls, channel: str = "PCAN_USBBUS1", bitrate: int = 1_000_000) -> "RVMotorDriver":
        """使用 PCAN（需要 python-can + PCAN 驱动）。"""

        return cls(can_iface=PCANInterface(channel=channel, bitrate=bitrate))

    @classmethod
    def with_send_fn(cls, send_fn: SendFn) -> "RVMotorDriver":
        """使用你自定义的发送函数（比如串口网关/网络转发）。"""

        class _FnIface:
            def __init__(self, fn: SendFn):
                self._fn = fn

            def send(self, frame:  CanFrame) -> None:
                self._fn(frame)

        return cls(can_iface=_FnIface(send_fn))

    # ---------- 发送侧 API（参数和 C 版一致） ----------

    def motor_setting(self, motor_id: int, cmd: int) -> None:
        self._motor.MotorSetting(motor_id=motor_id, cmd=cmd)

    def set_comm_auto(self, motor_id: int) -> None:
        """切换到自动反馈模式（comm_auto）。

        对应官方 C demo 的注释：cmd=0x01 -> automatic feedback。
        """

        self.motor_setting(motor_id=motor_id, cmd=0x01)

    def set_comm_ack(self, motor_id: int) -> None:
        """切换到响应/问答模式（comm_ack）。

        对应官方 C demo 的注释：cmd=0x02 -> response。
        """

        self.motor_setting(motor_id=motor_id, cmd=0x02)

    def motor_id_reset(self) -> None:
        self._motor.MotorIDReset()

    def motor_id_setting(self, motor_id: int, motor_id_new: int) -> None:
        self._motor.MotorIDSetting(motor_id=motor_id, motor_id_new=motor_id_new)

    def motor_comm_mode_reading(self, motor_id: int) -> None:
        self._motor.MotorCommModeReading(motor_id=motor_id)

    def motor_id_reading(self) -> None:
        self._motor.MotorIDReading()

    def send_ctrl(self, motor_id: int, kp: float, kd: float, pos: float, spd: float, tor: float) -> None:
        """对应 C 的 `send_motor_ctrl_cmd`。"""

        self._motor.send_motor_ctrl_cmd(motor_id=motor_id, kp=kp, kd=kd, pos=pos, spd=spd, tor=tor)

    def set_position(self, motor_id: int, pos: float, spd: int, cur: int, ack_status: int = 0) -> None:
        self._motor.set_motor_position(motor_id=motor_id, pos=pos, spd=spd, cur=cur, ack_status=ack_status)

    def set_speed(self, motor_id: int, spd: float, cur: int, ack_status: int = 0) -> None:
        self._motor.set_motor_speed(motor_id=motor_id, spd=spd, cur=cur, ack_status=ack_status)

    def set_current_or_torque(self, motor_id: int, cur_tor: int, ctrl_status: int = 0, ack_status: int = 0) -> None:
        self._motor.set_motor_cur_tor(
            motor_id=motor_id,
            cur_tor=cur_tor,
            ctrl_status=ctrl_status,
            ack_status=ack_status,
        )

    def set_acceleration(self, motor_id: int, acc: int, ack_status: int = 0) -> None:
        self._motor.set_motor_acceleration(motor_id=motor_id, acc=acc, ack_status=ack_status)

    def set_feedback_kp_kd(self, motor_id: int, kp: int, kd: int, ack_status: int = 0) -> None:
        self._motor.set_motor_feedbackKP_KD(motor_id=motor_id, fdbKP=kp, fdbKD=kd, ack_status=ack_status)

    def get_parameter(self, motor_id: int, param_cmd: int) -> None:
        self._motor.get_motor_parameter(motor_id=motor_id, param_cmd=param_cmd)

    # ---------- 接收解包 ----------

    def on_frame(self, arbitration_id: int, data: bytes, comm_mode: int =  comm_ack) -> None:
        """把接收到的一帧交给解包器更新全局状态。"""

        RV_can_data_repack( CanFrame(arbitration_id=arbitration_id, data=data), comm_mode=comm_mode)

    @property
    def motor_states(self):
        """返回全局电机状态数组（长度 8），与 C 版 `rv_motor_msg[8]` 对齐。"""

        return  rv_motor_msg

    # ---------- （可选）接收辅助：复用同一个 driver 的 bus ----------

    def recv_raw(self, timeout: float = 1.0):
        """从 driver 内部的 python-can bus 接收一帧并返回。

        只有当你用的是 `RVMotorDriver.pcan(...)` 或 `RVMotorDriver.socketcan(...)`
        这种内部持有 python-can `Bus` 的接口时才可用。
        """

        bus = getattr(self.can_iface, "_bus", None)
        if bus is None:
            raise RuntimeError(
                "当前 driver 没有可用的接收 bus：\n"
                "- 如果你想接收回包，请用 RVMotorDriver.pcan()/socketcan() 创建\n"
                "- 或者你自己在外部创建 bus.recv()，再把帧交给 driver.on_frame() 解包"
            )
        return bus.recv(timeout=timeout)

    def recv_and_unpack(self, timeout: float = 1.0, comm_mode: int =  comm_ack) -> bool:
        """接收一帧并自动调用 on_frame 解包；成功收到返回 True，否则 False。"""

        rx = self.recv_raw(timeout=timeout)
        if rx is None:
            return False
        self.on_frame(arbitration_id=rx.arbitration_id, data=bytes(rx.data), comm_mode=comm_mode)
        return True

    def drain_and_unpack(self, comm_mode: int, max_frames: int = 100) -> int:
        """非阻塞地尽量多接收并解包（用于高频控制循环）。

        - 每次最多处理 max_frames 帧，避免在极端情况下占用太久。
        - 内部使用 timeout=0.0，因此不会阻塞。
        """

        count = 0
        for _ in range(int(max_frames)):
            rx = self.recv_raw(timeout=0.0)
            if rx is None:
                break
            self.on_frame(arbitration_id=rx.arbitration_id, data=bytes(rx.data), comm_mode=comm_mode)
            count += 1
        return count

    def get_auto_angle_rad(self, motor_id: int) -> float:
        """从 comm_auto 自动反馈的原始 int 位置，换算成弧度。

        注意：comm_auto 回包里 angle_actual_int 是 uint16；官方 demo 未给出单位，
        但它与 ask/response 模式下的 POS_MIN/POS_MAX 映射是一致的（16bit 定点）。
        """

        st = self.motor_states[motor_id - 1]
        return  uint_to_float(int(st.angle_actual_int),  POS_MIN,  POS_MAX, 16)

    def get_angle(self, motor_id: int, timeout: float = 0.002) -> float | None:
        """comm_ack 模式下读取“位置(角度)”。

        - 发起一次 param_get_pos 查询
        - 在 timeout 时间内轮询接收
        - 收到对应 INS_code 后返回 angle_actual_float
        - 超时返回 None（保证不会无限卡住）
        """

        motor_comm_fbd.INS_code = 0
        self.get_parameter(motor_id=motor_id, param_cmd= param_get_pos)

        deadline = time.time() + float(timeout)
        while time.time() < deadline:
            if not self.recv_and_unpack(timeout=0.0, comm_mode= comm_ack):
                time.sleep(0.0002)
                continue
            if  motor_comm_fbd.INS_code ==  param_get_pos:
                return self.motor_states[motor_id - 1].angle_actual_float
        return None

    def get_speed(self, motor_id: int, timeout: float = 0.002) -> float | None:
        """comm_ack 模式下读取“转速”。超时返回 None。"""

        motor_comm_fbd.INS_code = 0
        self.get_parameter(motor_id=motor_id, param_cmd= param_get_spd)

        deadline = time.time() + float(timeout)
        while time.time() < deadline:
            if not self.recv_and_unpack(timeout=0.0, comm_mode= comm_ack):
                time.sleep(0.0002)
                continue
            if  motor_comm_fbd.INS_code ==  param_get_spd:
                return self.motor_states[motor_id - 1].speed_actual_float
        return None


if __name__ == "__main__":
    # 仅用于快速自检：不需要任何硬件。
    # driver = RVMotorDriver.print_only()
    # driver.send_ctrl(motor_id=1, kp=20.0, kd=5.0, pos=0.0, spd=0.0, tor=0.0)

    # 2) comm_ack（问答/响应）最小示例（最简单版）：
    #    - 先发送 MotorSetting(cmd=0x02) 切到响应模式
    #    - 每次：get_parameter(param_get_pos) 请求一次位置
    #    - 然后 recv_and_unpack(timeout=...) 阻塞等一帧回包
    #    - 直接从 motor_states 里读 angle_actual_float

    motor_id = 1

    # 这里用“同一个 driver”同时负责发送+接收：内部只打开一次 PCAN 句柄。
    # 如果你希望走 can0，把这一行换成：driver = RVMotorDriver.socketcan(channel="can0")
    driver = RVMotorDriver.pcan(channel="PCAN_USBBUS1", bitrate=1_000_000)

    # 切到 comm_ack 响应模式
    driver.set_comm_ack(motor_id=motor_id)
    time.sleep(0.05)

    for _ in range(50):
        driver.get_parameter(motor_id=motor_id, param_cmd= param_get_pos)
        ok = driver.recv_and_unpack(timeout=0.02, comm_mode= comm_ack)
        if not ok:
            print(f"motor_id={motor_id} no response")
        else:
            pos = driver.motor_states[motor_id - 1].angle_actual_float
            print(f"motor_id={motor_id} pos={pos}")
        time.sleep(0.1)

    # 2.1) comm_ack（问答/响应）更稳健版（保留，暂时不用就注释）
    # 说明：这个版本会在 0.3s 内循环接收，直到匹配到 param_get_pos 的回包。
    #
    # driver.set_comm_ack(motor_id=motor_id)
    # time.sleep(0.05)
    #
    # for _ in range(50):
    #     # 发起一次“读取位置”的请求（不控制电机运动）
    #      motor_comm_fbd.INS_code = 0
    #     driver.get_parameter(motor_id=motor_id, param_cmd= param_get_pos)
    #
    #     # 阻塞等回包（最多等 0.3s）
    #     deadline = time.time() + 0.3
    #     got = False
    #     while time.time() < deadline:
    #         if not driver.recv_and_unpack(timeout=0.05, comm_mode= comm_ack):
    #             continue
    #         if  motor_comm_fbd.INS_code ==  param_get_pos:
    #             got = True
    #             break
    #
    #     if not got:
    #         print(f"motor_id={motor_id} no response")
    #     else:
    #         pos = driver.motor_states[motor_id - 1].angle_actual_float
    #         print(f"motor_id={motor_id} pos={pos}")
    #
    #     time.sleep(0.1)

    # ------------------------------
    # 3) comm_auto（自动反馈）示例（先保留，暂时不用就注释）
    #    - 先发送 MotorSetting(cmd=0x01) 切到自动反馈
    #    - 然后电机会周期性上报状态（StdId 通常是 0x205 + index）
    #    - 你的控制循环里用 drain_and_unpack(comm_auto) 非阻塞更新最新状态
    #
    # 取消下面整段注释即可切回 comm_auto。
    #
    # driver.set_comm_auto(motor_id=motor_id)
    #
    # ctrl_dt = 0.02  # 50Hz
    # for _ in range(200):
    #     # 非阻塞接收并解包（把本周期收到的回包尽量都处理掉）
    #     driver.drain_and_unpack(comm_mode= comm_auto, max_frames=100)
    #
    #     # 读取“最新缓存”的角度（弧度）
    #     angle = driver.get_auto_angle_rad(motor_id=motor_id)
    #     print(f"motor_id={motor_id} angle(rad)={angle}")
    #
    #     # 你的 50Hz 控制发送可以放在这里：
    #     # driver.send_ctrl(motor_id=motor_id, kp=10.0, kd=0.5, pos=0.0, spd=0.0, tor=0.0)
    #
    #     time.sleep(ctrl_dt)
