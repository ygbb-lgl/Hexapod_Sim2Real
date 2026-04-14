from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_ as LowCmdGo
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_ as LowCmdHG
from typing import Union


class MotorMode:
    PR = 0  # Series Control for Pitch/Roll Joints
    AB = 1  # Parallel Control for A/B Joints

# dq和 qd 的兼容性处理，部分版本的 SDK 使用 qd 作为速度命令字段名，而较新的版本使用 dq。
def _set_motor_dq(motor_cmd, value: float) -> None:
    # Unitree SDK2 Python uses field name `dq` in MotorCmd_.
    # Some older variants use `qd`, so keep a small compatibility shim.
    if hasattr(motor_cmd, "dq"):
        motor_cmd.dq = value
    else:
        motor_cmd.qd = value

# 进入阻尼模式，关电机力矩，增加阻尼
def create_damping_cmd(cmd: Union[LowCmdGo, LowCmdHG]):
    size = len(cmd.motor_cmd)
    for i in range(size):
        cmd.motor_cmd[i].q = 0
        # 其实就是设置dq=0
        _set_motor_dq(cmd.motor_cmd[i], 0)
        cmd.motor_cmd[i].kp = 0
        cmd.motor_cmd[i].kd = 8
        cmd.motor_cmd[i].tau = 0

# 进入零力模式，关电机力矩，不增加阻尼
def create_zero_cmd(cmd: Union[LowCmdGo, LowCmdHG]):
    size = len(cmd.motor_cmd)
    for i in range(size):
        cmd.motor_cmd[i].q = 0
        _set_motor_dq(cmd.motor_cmd[i], 0)
        cmd.motor_cmd[i].kp = 0
        cmd.motor_cmd[i].kd = 0
        cmd.motor_cmd[i].tau = 0


def init_cmd_hg(cmd: LowCmdHG, mode_machine: int, mode_pr: int):
    cmd.mode_machine = mode_machine
    cmd.mode_pr = mode_pr
    size = len(cmd.motor_cmd)
    for i in range(size):
        cmd.motor_cmd[i].mode = 1
        cmd.motor_cmd[i].q = 0
        _set_motor_dq(cmd.motor_cmd[i], 0)
        cmd.motor_cmd[i].kp = 0
        cmd.motor_cmd[i].kd = 0
        cmd.motor_cmd[i].tau = 0


def init_cmd_go(cmd: LowCmdGo, weak_motor: list):
    cmd.head[0] = 0xFE
    cmd.head[1] = 0xEF
    cmd.level_flag = 0xFF
    cmd.gpio = 0
    PosStopF = 2.146e9
    VelStopF = 16000.0
    size = len(cmd.motor_cmd)
    for i in range(size):
        if i in weak_motor:
            cmd.motor_cmd[i].mode = 1
        else:
            cmd.motor_cmd[i].mode = 0x0A
        cmd.motor_cmd[i].q = PosStopF
        _set_motor_dq(cmd.motor_cmd[i], VelStopF)
        cmd.motor_cmd[i].kp = 0
        cmd.motor_cmd[i].kd = 0
        cmd.motor_cmd[i].tau = 0
