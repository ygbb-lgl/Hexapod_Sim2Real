try:
    from el4090_motor_sdk import *
except ImportError:
    from .el4090_motor_sdk import *

import os
import sys
import time
import math
import threading

import pysoem

# ==================== Configuration ====================
IFNAME = "enp109s0"  # <--- 修改为你的网卡


class RL_Real_Speed_PySOEM:
    """EtherCAT 速度模式控制（基于你现有 pd_loop 框架，替换应用层指令为 set_motor_speed）。

    约定：
    - 仍然使用 hexapod.yaml 的 joint2motor_idx / motor_directions / motor_offsets 做映射
    - 速度指令以 policy index 对齐（长度 num_dofs）
    - 下发 set_motor_speed 所需的 spd_rpm / cur_01a

    注意：此模式下，电机内部是速度伺服；这里仍然会解析回读（位置/速度/电流）用于监控。
    """

    def __init__(self, ifname: str, *, motor_id: int = 1):
        self.ifname = ifname
        self.single_motor_id = int(motor_id)

        # 单板单电机专用：motor_id -> slave0/passage1
        # 约定：只有 1 个从站板子，且电机挂在该板子的第 1 路（passage=1）。
        self.num_dofs = 1
        self.policy_to_motor_id_ = [self.single_motor_id]
        self.motor_direction_ = {self.single_motor_id: 1.0}
        self.motor_offset_ = {self.single_motor_id: 0.0}
        self.motor_ethercat_addr_ = {self.single_motor_id: {"slave": 0, "passage": 1}}

        # Command/State buffers
        self.motor_command_buffer = type("CommandBuffer", (), {})()
        self.motor_command_buffer.target_speed_rpm = [0.0] * self.num_dofs
        self.motor_command_buffer.current_limit_01a = [500] * self.num_dofs  # 50.0A 默认
        self.motor_command_buffer.ack_status = [1] * self.num_dofs

        self.motor_state_buffer = type("StateBuffer", (), {})()
        self.motor_state_buffer.position = [0.0] * self.num_dofs
        self.motor_state_buffer.velocity = [0.0] * self.num_dofs
        self.motor_state_buffer.torque = [0.0] * self.num_dofs

        self.motorData = None
        self._msg_cls = None

        self.master = pysoem.Master()
        self.master.open(self.ifname)

        self.running = False
        self.thread = None

    def _write_slave_output(self, slave, payload: bytes):
        if not hasattr(slave, "output") or slave.output is None:
            return

        out_len = len(slave.output)
        if out_len <= 0:
            return

        data = payload
        if len(data) < out_len:
            data = data + (b"\x00" * (out_len - len(data)))
        elif len(data) > out_len:
            data = data[:out_len]

        try:
            slave.output[:] = data
        except Exception:
            slave.output = data

    def init_ethercat(self) -> bool:
        print(f"Initializing EtherCAT on {self.ifname}...")

        if self.master.config_init() <= 0:
            print("No slaves found.")
            return False

        print(f"Found {len(self.master.slaves)} slaves.")

        for slave in self.master.slaves:
            if hasattr(slave, "_disable_complete_access"):
                slave._disable_complete_access()
            elif hasattr(slave, "coedetails"):
                try:
                    slave.coedetails = int(slave.coedetails) & ~0x08
                except Exception:
                    pass

        io_map_size = self.master.config_map()
        if io_map_size <= 0:
            print("EtherCAT config_map failed.")
            return False

        print(f"EtherCAT config_map successful. Total IO size: {io_map_size} bytes")
        self.master.config_dc()

        in_lens, out_lens = [], []
        for i, slave in enumerate(self.master.slaves):
            in_len = len(slave.input) if getattr(slave, "input", None) is not None else 0
            out_len = len(slave.output) if getattr(slave, "output", None) is not None else 0
            in_lens.append(in_len)
            out_lens.append(out_len)
            print(f"Slave {i} PDO sizes: IN={in_len} bytes, OUT={out_len} bytes")

        pdo_bytes = max(in_lens + out_lens)
        self._msg_cls = make_ethercat_msg_type(pdo_bytes)
        print(f"Using EtherCAT message size: {ctypes.sizeof(self._msg_cls)} bytes")

        self.motorData = MotorData(slave_count=len(self.master.slaves), msg_cls=self._msg_cls)

        for slave_idx in range(len(self.master.slaves)):
            tx0 = self.motorData.getTxMsg(slave_idx)
            tx0.motor_num = 6
            tx0.can_ide = 1
            self.motorData.setTxMsg(slave_idx, tx0)
            self._write_slave_output(self.master.slaves[slave_idx], bytes(tx0))

        if self.master.state_check(pysoem.SAFEOP_STATE, 50000) != pysoem.SAFEOP_STATE:
            print("Failed to reach SAFEOP state.")
            self.master.read_state()
            for slave in self.master.slaves:
                print(f"Slave {slave.name} state: {slave.state}")
            return False

        print("Requesting OP state...")
        self.master.state = pysoem.OP_STATE
        for slave in self.master.slaves:
            try:
                slave.state = pysoem.OP_STATE
            except Exception:
                pass

        self.master.send_processdata()
        self.master.receive_processdata(2000)
        if hasattr(self.master, "write_state"):
            self.master.write_state()

        for _ in range(40):
            self.master.send_processdata()
            self.master.receive_processdata(2000)
            self.master.state_check(pysoem.OP_STATE, 50000)
            if all(slave.state == pysoem.OP_STATE for slave in self.master.slaves):
                print("All slaves reached OP state.")
                return True

        print("Failed to reach OP state.")
        self.master.read_state()
        for i, slave in enumerate(self.master.slaves):
            print(f"Slave {i} {slave.name} state: {slave.state}")
        return False

    def HardwareSendSpeed(self):
        """把 policy 的速度指令映射到对应 motor_id，并按 EtherCAT passage 下发。"""
        for policy_idx in range(self.num_dofs):
            motor_id = self.policy_to_motor_id_[policy_idx]
            if motor_id not in self.motor_ethercat_addr_:
                continue

            addr = self.motor_ethercat_addr_[motor_id]
            slave_idx = addr["slave"]
            if slave_idx >= len(self.master.slaves):
                continue

            # 速度方向统一：如果你的 motor_direction 定义为“URDF->Motor”，速度也应一致。
            spd_rpm = self.motor_direction_[motor_id] * float(self.motor_command_buffer.target_speed_rpm[policy_idx])
            cur_01a = int(self.motor_command_buffer.current_limit_01a[policy_idx])
            ack = int(self.motor_command_buffer.ack_status[policy_idx])

            tx_msg = self.motorData.getTxMsg(slave_idx)
            set_motor_speed(tx_msg, addr["passage"], motor_id, spd_rpm=spd_rpm, cur_01a=cur_01a, ack_status=ack)
            self.motorData.setTxMsg(slave_idx, tx_msg)

    def HardwareRecv(self):
        for policy_idx in range(self.num_dofs):
            motor_id = self.policy_to_motor_id_[policy_idx]
            if motor_id not in self.motor_ethercat_addr_:
                continue

            addr = self.motor_ethercat_addr_[motor_id]
            slave_idx = addr["slave"]
            if slave_idx >= len(self.master.slaves):
                continue

            motor_msg = self.motorData.getRxMotorMsg(slave_idx, addr["passage"])
            if motor_msg.motor_id != motor_id:
                continue

            motor_pos = motor_msg.angle_actual_rad
            urdf_angle = self.motor_direction_[motor_id] * (motor_pos - self.motor_offset_[motor_id])

            self.motor_state_buffer.position[policy_idx] = urdf_angle
            self.motor_state_buffer.velocity[policy_idx] = self.motor_direction_[motor_id] * motor_msg.speed_actual_rad
            self.motor_state_buffer.torque[policy_idx] = motor_msg.current_actual_float

    def pd_loop(self):
        print("Starting Process Data Loop (Speed Mode)...")
        self.running = True

        TARGET_FREQ = 150.0
        dt = 1.0 / TARGET_FREQ
        next_wake_time = time.perf_counter()

        try:
            while self.running:
                self.master.send_processdata()
                self.master.receive_processdata(2000)

                # input bytes -> sdk rx
                for i, slave in enumerate(self.master.slaves):
                    if getattr(slave, "input", None):
                        rx = self._msg_cls()
                        ctypes.memmove(ctypes.addressof(rx), slave.input, ctypes.sizeof(self._msg_cls))
                        self.motorData.setRxMsgRaw(i, rx)

                self.HardwareRecv()
                self.HardwareSendSpeed()

                for i, slave in enumerate(self.master.slaves):
                    tx_msg = self.motorData.getTxMsg(i)
                    self._write_slave_output(slave, bytes(tx_msg))

                now = time.perf_counter()
                sleep_time = next_wake_time - now
                if sleep_time > 0:
                    time.sleep(sleep_time)
                next_wake_time += dt

        except Exception as e:
            print(f"Exception in control loop: {e}")
        finally:
            self.running = False

    def start(self) -> bool:
        if self.init_ethercat():
            self.thread = threading.Thread(target=self.pd_loop, daemon=True)
            self.thread.start()
            return True
        return False

    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join(timeout=2.0)
        self.master.close()


def _find_policy_index_by_motor_id(robot: RL_Real_Speed_PySOEM, motor_id: int) -> int:
    for i, mid in enumerate(robot.policy_to_motor_id_):
        if int(mid) == int(motor_id):
            return i
    raise ValueError(f"motor_id {motor_id} not found in joint2motor_idx")


if __name__ == "__main__":
    if len(sys.argv) > 1:
        IFNAME = sys.argv[1]

    # 可选: 第二个参数指定 motor_id（更方便单电机测试）
    motor_id = int(sys.argv[2]) if len(sys.argv) > 2 else 4

    # 可选: 第三个参数 指定恒速转动指令（rpm）
    const_rpm = float(sys.argv[3]) if len(sys.argv) > 3 else 100.0

    print(f"Launching Speed Mode on interface {IFNAME}, motor_id={motor_id}")

    robot = None
    try:
        robot = RL_Real_Speed_PySOEM(IFNAME, motor_id=motor_id)
        idx = _find_policy_index_by_motor_id(robot, motor_id)

        # 初始化：所有电机速度为 0
        for i in range(robot.num_dofs):
            robot.motor_command_buffer.target_speed_rpm[i] = 0.0
            robot.motor_command_buffer.current_limit_01a[i] = 500
            robot.motor_command_buffer.ack_status[i] = 1

        if not robot.start():
            print("Failed to start EtherCAT.")
            sys.exit(1)

        print("Loop running. Ctrl+C to stop.")

        t0 = time.time()
        while True:
            t = time.time() - t0

            # 恒速转动（rpm）
            robot.motor_command_buffer.target_speed_rpm[idx] = const_rpm

            if int(t * 10) % 10 == 0:
                v = robot.motor_state_buffer.velocity[idx]
                p = robot.motor_state_buffer.position[idx]
                c = robot.motor_state_buffer.torque[idx]
                print(
                    f"t={t:6.2f}s cmd_rpm={const_rpm:8.1f} "
                    f"vel(rad/s)={v:8.3f} pos(rad)={p:8.3f} cur={c:8.3f}"
                )

            time.sleep(0.02)

    except KeyboardInterrupt:
        print("Stopping...")
    finally:
        if robot is not None:
            robot.stop()
