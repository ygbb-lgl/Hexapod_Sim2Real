try:
    from el4090_motor_sdk import *
except ImportError:
    from .el4090_motor_sdk import *

import sys
import time
import math
import threading

import pysoem

# ==================== Configuration ====================
IFNAME = "enp86s0"  # <--- 修改为你的网卡


class RL_Real_Position_PySOEM:
    """EtherCAT 伺服位置模式控制。

    官方 C SDK 对应：`set_motor_position`。

    约定：
    - 单电机测试：slave3 / passage1 / motor_id=19
    - 位置命令以 policy index 对齐：`target_position_deg`
    - 命令单位用官方位置模式的 degree；状态里的 position 仍保持 rad
    """

    def __init__(self, ifname: str, *, motor_id: int = 19):
        self.ifname = ifname
        self.single_motor_id = int(motor_id)

        self.num_dofs = 1
        self.policy_to_motor_id_ = [self.single_motor_id]
        self.motor_direction_ = {self.single_motor_id: 1.0}
        self.motor_offset_ = {self.single_motor_id: 0.0}
        self.motor_ethercat_addr_ = {self.single_motor_id: {"slave": 3, "passage": 1}}

        self.motor_command_buffer = type("CommandBuffer", (), {})()
        self.motor_command_buffer.target_position_deg = [0.0] * self.num_dofs
        self.motor_command_buffer.speed_limit_01rpm = [50] * self.num_dofs
        self.motor_command_buffer.current_limit_01a = [500] * self.num_dofs
        self.motor_command_buffer.ack_status = [2] * self.num_dofs

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

    def HardwareSendPosition(self):
        """把 policy 的位置指令映射到 motor_id，并按 EtherCAT passage 下发。"""
        for policy_idx in range(self.num_dofs):
            motor_id = self.policy_to_motor_id_[policy_idx]
            if motor_id not in self.motor_ethercat_addr_:
                continue

            addr = self.motor_ethercat_addr_[motor_id]
            slave_idx = addr["slave"]
            if slave_idx >= len(self.master.slaves):
                continue

            target_rad = math.radians(float(self.motor_command_buffer.target_position_deg[policy_idx]))
            motor_pos_rad = self.motor_direction_[motor_id] * target_rad + self.motor_offset_[motor_id]
            motor_pos_deg = math.degrees(motor_pos_rad)
            spd_01rpm = int(self.motor_command_buffer.speed_limit_01rpm[policy_idx])
            cur_01a = int(self.motor_command_buffer.current_limit_01a[policy_idx])
            ack = int(self.motor_command_buffer.ack_status[policy_idx])

            tx_msg = self.motorData.getTxMsg(slave_idx)
            set_motor_position(
                tx_msg,
                addr["passage"],
                motor_id,
                pos_deg=motor_pos_deg,
                spd_01rpm=spd_01rpm,
                cur_01a=cur_01a,
                ack_status=ack,
            )
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

            ack = int(self.motor_command_buffer.ack_status[policy_idx])
            if ack == 2:
                motor_pos = math.radians(float(motor_msg.angle_actual_float))
            else:
                motor_pos = float(motor_msg.angle_actual_rad)

            urdf_angle = self.motor_direction_[motor_id] * (motor_pos - self.motor_offset_[motor_id])
            self.motor_state_buffer.position[policy_idx] = urdf_angle

            if ack == 3:
                motor_vel = float(motor_msg.speed_actual_float) * 2.0 * math.pi / 60.0
            else:
                motor_vel = float(motor_msg.speed_actual_rad)
            self.motor_state_buffer.velocity[policy_idx] = self.motor_direction_[motor_id] * motor_vel
            self.motor_state_buffer.torque[policy_idx] = motor_msg.current_actual_float

    def pd_loop(self):
        print("Starting Process Data Loop (Position Mode)...")
        self.running = True

        TARGET_FREQ = 250.0
        dt = 1.0 / TARGET_FREQ
        next_wake_time = time.perf_counter()

        try:
            while self.running:
                self.master.send_processdata()
                self.master.receive_processdata(2000)

                for i, slave in enumerate(self.master.slaves):
                    if getattr(slave, "input", None):
                        rx = self._msg_cls()
                        ctypes.memmove(ctypes.addressof(rx), slave.input, ctypes.sizeof(self._msg_cls))
                        self.motorData.setRxMsgRaw(i, rx)

                self.HardwareRecv()
                self.HardwareSendPosition()

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


def _find_policy_index_by_motor_id(robot: RL_Real_Position_PySOEM, motor_id: int) -> int:
    for i, mid in enumerate(robot.policy_to_motor_id_):
        if int(mid) == int(motor_id):
            return i
    raise ValueError(f"motor_id {motor_id} not found in joint2motor_idx")


if __name__ == "__main__":
    if len(sys.argv) > 1:
        IFNAME = sys.argv[1]

    motor_id = int(sys.argv[2]) if len(sys.argv) > 2 else 19
    const_position_deg = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
    speed_limit_01rpm = int(sys.argv[4]) if len(sys.argv) > 4 else 50
    current_limit_01a = int(sys.argv[5]) if len(sys.argv) > 5 else 500
    ack_status = int(sys.argv[6]) if len(sys.argv) > 6 else 2

    print(
        f"Launching Position Mode on interface {IFNAME}, motor_id={motor_id}, "
        f"target={const_position_deg:.3f} deg"
    )

    robot = None
    try:
        robot = RL_Real_Position_PySOEM(IFNAME, motor_id=motor_id)
        idx = _find_policy_index_by_motor_id(robot, motor_id)

        for i in range(robot.num_dofs):
            robot.motor_command_buffer.target_position_deg[i] = 0.0
            robot.motor_command_buffer.speed_limit_01rpm[i] = speed_limit_01rpm
            robot.motor_command_buffer.current_limit_01a[i] = current_limit_01a
            robot.motor_command_buffer.ack_status[i] = ack_status

        if not robot.start():
            print("Failed to start EtherCAT.")
            sys.exit(1)

        print("Loop running. Ctrl+C to stop.")

        t0 = time.time()
        while True:
            t = time.time() - t0

            robot.motor_command_buffer.target_position_deg[idx] = const_position_deg

            if int(t * 10) % 10 == 0:
                v = robot.motor_state_buffer.velocity[idx]
                p = robot.motor_state_buffer.position[idx]
                c = robot.motor_state_buffer.torque[idx]
                print(
                    f"t={t:6.2f}s cmd_pos(deg)={const_position_deg:8.3f} "
                    f"pos(rad)={p:8.3f} vel(rad/s)={v:8.3f} cur={c:8.3f}"
                )

            time.sleep(0.02)

    except KeyboardInterrupt:
        print("Stopping...")
    finally:
        if robot is not None:
            robot.stop()
