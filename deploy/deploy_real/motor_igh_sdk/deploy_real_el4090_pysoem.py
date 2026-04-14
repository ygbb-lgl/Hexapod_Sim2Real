import sys
import time
import ctypes
import threading
import math
import pysoem
import yaml
import os
try:
    from el4090_motor_sdk import *
except ImportError:
    from .el4090_motor_sdk import *

# ==================== Configuration ====================
IFNAME = 'wlo1'  # <--- 请修改为你实际的网卡名称，例如 'enp3s0'

# ==================== RL_Real Class with Real EtherCAT ====================

class RL_Real_PySOEM:
    def __init__(self, ifname):
        self.ifname = ifname
        self.num_dofs = 18

        # --- Load Configuration from hexapod.yaml ---
        script_dir = os.path.dirname(os.path.abspath(__file__))
        config_path = os.path.join(script_dir, "../configs/hexapod.yaml")
        print(f"Loading configuration from {config_path}")
        
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)

        # 1. Joint Mapping (policy_idx -> motor_id)
        self.policy_to_motor_id_ = config['joint2motor_idx']

        # 2. Motor Directions (按 Policy Index 顺序)
        _motor_directions_yaml = config['motor_directions']

        # 3. Motor Offsets (按 Motor ID 顺序)
        _motor_offsets_yaml = config['motor_offsets']

        # 构建最终的查询字典
        self.motor_direction_ = {}
        self.motor_offset_ = {} # 按 motor_id 索引

        for policy_idx in range(self.num_dofs):
            motor_id = self.policy_to_motor_id_[policy_idx]
            
            # Direction 是跟着 Policy Index 走的
            self.motor_direction_[motor_id] = _motor_directions_yaml[policy_idx]
            
            # Offset 也是跟着 Policy Index 走的 (用户已在 YAML 中按顺序重排)
            self.motor_offset_[motor_id] = _motor_offsets_yaml[policy_idx]

        
        # EtherCAT Address Mapping (Strictly following rl_real_el4090.cpp)
        # slave 0: motor_id [7,8,9,1,2,3]
        # slave 1: motor_id [13,17,18,16,14,15]
        # slave 2: motor_id [4,5,6,10,11,12]
        
        # 定义 C++ 中的映射表
        # 注意: 即使你是 12 DOF，如果硬件实际上是按照这个 ID 排序插在网口上的，就需要保留这个映射
        motor_id_map = [
            [7, 8, 9, 1, 2, 3],        # Slave 0
            [13, 17, 18, 16, 15, 14],  # Slave 1
            [4, 5, 6, 10, 11, 12]      # Slave 2
        ]

        self.motor_ethercat_addr_ = {}
        for slave_idx, motor_ids in enumerate(motor_id_map):
            for passage_idx, m_id in enumerate(motor_ids):
                # passage is 1-based (1~6)
                self.motor_ethercat_addr_[m_id] = {'slave': slave_idx, 'passage': passage_idx + 1}

        # Buffers
        self.motor_command_buffer = type('CommandBuffer', (), {})()
        self.motor_command_buffer.target_position = [0.0] * self.num_dofs
        self.motor_command_buffer.target_velocity = [0.0] * self.num_dofs
        self.motor_command_buffer.kp = [0.0] * self.num_dofs
        self.motor_command_buffer.kd = [0.0] * self.num_dofs
        self.motor_command_buffer.feedforward_torque = [0.0] * self.num_dofs
        
        self.motor_state_buffer = type('StateBuffer', (), {})()
        self.motor_state_buffer.position = [0.0] * self.num_dofs
        self.motor_state_buffer.velocity = [0.0] * self.num_dofs
        self.motor_state_buffer.torque = [0.0] * self.num_dofs

        # SDK Data Wrapper (initialized after PDO size is known)
        self.motorData = None
        self._msg_cls = None
        
        # EtherCAT Master
        self.master = pysoem.Master()
        self.master.open(self.ifname)
        
        # Thread Signals
        self.running = False
        self.thread = None

    def _write_slave_output(self, slave, payload: bytes):
        """Write bytes to slave output buffer in-place when possible."""
        if not hasattr(slave, 'output') or slave.output is None:
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

    def init_ethercat(self):
        print(f"Initializing EtherCAT on {self.ifname}...")
        
        if self.master.config_init() > 0:
            print(f"Found {len(self.master.slaves)} slaves.")
            
            # CRITICAL FIX from C++ Source (transmit.cpp):
            # The C++ code disables SDO Complete Access:
            # ec_slave[slave_idx + 1].CoEdetails &= ~ECT_COEDET_SDOCA (0x08);
            for slave in self.master.slaves:
                # Use internal helper if available
                if hasattr(slave, '_disable_complete_access'):
                    slave._disable_complete_access()
                elif hasattr(slave, 'coedetails'):
                    # Mirror SOEM: CoEdetails &= ~ECT_COEDET_SDOCA (0x08)
                    try:
                        slave.coedetails = int(slave.coedetails) & (~0x08)
                    except Exception:
                        print(f"Warning: Could not update coedetails for slave {getattr(slave, 'name', '?')}")
                else:
                    print(f"Warning: Could not disable Complete Access for slave {slave.name}")

            # IMPORTANT: config_map MUST be called BEFORE config_dc
            io_map_size = self.master.config_map()
            
            if io_map_size > 0:
                print(f"EtherCAT config_map successful. Total IO size: {io_map_size} bytes")
                
                # Configure Distributed Clocks (DC)
                # This aligns with C++ ec_configdc() call AFTER mapping
                self.master.config_dc()

                # Determine actual per-direction PDO sizes from pysoem buffers.
                in_lens = []
                out_lens = []
                for i, slave in enumerate(self.master.slaves):
                    in_len = len(slave.input) if getattr(slave, 'input', None) is not None else 0
                    out_len = len(slave.output) if getattr(slave, 'output', None) is not None else 0
                    in_lens.append(in_len)
                    out_lens.append(out_len)
                    print(f"Slave {i} PDO sizes: IN={in_len} bytes, OUT={out_len} bytes")

                pdo_bytes = max(in_lens + out_lens)
                self._msg_cls = make_ethercat_msg_type(pdo_bytes)
                print(f"Using EtherCAT message size: {ctypes.sizeof(self._msg_cls)} bytes")

                # Initialize SDK wrapper with the correct message type
                self.motorData = MotorData(slave_count=len(self.master.slaves), msg_cls=self._msg_cls)
                for slave_idx in range(len(self.master.slaves)):
                    tx0 = self.motorData.getTxMsg(slave_idx)
                    tx0.motor_num = 6
                    tx0.can_ide = 1
                    self.motorData.setTxMsg(slave_idx, tx0)
                    self._write_slave_output(self.master.slaves[slave_idx], bytes(tx0))

                for i, slave in enumerate(self.master.slaves):
                    print(f"Slave {i}: {slave.name} (ID: {slave.id}) State: {slave.state}")
            else:
                print("EtherCAT config_map failed.")
                return False
            
            if self.master.state_check(pysoem.SAFEOP_STATE, 50000) != pysoem.SAFEOP_STATE:
                print("Failed to reach SAFEOP state.")
                self.master.read_state()
                for slave in self.master.slaves:
                    print(f"Slave {slave.name} state: {slave.state}")
                    if slave.state != pysoem.SAFEOP_STATE:
                        print(f"SafeOp Error: {slave.state}")
                        try:
                            print(f"  AL Status Code: {hex(slave.al_status)}")
                            # pysoem 可能会有 al_status_code_to_string 工具函数，如果没有则跳过
                            if hasattr(pysoem, 'al_status_code_to_string'):
                                print(f"  AL Status Message: {pysoem.al_status_code_to_string(slave.al_status)}")
                        except Exception as e:
                            print(f"  Could not read AL Status: {e}")
                return False

            # Request OP state (mirror SOEM sequence: send one valid processdata, then write_state)
            print("Requesting OP state...")
            self.master.state = pysoem.OP_STATE
            for slave in self.master.slaves:
                try:
                    slave.state = pysoem.OP_STATE
                except Exception:
                    pass

            # send one valid process data to make outputs in slaves happy
            self.master.send_processdata()
            self.master.receive_processdata(2000)

            # request OP state for all slaves
            if hasattr(self.master, 'write_state'):
                self.master.write_state()

            # Wait for OP state
            for _ in range(40):
                self.master.send_processdata()
                self.master.receive_processdata(2000)
                self.master.state_check(pysoem.OP_STATE, 50000)

                all_op = True
                for slave in self.master.slaves:
                    if slave.state != pysoem.OP_STATE:
                        all_op = False
                        break
                if all_op:
                    print("All slaves reached OP state.")
                    return True

            print("Failed to reach OP state.")
            self.master.read_state()
            for i, slave in enumerate(self.master.slaves):
                print(f"Slave {i} {slave.name} state: {slave.state}")
                try:
                    print(f"  AL Status Code: {hex(slave.al_status)}")
                    if hasattr(pysoem, 'al_status_code_to_string'):
                        print(f"  AL Status Message: {pysoem.al_status_code_to_string(slave.al_status)}")
                except Exception:
                    pass
            return False
        else:
            print("No slaves found.")
            return False

    def pd_loop(self):
        """EtherCAT Process Data Loop (Real-time thread)"""
        print("Starting Process Data Loop...")
        self.running = True
        
        # Frequency Control
        TARGET_FREQ = 150.0  # Hz
        dt = 1.0 / TARGET_FREQ
        next_wake_time = time.perf_counter()

        try:
            while self.running:
                # 1. Send / Receive Process Data
                self.master.send_processdata()
                self.master.receive_processdata(2000)
                
                # 2. Hardware Recv: 从 IOmap 读取数据更新到 SDK buffer
                for i, slave in enumerate(self.master.slaves):
                    # 获取 Input Buffer (Rx mapped area in SOEM usually means Inputs to Master)
                    # 注意: pysoem 的 slave.input 是 bytes，我们需要将其拷贝到我们的 SDK 结构体中
                    
                    # 假设 slave.input 对应 RxPDO (Motor status)
                    # 假设 slave.output 对应 TxPDO (Motor command)
                    
                    # 这里是一个极为关键的内存映射步骤：
                    # 我们需要把 slave.input (bytes) -> MotorData.rx_msgs (ctypes)
                    if getattr(slave, 'input', None):
                        copy_size = min(len(slave.input), ctypes.sizeof(self._msg_cls))
                        ctypes.memmove(
                            ctypes.addressof(self.motorData.rx_msgs[i]),
                            bytes(slave.input),
                            copy_size,
                        )
                        
                    # 触发 SDK 解包逻辑
                    with self.motorData.motor_msg_mutexes[i]:
                        RV_can_data_repack(self.motorData.rx_msgs[i], 0x00, self.motorData.rx_motor_msgs[i], i)

                # 3. 执行应用层逻辑 (HardwareRecv -> Controller -> HardwareSend)
                self.HardwareRecv()
                
                # 这里可以插入简单的 PD 控制测试
                # e.g. self.motor_command_buffer.target_position[0] = 0.5 * math.sin(time.time())
                
                self.HardwareSend()

                # 4. Hardware Send: 从 SDK buffer 写回到 IOmap
                for i, slave in enumerate(self.master.slaves):
                    # 把 MotorData.tx_msgs (ctypes) -> slave.output (bytes)
                    tx_msg = self.motorData.getTxMsg(i)  # thread-safe copy
                    self._write_slave_output(slave, bytes(tx_msg))

                # Precision Loop Pacing (Drift compensation)
                now = time.perf_counter()
                sleep_time = next_wake_time - now
                if sleep_time > 0:
                    time.sleep(sleep_time)
                else:
                    # Overrun detection (optional logging)
                    pass
                
                # Schedule next iteration
                next_wake_time += dt

        except Exception as e:
            print(f"Exception in control loop: {e}")
        finally:
            self.running = False

    def start(self):
        if self.init_ethercat():
            self.thread = threading.Thread(target=self.pd_loop)
            self.thread.start()
            return True
        return False

    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join()
        self.master.close()

    # Reuse the ported logic derived earlier
    def HardwareSend(self):
        # ... logic as defined in your previous script ...
        for policy_idx in range(self.num_dofs):
            motor_id = self.policy_to_motor_id_[policy_idx]
            # Safety check
            if motor_id not in self.motor_ethercat_addr_: continue
            
            addr = self.motor_ethercat_addr_[motor_id]
            slave_idx = addr['slave']
            
            # 只有当 slave 存在时才操作
            if slave_idx >= len(self.master.slaves): continue

            urdf_angle = self.motor_command_buffer.target_position[policy_idx]
            motor_pos = self.motor_direction_[motor_id] * urdf_angle + self.motor_offset_[motor_id]

            tx_msg = self.motorData.getTxMsg(slave_idx)
            send_motor_ctrl_cmd(
                tx_msg,
                addr['passage'],
                motor_id,
                self.motor_command_buffer.kp[policy_idx],
                self.motor_command_buffer.kd[policy_idx],
                motor_pos,
                self.motor_command_buffer.target_velocity[policy_idx],
                self.motor_command_buffer.feedforward_torque[policy_idx]
            )
            self.motorData.setTxMsg(slave_idx, tx_msg)

    def HardwareRecv(self):
        # ... logic as defined in your previous script ...
        for policy_idx in range(self.num_dofs):
            motor_id = self.policy_to_motor_id_[policy_idx]
            if motor_id not in self.motor_ethercat_addr_: continue
            
            addr = self.motor_ethercat_addr_[motor_id]
            slave_idx = addr['slave']
            
            if slave_idx >= len(self.master.slaves): continue

            motor_msg = self.motorData.getRxMotorMsg(slave_idx, addr['passage'])

            if motor_msg.motor_id == motor_id:
                motor_pos = motor_msg.angle_actual_rad
                urdf_angle = self.motor_direction_[motor_id] * (motor_pos - self.motor_offset_[motor_id])

                self.motor_state_buffer.position[policy_idx] = urdf_angle
                self.motor_state_buffer.velocity[policy_idx] = self.motor_direction_[motor_id] * motor_msg.speed_actual_rad
                self.motor_state_buffer.torque[policy_idx] = motor_msg.current_actual_float

# ==================== Main ====================

if __name__ == "__main__":
    if len(sys.argv) > 1:
        IFNAME = sys.argv[1]
    
    print(f"Launching RL_Real on interface {IFNAME}")
    
    try:
        robot = RL_Real_PySOEM(IFNAME)
        
        # Test Sequence: Enable Motors with zero-torque mode first
        print("Setting zero torque command...")
        for i in range(robot.num_dofs):
            robot.motor_command_buffer.kp[i] = 0.0
            robot.motor_command_buffer.kd[i] = 1.0 # slight damping
            robot.motor_command_buffer.target_position[i] = 0.0

        if robot.start():
            print("Robot Loop Running. Press Ctrl+C to stop.")
            
            # Simple interaction demo
            start_time = time.time()
            try:
                while True:
                    # 小 KD + 小 Position 用作测试
                    # 15 对应4号电机（根据 YAML 映射），你可以修改为其他 ID 来测试不同的电机
                    robot.motor_command_buffer.kp[15] = 5.0 
                    robot.motor_command_buffer.kd[15] = 0.5
                    robot.motor_command_buffer.target_position[15] = 0

                    current_pos = robot.motor_state_buffer.position[15]
                    current_vel = robot.motor_state_buffer.velocity[15]
                    current_tau = robot.motor_state_buffer.torque[15]
                    
                    # 打印看看
                    print(f"  Real Pos: {current_pos:.3f} | Real Vel: {current_vel:.3f} | Real Tau: {current_tau:.3f}")
                    
                    time.sleep(0.02)
                    
            except KeyboardInterrupt:
                print("Stopping...")
        else:
            print("Failed to start.")

    except Exception as e:
        print(f"Error: {e}")
        print("Ensure you have root privileges (sudo) to access raw sockets for EtherCAT.")
    finally:
        if 'robot' in locals():
            robot.stop()
