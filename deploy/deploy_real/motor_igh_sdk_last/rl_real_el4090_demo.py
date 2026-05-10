import time
import random
from el4090_motor_sdk import *

# ==================== Demo RL_Real Class ====================

class RL_Real:
    def __init__(self, num_dofs=12):
        self.num_dofs = num_dofs
        
        # --- Mocking Configuration ---
        # 假设 1-12 号电机，Policy 0-11
        self.policy_to_motor_id_ = [i+1 for i in range(num_dofs)] 
        
        # Directions: +1 for all
        self.motor_direction_ = {i: 1.0 for i in range(1, num_dofs+20)} 
        
        # Offsets: 0.0 for all
        self.motor_offset_ = {i: 0.0 for i in range(1, num_dofs+20)}
        
        # EtherCAT Address Mapping (Simulate a setup)
        # Slave 0: Motors 1-6
        # Slave 1: Motors 7-12
        self.motor_ethercat_addr_ = {}
        for m_id in range(1, 7):
            self.motor_ethercat_addr_[m_id] = {'slave': 0, 'passage': m_id}
        for m_id in range(7, 13):
            self.motor_ethercat_addr_[m_id] = {'slave': 1, 'passage': m_id - 6}

        # Buffers
        class CommandBuffer:
            def __init__(self, n):
                self.target_position = [0.0] * n
                self.target_velocity = [0.0] * n
                self.kp = [0.0] * n
                self.kd = [0.0] * n
                self.feedforward_torque = [0.0] * n
        
        class StateBuffer:
            def __init__(self, n):
                self.position = [0.0] * n
                self.velocity = [0.0] * n
                self.torque = [0.0] * n

        self.motor_command_buffer = CommandBuffer(self.num_dofs)
        self.motor_state_buffer = StateBuffer(self.num_dofs)
        
        # Initialize SDK Data Wrapper
        self.motorData = MotorData(slave_count=2) # 2 slaves for 12 motors

    # ------------------ Ported Function: HardwareSend ------------------
    def HardwareSend(self):
        # 发送流程: Policy Index → Motor ID → EtherCAT
        for policy_idx in range(self.num_dofs):
            motor_id = self.policy_to_motor_id_[policy_idx]
            addr = self.motor_ethercat_addr_[motor_id]

            # 坐标变换: URDF angle → Motor physical position
            # motor_pos = direction * urdf_angle + offset
            urdf_angle = self.motor_command_buffer.target_position[policy_idx]
            motor_pos = self.motor_direction_[motor_id] * urdf_angle + self.motor_offset_[motor_id]

            # 发送电机命令
            # Python 里 getTxMsg 返回的是对象引用，可以直接修改
            tx_msg = self.motorData.getTxMsg(addr['slave'])
            
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
            
            # 在 Python 中因为是引用传递，setTxMsg 其实是不必要的，
            # 但为了保持和 C++ 代码逻辑一致，我们还是写在这里
            self.motorData.setTxMsg(addr['slave'], tx_msg)

    # ------------------ Ported Function: HardwareRecv ------------------
    def HardwareRecv(self):
        # 接收流程: EtherCAT → Motor ID → Policy Index
        # (Using instance variable for counter simulation)
        if not hasattr(self, '_recv_print_counter'): self._recv_print_counter = 0
        should_print = (self._recv_print_counter % 250 == 0)
        self._recv_print_counter += 1

        # 先收集所有数据
        for policy_idx in range(self.num_dofs):
            motor_id = self.policy_to_motor_id_[policy_idx]
            addr = self.motor_ethercat_addr_[motor_id]

            # 读取电机状态
            motor_msg = self.motorData.getRxMotorMsg(addr['slave'], addr['passage'])

            # if should_print:
            #     print(f"DEBUG: Motor {motor_msg.motor_id} Raw Pos: {motor_msg.angle_actual_rad}")

            if motor_msg.motor_id == motor_id:
                # 坐标变换: Motor physical position → URDF angle
                # urdf_angle = direction * (motor_pos - offset)
                motor_pos = motor_msg.angle_actual_rad
                urdf_angle = self.motor_direction_[motor_id] * (motor_pos - self.motor_offset_[motor_id])

                self.motor_state_buffer.position[policy_idx] = urdf_angle
                self.motor_state_buffer.velocity[policy_idx] = self.motor_direction_[motor_id] * motor_msg.speed_actual_rad
                self.motor_state_buffer.torque[policy_idx] = motor_msg.current_actual_float

        self.UpdateVelocityEstimation()

    def UpdateVelocityEstimation(self):
        pass # Placeholder

# ==================== Verification Demo ====================

def run_verification():
    print("Initializing Fake Robot...")
    robot = RL_Real(num_dofs=2) # Test with 2 motors for simplicity

    # 1. Setup Command (Simulation -> Hardware)
    print("\n--- Test 1: Sending Command (Sim -> Real) ---")
    target_pos = 0 
    robot.motor_command_buffer.target_position[0] = target_pos
    robot.motor_command_buffer.kp[0] = 20.0
    robot.motor_command_buffer.kd[0] = 5.0
    
    # 2. Execute HardwareSend
    robot.HardwareSend()
    
    # 3. Verify Packet Content (Hardware Side)
    # Check the Tx buffer of Slave 0, Passage 1 (Motor ID 1)
    slave_0_tx = robot.motorData.getTxMsg(0)
    motor_1_data = slave_0_tx.motor[0] # passage 1 is index 0
    
    print(f"Command Sent to Motor {motor_1_data.id}:")
    print(f"  Raw Data Bytes: {[hex(x) for x in motor_1_data.data]}")
    
    # Manually unpack pos to verify (Bytes 3 and 4)
    pos_int_extracted = (motor_1_data.data[3] << 8) | motor_1_data.data[4]
    pos_reconstructed = uint_to_float(pos_int_extracted, POS_MIN, POS_MAX, 16)
    print(f"  Reconstructed Position: {pos_reconstructed:.4f} (Expected: {target_pos:.4f})")
    
    if abs(pos_reconstructed - target_pos) < 0.01:
        print("  SUCCESS: Send Logic Packet Packing Verified.")
    else:
        print("  FAILURE: Send Logic Packet Mismatch.")

    # 4. Simulate Hardware Response (Hardware -> Simulation)
    print("\n--- Test 2: Receiving Data (Real -> Sim) ---")
    
    # Create a fake response packet simulating Motor 1 reporting it is at 1.0 rad
    # We use the same pack logic but "reverse" manually to create the raw bytes
    # Frame format 1: 0x20(ack=1) | pos_hi | pos_lo | spd_hi | ...
    
    actual_pos_hardware = 1.0
    pos_int_hw = float_to_uint(actual_pos_hardware, POS_MIN, POS_MAX, 16)
    
    # Construct raw CAN bytes for "Response Frame 1"
    raw_rx_bytes = [0] * 8
    raw_rx_bytes[0] = (1 << 5) | (0x00) # Ack Status = 1 (bits 5-7), Error = 0
    raw_rx_bytes[1] = (pos_int_hw >> 8) & 0xFF
    raw_rx_bytes[2] = pos_int_hw & 0xFF
    # ... ignoring speed/current for brevity, fill with zeros
    
    rx_packet = EtherCAT_Msg()
    rx_packet.motor[0].id = 1 # ID must match
    rx_packet.motor[0].dlc = 8
    for i in range(8):
        rx_packet.motor[0].data[i] = raw_rx_bytes[i]
        
    # Inject into robot's RX buffer
    robot.motorData.setRxMsgRaw(0, rx_packet)
    
    # 5. Execute HardwareRecv
    robot.HardwareRecv()
    
    # 6. Verify State Buffer
    received_pos = robot.motor_state_buffer.position[0]
    print(f"State Buffer Position: {received_pos:.4f} (Expected: {actual_pos_hardware:.4f})")
    
    if abs(received_pos - actual_pos_hardware) < 0.01:
        print("  SUCCESS: Recv Logic Unpacking Verified.")
    else:
        print("  FAILURE: Recv Logic Mismatch.")

if __name__ == "__main__":
    run_verification()