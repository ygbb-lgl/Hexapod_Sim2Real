import ctypes
import threading
from typing import List

# Constants defined in motor_control.c
KP_MIN = 0.0
KP_MAX = 500.0
KD_MIN = 0.0
KD_MAX = 5.0
POS_MIN = -12.5
POS_MAX = 12.5
SPD_MIN = -18.0
SPD_MAX = 18.0
T_MIN = -85.0
T_MAX = 85.0
I_MIN = -60.0
I_MAX = 60.0

# Servo mode ranges (from official C SDK comments)
SERVO_SPD_RPM_MIN = -18000.0
SERVO_SPD_RPM_MAX = 18000.0
SERVO_POS_SPD_01RPM_MIN = 0
SERVO_POS_SPD_01RPM_MAX = 18000
SERVO_CUR_01A_MIN = 0
SERVO_CUR_01A_MAX = 3000

# ==================== Data Structures ====================

# Matches official C SDK `config.h`:
#
# typedef struct {
#   uint32_t id;
#   uint8_t  rtr;
#   uint8_t  dlc;
#   uint8_t  data[8];
# } Motor_Msg;                // 14 bytes
#
# typedef struct {
#   uint8_t motor_num;
#   uint8_t can_ide;
#   Motor_Msg motor[6];
# } EtherCAT_Msg;             // 86 bytes
class Motor_Msg(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("id", ctypes.c_uint32),
        ("rtr", ctypes.c_uint8),
        ("dlc", ctypes.c_uint8),
        ("data", ctypes.c_uint8 * 8),
    ]


class EtherCAT_Msg_Base(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("motor_num", ctypes.c_uint8),
        ("can_ide", ctypes.c_uint8),
        ("motor", Motor_Msg * 6),
    ]


def make_ethercat_msg_type(pdo_bytes: int):
    """Create a ctypes EtherCAT message struct matching the actual PDO size.

    Some EtherCAT devices expose 89 bytes (86 + padding). Padding is assumed to
    be at the end, which is safe for our usage.
    """
    base_size = ctypes.sizeof(EtherCAT_Msg_Base)
    if pdo_bytes <= 0:
        return EtherCAT_Msg_Base
    if pdo_bytes < base_size:
        raise ValueError(f"PDO size {pdo_bytes} is smaller than base {base_size}")
    if pdo_bytes == base_size:
        return EtherCAT_Msg_Base

    pad = pdo_bytes - base_size

    class EtherCAT_Msg_Padded(ctypes.Structure):
        _pack_ = 1
        _fields_ = [
            ("motor_num", ctypes.c_uint8),
            ("can_ide", ctypes.c_uint8),
            ("motor", Motor_Msg * 6),
            ("_padding", ctypes.c_uint8 * pad),
        ]

    return EtherCAT_Msg_Padded

class OD_Motor_Msg(ctypes.Structure):
    _fields_ = [
        ("angle_actual_int", ctypes.c_uint16),
        ("angle_desired_int", ctypes.c_uint16),
        ("speed_actual_int", ctypes.c_int16),
        ("speed_desired_int", ctypes.c_int16),
        ("current_actual_int", ctypes.c_int16),
        ("current_desired_int", ctypes.c_int16),
        ("speed_actual_rad", ctypes.c_float),
        ("speed_desired_rad", ctypes.c_float),
        ("angle_actual_rad", ctypes.c_float),
        ("angle_desired_rad", ctypes.c_float),
        ("motor_id", ctypes.c_uint16),
        ("temperature", ctypes.c_uint8),
        ("error", ctypes.c_uint8),
        ("angle_actual_float", ctypes.c_float),
        ("speed_actual_float", ctypes.c_float),
        ("current_actual_float", ctypes.c_float),
        ("angle_desired_float", ctypes.c_float),
        ("speed_desired_float", ctypes.c_float),
        ("current_desired_float", ctypes.c_float),
        ("power", ctypes.c_float),
        ("acceleration", ctypes.c_uint16),
        ("linkage_KP", ctypes.c_uint16),
        ("speed_KI", ctypes.c_uint16),
        ("feedback_KP", ctypes.c_uint16),
        ("feedback_KD", ctypes.c_uint16),
    ]

# ==================== Helper Functions ====================

def float_to_uint(x, x_min, x_max, bits):
    """Converts a float to an unsigned int, given range and number of bits"""
    span = x_max - x_min
    offset = x_min
    if span == 0: return 0
    # Create the scaling factor for the given number of bits
    # ((1 << bits) - 1) is simply 2^bits - 1
    return int((x - offset) * ((1 << bits) - 1) / span)

def uint_to_float(x_int, x_min, x_max, bits):
    """Converts unsigned int to float, given range and number of bits"""
    span = x_max - x_min
    offset = x_min
    return float(x_int) * span / ((1 << bits) - 1) + offset

def clamp(val, min_val, max_val):
    return max(min_val, min(val, max_val))


def _float_to_u8_le(x: float) -> List[int]:
    """Pack float32 into little-endian 4 bytes (b0..b3).

    Note: official C uses a union and then accesses buf[0..3].
    """
    b = ctypes.c_float(float(x))
    raw = bytes(b)
    return [raw[0], raw[1], raw[2], raw[3]]

# ==================== Core Logic ====================

def send_motor_ctrl_cmd(msg, passage, motor_id, kp, kd, pos, spd, tor):
    """
    Simulates: void send_motor_ctrl_cmd(EtherCAT_Msg* tx_msg, uint8_t passage, uint16_t motor_id, ...)
    """
    # Point to the specific motor slot in the EtherCAT message
    # passage is 1-based (1-6)
    idx = passage - 1
    motor_slot = msg.motor[idx]

    # Set Header (matches C)
    msg.can_ide = 1
    motor_slot.rtr = 0
    motor_slot.id = int(motor_id)
    motor_slot.dlc = 8

    # Limit inputs
    kp = clamp(kp, KP_MIN, KP_MAX)
    kd = clamp(kd, KD_MIN, KD_MAX)
    pos = clamp(pos, POS_MIN, POS_MAX)
    spd = clamp(spd, SPD_MIN, SPD_MAX)
    tor = clamp(tor, T_MIN, T_MAX)

    # Convert to Integers
    kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12)
    kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 9)
    pos_int = float_to_uint(pos, POS_MIN, POS_MAX, 16)
    spd_int = float_to_uint(spd, SPD_MIN, SPD_MAX, 12)
    tor_int = float_to_uint(tor, T_MIN, T_MAX, 12)

    # Pack Data (Bit manipulation)
    motor_slot.data[0] = (kp_int >> 7) & 0xFF
    motor_slot.data[1] = ((kp_int & 0x7F) << 1) | ((kd_int & 0x100) >> 8)
    motor_slot.data[2] = kd_int & 0xFF
    motor_slot.data[3] = (pos_int >> 8) & 0xFF
    motor_slot.data[4] = pos_int & 0xFF
    motor_slot.data[5] = (spd_int >> 4) & 0xFF
    motor_slot.data[6] = ((spd_int & 0x0F) << 4) | (tor_int >> 8)
    motor_slot.data[7] = tor_int & 0xFF


def set_motor_speed(msg, passage, motor_id, spd_rpm, cur_01a, ack_status=1):
        """Servo speed control (official C: set_motor_speed).

        - spd_rpm: desired speed in rpm, range [-18000, 18000]
        - cur_01a: current threshold in 0.1A units, range [0, 3000]
        - ack_status: 0=no ack, 1~3=response frame type

        The official C sets:
            can_ide=0, dlc=7,
            data[0]=0x40|ack_status,
            data[1..4]=float bytes (buf[3],buf[2],buf[1],buf[0])
            data[5..6]=cur high/low
        """
        idx = passage - 1
        motor_slot = msg.motor[idx]

        if ack_status < 0 or ack_status > 3:
                raise ValueError(f"ack_status must be 0..3, got {ack_status}")

        spd_rpm = float(clamp(spd_rpm, SERVO_SPD_RPM_MIN, SERVO_SPD_RPM_MAX))
        cur_01a = int(clamp(int(cur_01a), SERVO_CUR_01A_MIN, SERVO_CUR_01A_MAX))

        msg.can_ide = 0
        motor_slot.rtr = 0
        motor_slot.id = int(motor_id)
        motor_slot.dlc = 7

        b0, b1, b2, b3 = _float_to_u8_le(spd_rpm)
        motor_slot.data[0] = 0x40 | int(ack_status)
        # C uses buf[3],buf[2],buf[1],buf[0]
        motor_slot.data[1] = b3
        motor_slot.data[2] = b2
        motor_slot.data[3] = b1
        motor_slot.data[4] = b0
        motor_slot.data[5] = (cur_01a >> 8) & 0xFF
        motor_slot.data[6] = cur_01a & 0xFF


def set_motor_position(msg, passage, motor_id, pos_deg, spd_01rpm=50, cur_01a=500, ack_status=2):
    """Servo position control (official C: set_motor_position).

    - pos_deg: desired motor position in degrees
    - spd_01rpm: speed limit in 0.1 rpm units, range [0, 18000]
    - cur_01a: current threshold in 0.1A units, range [0, 3000]
    - ack_status: 0=no ack, 1~3=response frame type
    """
    idx = passage - 1
    motor_slot = msg.motor[idx]

    if ack_status < 0 or ack_status > 3:
        raise ValueError(f"ack_status must be 0..3, got {ack_status}")

    pos_deg = float(pos_deg)
    spd_01rpm = int(clamp(int(spd_01rpm), SERVO_POS_SPD_01RPM_MIN, SERVO_POS_SPD_01RPM_MAX))
    cur_01a = int(clamp(int(cur_01a), SERVO_CUR_01A_MIN, SERVO_CUR_01A_MAX))

    msg.can_ide = 0
    motor_slot.rtr = 0
    motor_slot.id = int(motor_id)
    motor_slot.dlc = 8

    b0, b1, b2, b3 = _float_to_u8_le(pos_deg)
    motor_slot.data[0] = (0x20 | (b3 >> 3)) & 0xFF
    motor_slot.data[1] = ((b3 << 5) | (b2 >> 3)) & 0xFF
    motor_slot.data[2] = ((b2 << 5) | (b1 >> 3)) & 0xFF
    motor_slot.data[3] = ((b1 << 5) | (b0 >> 3)) & 0xFF
    motor_slot.data[4] = ((b0 << 5) | (spd_01rpm >> 10)) & 0xFF
    motor_slot.data[5] = ((spd_01rpm & 0x3FC) >> 2) & 0xFF
    motor_slot.data[6] = (((spd_01rpm & 0x03) << 6) | (cur_01a >> 6)) & 0xFF
    motor_slot.data[7] = (((cur_01a & 0x3F) << 2) | int(ack_status)) & 0xFF


def set_motor_cur_tor(msg, passage, motor_id, cur_tor_001, ctrl_status=1, ack_status=1):
    """Servo current/torque/brake control (official C: set_motor_cur_tor).

    Args:
        msg: EtherCAT_Msg
        passage: 1..6
        motor_id: motor CAN id
        cur_tor_001: desired current/torque in 0.01(A or Nm) units (int16).
            - torque mode (ctrl_status=1): clamp [-3000, 3000] => [-30.00, 30.00]
            - current mode (ctrl_status=0): clamp [-2000, 2000] => [-20.00, 20.00]
        ctrl_status:
            0=current control
            1=torque control
            2=variable damping brake (full brake)
            3=energy consumption brake
            4=regenerative brake
        ack_status: 0..3
    """
    idx = passage - 1
    motor_slot = msg.motor[idx]

    ctrl_status = int(ctrl_status)
    ack_status = int(ack_status)
    if ack_status < 0 or ack_status > 3:
        raise ValueError(f"ack_status must be 0..3, got {ack_status}")
    if ctrl_status < 0 or ctrl_status > 7:
        raise ValueError(f"ctrl_status must be 0..7, got {ctrl_status}")

    cur_tor_001 = int(cur_tor_001)
    if ctrl_status != 0:
        cur_tor_001 = int(clamp(cur_tor_001, -3000, 3000))
    else:
        cur_tor_001 = int(clamp(cur_tor_001, -2000, 2000))

    msg.can_ide = 0
    motor_slot.rtr = 0
    motor_slot.id = int(motor_id)
    motor_slot.dlc = 3

    motor_slot.data[0] = 0x60 | ((ctrl_status & 0x07) << 2) | (ack_status & 0x03)
    motor_slot.data[1] = (cur_tor_001 >> 8) & 0xFF
    motor_slot.data[2] = cur_tor_001 & 0xFF


def set_motor_torque(msg, passage, motor_id, torque_nm, ack_status=1):
    """Convenience wrapper: torque control with float torque in Nm.

    Official scaling: 0.01 Nm per LSB (ratio 100:1).
    """
    cur_tor_001 = int(round(float(torque_nm) * 100.0))
    set_motor_cur_tor(msg, passage, motor_id, cur_tor_001, ctrl_status=1, ack_status=ack_status)

def _u8_to_float_le(b0: int, b1: int, b2: int, b3: int) -> float:
    return ctypes.c_float.from_buffer_copy(bytes([b0, b1, b2, b3])).value


def handle_response_mode(motor_data, motor_msg, ack_status):
    motor_msg.motor_id = int(motor_data.id)
    motor_msg.error = int(motor_data.data[0] & 0x1F)

    if ack_status == 1:  # response frame 1
        pos_int = (motor_data.data[1] << 8) | motor_data.data[2]
        spd_int = (motor_data.data[3] << 4) | ((motor_data.data[4] & 0xF0) >> 4)
        cur_int = ((motor_data.data[4] & 0x0F) << 8) | motor_data.data[5]

        motor_msg.angle_actual_rad = uint_to_float(pos_int, POS_MIN, POS_MAX, 16)
        motor_msg.speed_actual_rad = uint_to_float(spd_int, SPD_MIN, SPD_MAX, 12)
        motor_msg.current_actual_float = uint_to_float(cur_int, I_MIN, I_MAX, 12)
        motor_msg.temperature = int((motor_data.data[6] - 50) / 2)

    elif ack_status == 2:  # response frame 2
        motor_msg.angle_actual_float = _u8_to_float_le(
            int(motor_data.data[4]),
            int(motor_data.data[3]),
            int(motor_data.data[2]),
            int(motor_data.data[1]),
        )
        motor_msg.current_actual_int = (motor_data.data[5] << 8) | motor_data.data[6]
        motor_msg.temperature = int((motor_data.data[7] - 50) / 2)
        motor_msg.current_actual_float = float(motor_msg.current_actual_int) / 100.0

    elif ack_status == 3:  # response frame 3
        motor_msg.speed_actual_float = _u8_to_float_le(
            int(motor_data.data[4]),
            int(motor_data.data[3]),
            int(motor_data.data[2]),
            int(motor_data.data[1]),
        )
        motor_msg.current_actual_int = (motor_data.data[5] << 8) | motor_data.data[6]
        motor_msg.temperature = int((motor_data.data[7] - 50) / 2)
        motor_msg.current_actual_float = float(motor_msg.current_actual_int) / 100.0


def RV_can_data_repack(rx_msg, comm_mode, motor_msg_array, slave_idx, print_debug=False):
    """
    Simulates: void RV_can_data_repack(...)
    Parses received EtherCAT bytes back into physical data.
    """
    for i in range(6):
        motor_data = rx_msg.motor[i]
        if int(motor_data.dlc) == 0:
            continue
        motor_msg = motor_msg_array[i]

        if int(motor_data.id) == 0x7FF:
            # Configuration / special message. We don't rely on it for control loop.
            continue

        if comm_mode == 0x00:
            ack_status = int(motor_data.data[0] >> 5)
            handle_response_mode(motor_data, motor_msg, ack_status)
        elif comm_mode == 0x01:
            # automatic mode is not used in current deploy script
            continue

# ==================== Thread-Safe Data Manager ====================

class MotorData:
    def __init__(self, slave_count=3, msg_cls=EtherCAT_Msg_Base):
        self.msg_cls = msg_cls
        self.tx_msgs = [self.msg_cls() for _ in range(slave_count)]
        self.rx_msgs = [self.msg_cls() for _ in range(slave_count)]
        
        # 2D array: [slave][motor_passage]
        self.rx_motor_msgs = [[OD_Motor_Msg() for _ in range(6)] for _ in range(slave_count)]
        
        # Mutexes for thread safety (mimicking std::shared_mutex behavior with RLock)
        # One lock per slave to reduce contention
        self.tx_mutexes = [threading.RLock() for _ in range(slave_count)]
        self.rx_mutexes = [threading.RLock() for _ in range(slave_count)]
        self.motor_msg_mutexes = [threading.RLock() for _ in range(slave_count)]

    def getTxMsg(self, slave_idx):
        """
        Mimics C++: const EtherCAT_Msg& getTxMsg(int slave) const
        Returns a COPY of the message to work on locally (Thread-Safe Read)
        """
        with self.tx_mutexes[slave_idx]:
            # Create a shallow copy is NOT enough for ctypes Structure with arrays,
            # we need a new instance with same content.
            msg_copy = self.msg_cls()
            ctypes.memmove(
                ctypes.addressof(msg_copy),
                ctypes.addressof(self.tx_msgs[slave_idx]),
                ctypes.sizeof(self.msg_cls),
            )
            return msg_copy

    def setTxMsg(self, slave_idx, msg):
        """
        Mimics C++: void setTxMsg(int slave, const EtherCAT_Msg& msg)
        Writes the local copy back to shared memory (Thread-Safe Write)
        """
        with self.tx_mutexes[slave_idx]:
            # Overwrite the internal shared memory with the new message content
            ctypes.memmove(
                ctypes.addressof(self.tx_msgs[slave_idx]),
                ctypes.addressof(msg),
                ctypes.sizeof(self.msg_cls),
            )

    def getRxMotorMsg(self, slave_idx, passage):
        """
        Mimics C++: OD_Motor_Msg getRxMotorMsg(int slave, int passage)
        Returns a copy of the motor state
        """
        with self.motor_msg_mutexes[slave_idx]:
            # Return a COPY of the structure
            src = self.rx_motor_msgs[slave_idx][passage - 1]
            dest = OD_Motor_Msg()
            ctypes.memmove(ctypes.addressof(dest), ctypes.addressof(src), ctypes.sizeof(OD_Motor_Msg))
            return dest

    def setRxMsgRaw(self, slave_idx, msg):
        """
        Simulate receiving raw data from hardware (Thread-Safe Write)
        This would be called by the EtherCAT Sim/Real receiving thread
        """
        with self.rx_mutexes[slave_idx]:
            ctypes.memmove(
                ctypes.addressof(self.rx_msgs[slave_idx]),
                ctypes.addressof(msg),
                ctypes.sizeof(self.msg_cls),
            )
        
        # Immediately unpack (in a real scenario this might be triggered separately)
        with self.motor_msg_mutexes[slave_idx]:
            RV_can_data_repack(self.rx_msgs[slave_idx], 0x00, self.rx_motor_msgs[slave_idx], slave_idx)
