import ctypes
import threading

# Constants defined in motor_control.c
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
