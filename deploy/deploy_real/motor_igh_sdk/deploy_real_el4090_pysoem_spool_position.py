import sys
import time
import math

try:
    # Script-style (recommended when running this file directly)
    from deploy_real_el4090_pysoem import RL_Real_PySOEM
except ImportError:
    from .deploy_real_el4090_pysoem import RL_Real_PySOEM

try:
    # Script-style (recommended when running this file directly)
    from el4090_motor_sdk import set_motor_position
except ImportError:
    from .el4090_motor_sdk import set_motor_position


class RL_Real_PySOEM_WithSpoolPosition(RL_Real_PySOEM):
    """Extend `RL_Real_PySOEM` (18-joint PD) with ONE extra position-mode motor.

    Requirements from user:
    - Keep the original 18-joint PD command/state interface unchanged.
    - Add motor_id=19 on a new 4th slave board: slave_idx=3, passage=1.
    - Provide a position-command buffer in the same style as the standalone position wrapper.
    - Use the same background pd_loop thread (do NOT create a second EtherCAT master).

    Notes:
    - This class assumes the EtherCAT chain now has 4 slaves.
    - If fewer slaves are detected, spool control is disabled with a warning.
    """

    def __init__(
        self,
        ifname: str,
        *,
        spool_motor_id: int = 19,
        spool_slave_idx: int = 3,
        spool_passage: int = 1,
        spool_direction: float = 1.0,
    ):
        super().__init__(ifname)

        self.spool_motor_id = int(spool_motor_id)
        self.spool_slave_idx = int(spool_slave_idx)
        self.spool_passage = int(spool_passage)
        self.spool_direction = float(spool_direction)

        # Position-mode command/state buffers (match RL_Real_Position_PySOEM style)
        self.spool_command_buffer = type("SpoolCommandBuffer", (), {})()
        self.spool_command_buffer.target_position_deg = [0.0]
        self.spool_command_buffer.speed_limit_01rpm = [50]
        self.spool_command_buffer.current_limit_01a = [500]
        self.spool_command_buffer.ack_status = [2]

        self.spool_state_buffer = type("SpoolStateBuffer", (), {})()
        self.spool_state_buffer.position = [0.0]
        self.spool_state_buffer.velocity = [0.0]
        self.spool_state_buffer.torque = [0.0]

        self._spool_enabled = True

    def init_ethercat(self):
        ok = super().init_ethercat()
        if not ok:
            return False

        slave_count = len(self.master.slaves)
        if slave_count <= self.spool_slave_idx:
            print(
                f"[WARNING] Spool disabled: need slave_idx={self.spool_slave_idx} (4th slave) but only {slave_count} slaves detected."
            )
            self._spool_enabled = False
        return True

    def HardwareSend(self):
        # 1) Send original 18-joint PD commands
        super().HardwareSend()

        # 2) Send spool position command on slave 3 / passage 1
        if not self._spool_enabled:
            return

        if self.spool_slave_idx >= len(self.master.slaves):
            return

        target_rad = math.radians(float(self.spool_command_buffer.target_position_deg[0]))
        motor_pos_deg = math.degrees(self.spool_direction * target_rad)
        spd_01rpm = int(self.spool_command_buffer.speed_limit_01rpm[0])
        cur_01a = int(self.spool_command_buffer.current_limit_01a[0])
        ack = int(self.spool_command_buffer.ack_status[0])

        tx_msg = self.motorData.getTxMsg(self.spool_slave_idx)
        set_motor_position(
            tx_msg,
            int(self.spool_passage),
            int(self.spool_motor_id),
            pos_deg=float(motor_pos_deg),
            spd_01rpm=int(spd_01rpm),
            cur_01a=int(cur_01a),
            ack_status=int(ack),
        )
        self.motorData.setTxMsg(self.spool_slave_idx, tx_msg)

    def HardwareRecv(self):
        # 1) Receive original 18-joint states if they are returned.
        super().HardwareRecv()

        # 2) Receive spool motor state.
        if not self._spool_enabled:
            return
        if self.spool_slave_idx >= len(self.master.slaves):
            return

        motor_msg = self.motorData.getRxMotorMsg(self.spool_slave_idx, int(self.spool_passage))
        if motor_msg.motor_id != int(self.spool_motor_id):
            return

        ack = int(self.spool_command_buffer.ack_status[0])
        if ack == 2:
            motor_pos = math.radians(float(motor_msg.angle_actual_float))
        else:
            motor_pos = float(motor_msg.angle_actual_rad)

        self.spool_state_buffer.position[0] = float(self.spool_direction * motor_pos)

        if ack == 3:
            motor_vel = float(motor_msg.speed_actual_float) * 2.0 * math.pi / 60.0
        else:
            motor_vel = float(motor_msg.speed_actual_rad)

        self.spool_state_buffer.velocity[0] = float(self.spool_direction * motor_vel)
        self.spool_state_buffer.torque[0] = float(motor_msg.current_actual_float)


if __name__ == "__main__":
    # Usage:
    #   python deploy_real_el4090_pysoem_spool_position.py <ifname> [target_deg] [speed_0.1rpm] [current_0.1A]
    ifname = sys.argv[1] if len(sys.argv) > 1 else "enp86s0"
    target_deg = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
    speed_limit_01rpm = int(sys.argv[3]) if len(sys.argv) > 3 else 50
    current_limit_01a = int(sys.argv[4]) if len(sys.argv) > 4 else 500

    print(
        f"Launching RL_Real_PySOEM_WithSpoolPosition on {ifname} "
        f"(motor_id=19, slave_idx=3, passage=1), target_deg={target_deg}"
    )

    robot = None
    try:
        robot = RL_Real_PySOEM_WithSpoolPosition(ifname)

        # Keep joints in damping by default
        for i in range(robot.num_dofs):
            robot.motor_command_buffer.kp[i] = 0.0
            robot.motor_command_buffer.kd[i] = 1.0
            robot.motor_command_buffer.target_position[i] = 0.0
            robot.motor_command_buffer.target_velocity[i] = 0.0
            robot.motor_command_buffer.feedforward_torque[i] = 0.0

        # Spool position command
        robot.spool_command_buffer.target_position_deg[0] = target_deg
        robot.spool_command_buffer.speed_limit_01rpm[0] = speed_limit_01rpm
        robot.spool_command_buffer.current_limit_01a[0] = current_limit_01a
        robot.spool_command_buffer.ack_status[0] = 2

        if not robot.start():
            print("Failed to start EtherCAT.")
            sys.exit(1)

        print("Loop running. Ctrl+C to stop.")
        t0 = time.time()
        while True:
            time.sleep(0.2)
            t = time.time() - t0
            v = robot.spool_state_buffer.velocity[0]
            p = robot.spool_state_buffer.position[0]
            c = robot.spool_state_buffer.torque[0]
            if int(t * 5) % 5 == 0:
                print(
                    f"t={t:6.2f}s cmd_pos(deg)={target_deg:8.3f} "
                    f"pos(rad)={p:8.3f} vel(rad/s)={v:8.3f} cur={c:8.3f}"
                )

    except KeyboardInterrupt:
        print("Stopping...")
    finally:
        if robot is not None:
            robot.stop()
