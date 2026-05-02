import sys
import time

try:
    # Reuse the existing PD (18 joints) EtherCAT implementation
    from motor_igh_sdk.deploy_real_el4090_pysoem import RL_Real_PySOEM
except ImportError:
    from .deploy_real_el4090_pysoem import RL_Real_PySOEM

try:
    from motor_igh_sdk.el4090_motor_sdk import set_motor_speed
except ImportError:
    from .el4090_motor_sdk import set_motor_speed


class RL_Real_PySOEM_WithSpoolSpeed(RL_Real_PySOEM):
    """Extend `RL_Real_PySOEM` (18-joint PD) with ONE extra speed-mode motor.

    Requirements from user:
    - Keep the original 18-joint PD command/state interface unchanged.
    - Add motor_id=19 on a new 4th slave board: slave_idx=3, passage=1.
    - Provide a speed-command buffer in the same style as `RL_Real_Speed_PySOEM`.
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

        # Speed-mode command/state buffers (match RL_Real_Speed_PySOEM style)
        self.spool_command_buffer = type("SpoolCommandBuffer", (), {})()
        self.spool_command_buffer.target_speed_rpm = [0.0]
        self.spool_command_buffer.current_limit_01a = [500]  # 50.0A
        self.spool_command_buffer.ack_status = [1]

        self.spool_state_buffer = type("SpoolStateBuffer", (), {})()
        self.spool_state_buffer.position = [0.0]
        self.spool_state_buffer.velocity = [0.0]
        self.spool_state_buffer.torque = [0.0]

        self._spool_enabled = True

    def init_ethercat(self):
        ok = super().init_ethercat()
        if not ok:
            return False

        # Validate slave count for spool
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

        # 2) Send spool speed command on slave 3 / passage 1
        if not self._spool_enabled:
            return

        if self.spool_slave_idx >= len(self.master.slaves):
            return

        spd_rpm = self.spool_direction * float(self.spool_command_buffer.target_speed_rpm[0])
        cur_01a = int(self.spool_command_buffer.current_limit_01a[0])
        ack = int(self.spool_command_buffer.ack_status[0])

        tx_msg = self.motorData.getTxMsg(self.spool_slave_idx)
        set_motor_speed(
            tx_msg,
            int(self.spool_passage),
            int(self.spool_motor_id),
            spd_rpm=float(spd_rpm),
            cur_01a=int(cur_01a),
            ack_status=int(ack),
        )
        self.motorData.setTxMsg(self.spool_slave_idx, tx_msg)

    def HardwareRecv(self):
        # 1) Receive original 18-joint states
        super().HardwareRecv()

        # 2) Receive spool motor state
        if not self._spool_enabled:
            return

        if self.spool_slave_idx >= len(self.master.slaves):
            return

        motor_msg = self.motorData.getRxMotorMsg(self.spool_slave_idx, int(self.spool_passage))
        if motor_msg.motor_id != int(self.spool_motor_id):
            return

        # Keep same convention as joint motors: state buffers report URDF-side values.
        # For spool we default to direction-only (no offset).
        self.spool_state_buffer.position[0] = float(self.spool_direction * motor_msg.angle_actual_rad)
        self.spool_state_buffer.velocity[0] = float(self.spool_direction * motor_msg.speed_actual_rad)
        self.spool_state_buffer.torque[0] = float(motor_msg.current_actual_float)


if __name__ == "__main__":
    # Usage:
    #   python deploy_real_el4090_pysoem_spool_speed.py <ifname> [rpm]
    ifname = sys.argv[1] if len(sys.argv) > 1 else "enp86s0"
    rpm = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0

    print(
        f"Launching RL_Real_PySOEM_WithSpoolSpeed on {ifname} "
        f"(spool motor_id=19, slave_idx=3, passage=1), cmd_rpm={rpm}"
    )

    robot = None
    try:
        robot = RL_Real_PySOEM_WithSpoolSpeed(ifname)

        # Keep joints in zero-torque by default
        for i in range(robot.num_dofs):
            robot.motor_command_buffer.kp[i] = 0.0
            robot.motor_command_buffer.kd[i] = 0.0
            robot.motor_command_buffer.target_position[i] = 0.0
            robot.motor_command_buffer.target_velocity[i] = 0.0
            robot.motor_command_buffer.feedforward_torque[i] = 0.0

        # Spool speed command
        robot.spool_command_buffer.target_speed_rpm[0] = rpm
        robot.spool_command_buffer.current_limit_01a[0] = 500
        robot.spool_command_buffer.ack_status[0] = 1

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
                print(f"t={t:6.2f}s cmd_rpm={rpm:8.1f} vel(rad/s)={v:8.3f} pos(rad)={p:8.3f} cur={c:8.3f}")

    except KeyboardInterrupt:
        print("Stopping...")
    finally:
        if robot is not None:
            robot.stop()
