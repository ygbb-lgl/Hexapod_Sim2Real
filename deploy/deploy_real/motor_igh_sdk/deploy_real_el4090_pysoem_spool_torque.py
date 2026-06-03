import sys
import time

try:
    # Script-style (recommended when running this file directly)
    from deploy_real_el4090_pysoem import RL_Real_PySOEM
except ImportError:
    from .deploy_real_el4090_pysoem import RL_Real_PySOEM

try:
    # Script-style (recommended when running this file directly)
    from el4090_motor_sdk import set_motor_cur_tor, set_motor_torque
except ImportError:
    from .el4090_motor_sdk import set_motor_cur_tor, set_motor_torque


class RL_Real_PySOEM_WithSpoolTorque(RL_Real_PySOEM):
    """Extend `RL_Real_PySOEM` (18-joint PD) with ONE extra torque-mode motor.

    Requirements from user:
    - Keep the original 18-joint PD command/state interface unchanged.
    - Add motor_id=19 on a new 4th slave board: slave_idx=3, passage=1.
    - Provide a torque-command buffer in the same style as the standalone torque wrapper.
    - Use the same background pd_loop thread (do NOT create a second EtherCAT master).

    Notes:
    - This class assumes the EtherCAT chain now has 4 slaves.
    - If fewer slaves are detected, spool control is disabled with a warning.
    - Torque command uses official ask-mode: `set_motor_cur_tor` (ctrl_status=1).
      Scaling: 0.01 Nm per LSB; helper `set_motor_torque` accepts Nm float.
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

        # Torque-mode command/state buffers (aligned with spool_speed style)
        self.spool_command_buffer = type("SpoolCommandBuffer", (), {})()
        self.spool_command_buffer.target_torque_nm = [0.0]
        # ctrl_status: 1=torque control; see docs for brake modes.
        self.spool_command_buffer.ctrl_status = [1]
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

        # 2) Send spool torque command on slave 3 / passage 1
        if not self._spool_enabled:
            return
        if self.spool_slave_idx >= len(self.master.slaves):
            return

        torque_nm = self.spool_direction * float(self.spool_command_buffer.target_torque_nm[0])
        ctrl_status = int(self.spool_command_buffer.ctrl_status[0])
        ack = int(self.spool_command_buffer.ack_status[0])

        tx_msg = self.motorData.getTxMsg(self.spool_slave_idx)
        if ctrl_status == 1:
            set_motor_torque(
                tx_msg,
                int(self.spool_passage),
                int(self.spool_motor_id),
                torque_nm=float(torque_nm),
                ack_status=int(ack),
            )
        else:
            cur_tor_001 = int(round(float(torque_nm) * 100.0))
            set_motor_cur_tor(
                tx_msg,
                int(self.spool_passage),
                int(self.spool_motor_id),
                cur_tor_001=int(cur_tor_001),
                ctrl_status=int(ctrl_status),
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

        self.spool_state_buffer.position[0] = float(self.spool_direction * motor_msg.angle_actual_rad)
        self.spool_state_buffer.velocity[0] = float(self.spool_direction * motor_msg.speed_actual_rad)
        self.spool_state_buffer.torque[0] = float(motor_msg.current_actual_float)


if __name__ == "__main__":
    # Usage:
    #   python deploy_real_el4090_pysoem_spool_torque.py <ifname> [torque_nm]
    # NOTE:
    #   default torque is 0.0 (safer); set a small value to test.
    ifname = sys.argv[1] if len(sys.argv) > 1 else "enp86s0"
    torque_nm = float(sys.argv[2]) if len(sys.argv) > 2 else 2.0

    print(
        f"Launching RL_Real_PySOEM_WithSpoolTorque on {ifname} "
        f"(spool motor_id=19, slave_idx=3, passage=1), cmd_torque_nm={torque_nm}"
    )

    robot = None
    try:
        robot = RL_Real_PySOEM_WithSpoolTorque(ifname)

        # Keep joints in zero-torque by default
        for i in range(robot.num_dofs):
            robot.motor_command_buffer.kp[i] = 0.0
            robot.motor_command_buffer.kd[i] = 0.0
            robot.motor_command_buffer.target_position[i] = 0.0
            robot.motor_command_buffer.target_velocity[i] = 0.0
            robot.motor_command_buffer.feedforward_torque[i] = 0.0

        # Spool torque command
        robot.spool_command_buffer.target_torque_nm[0] = float(torque_nm)
        robot.spool_command_buffer.ctrl_status[0] = 1
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
                print(
                    f"t={t:6.2f}s cmd_torque_nm={torque_nm:8.3f} "
                    f"vel(rad/s)={v:8.3f} pos(rad)={p:8.3f} cur={c:8.3f}"
                )

    except KeyboardInterrupt:
        print("Stopping...")
    finally:
        if robot is not None:
            robot.stop()
