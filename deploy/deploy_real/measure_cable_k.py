import argparse
import csv
import math
import os
import sys
import time
from datetime import datetime

from hexapod_tethered_utils.cable_tension_sensor import CableTensionSensor
from hexapod_tethered_utils.joystick_reader import Gamepad
from motor_igh_sdk.deploy_real_el4090_pysoem_spool_torque import RL_Real_PySOEM_WithSpoolTorque


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_OUTPUT_DIR = os.path.join(SCRIPT_DIR, "cable_data", "K")

# A 后进入的安全初始力矩；每按一次 Y 增加 TORQUE_STEP_NM。
INITIAL_TORQUE_NM = 0.5
TORQUE_STEP_NM = 0.5


class TensionPlotter:
    def __init__(self, enabled: bool, history_s: float = 20.0, update_every: int = 5) -> None:
        self.enabled = bool(enabled)
        self.history_s = float(history_s)
        self.update_every = max(int(update_every), 1)
        self.counter = 0
        self.t_data = []
        self.tension_data = []
        self.ok = False

        if not self.enabled:
            return

        try:
            import matplotlib.pyplot as plt

            self.plt = plt
            self.plt.ion()
            self.fig, self.ax = self.plt.subplots(figsize=(9, 4))
            self.fig.canvas.manager.set_window_title("Cable tension")
            (self.line,) = self.ax.plot([], [], label="cable_tension_n")
            self.ax.set_xlabel("time (s)")
            self.ax.set_ylabel("tension (N)")
            self.ax.grid(True)
            self.ax.legend(loc="upper right")
            self.ok = True
        except Exception as e:
            self.enabled = False
            print(f"[Plot] disabled: {e}")

    def update(self, t_s: float, tension_n: float) -> None:
        if not self.enabled or not self.ok:
            return

        self.counter += 1
        self.t_data.append(float(t_s))
        self.tension_data.append(float(tension_n))

        t_min = float(t_s) - self.history_s
        while self.t_data and self.t_data[0] < t_min:
            self.t_data.pop(0)
            self.tension_data.pop(0)

        if (self.counter % self.update_every) != 0:
            return

        self.line.set_data(self.t_data, self.tension_data)
        self.ax.relim()
        self.ax.autoscale_view()
        try:
            self.fig.canvas.draw_idle()
            self.fig.canvas.flush_events()
            self.plt.pause(0.001)
        except Exception:
            self.enabled = False
            print("[Plot] disabled during runtime")


def init_joint_damping(robot, kp: float = 0.0, kd: float = 0.0) -> None:
    for i in range(robot.num_dofs):
        robot.motor_command_buffer.kp[i] = float(kp)
        robot.motor_command_buffer.kd[i] = float(kd)
        robot.motor_command_buffer.target_position[i] = 0.0
        robot.motor_command_buffer.target_velocity[i] = 0.0
        robot.motor_command_buffer.feedforward_torque[i] = 0.0


def wait_for_tension(sensor: CableTensionSensor, timeout_s: float = 5.0) -> None:
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        if sensor.get_cable_tension() is not None:
            return
        time.sleep(0.05)
    print("[WARNING] Timed out waiting for initial cable tension data.")


def build_output_path(output_dir: str) -> str:
    os.makedirs(output_dir, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return os.path.join(output_dir, f"cable_k_gamepad_{timestamp}.csv")


def main() -> int:
    parser = argparse.ArgumentParser(description="Hold one spool torque and log cable tension/angle.")
    parser.add_argument("--ifname", default="enp86s0")
    parser.add_argument("--sensor-port", default="/dev/ttyUSB_cable_tension")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--output-dir", default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--initial-torque-nm", type=float, default=INITIAL_TORQUE_NM)
    parser.add_argument("--torque-step-nm", type=float, default=TORQUE_STEP_NM)
    parser.add_argument("--sample-rate", type=float, default=50.0)
    parser.add_argument("--plot", action="store_true")
    args = parser.parse_args()

    torque_nm = 0.0
    armed = False
    output_path = build_output_path(args.output_dir)
    print(f"Saving data to: {output_path}")
    print(
        "Gamepad control: press A -> start at "
        f"{args.initial_torque_nm:.3f} Nm, press Y -> +{args.torque_step_nm:.3f} Nm, press LB -> stop."
    )

    robot = None
    sensor = None
    csv_file = None
    gamepad = None
    try:
        gamepad = Gamepad()

        robot = RL_Real_PySOEM_WithSpoolTorque(args.ifname)
        init_joint_damping(robot)
        robot.spool_command_buffer.target_torque_nm[0] = 0.0
        robot.spool_command_buffer.ctrl_status[0] = 1
        robot.spool_command_buffer.ack_status[0] = 1

        if not robot.start():
            print("Failed to start EtherCAT.")
            return 1

        sensor = CableTensionSensor(port=args.sensor_port, baudrate=args.baudrate)
        if not sensor.start():
            print("Failed to start cable tension sensor.")
            return 1
        wait_for_tension(sensor)

        csv_file = open(output_path, "w", newline="")
        writer = csv.writer(csv_file)
        writer.writerow(
            [
                "timestamp",
                "elapsed_s",
                "armed",
                "cmd_torque_nm",
                "actual_encoder_angle_rad",
                "actual_encoder_angle_deg",
                "cable_tension_n",
            ]
        )

        t0 = time.perf_counter()
        period = 1.0 / float(args.sample_rate)
        next_sample = time.perf_counter()
        rows = 0
        plotter = TensionPlotter(enabled=bool(args.plot))
        print("Logging with 0 Nm. Press A on gamepad to start.")

        while True:
            if gamepad.consume_a_click():
                armed = True
                torque_nm = float(args.initial_torque_nm)
                print(f"[Gamepad] A pressed: torque = {torque_nm:.6f} Nm")

            if armed and gamepad.consume_y_click():
                torque_nm += float(args.torque_step_nm)
                print(f"[Gamepad] Y pressed: torque = {torque_nm:.6f} Nm")

            if gamepad.consume_lb_click() or int(gamepad.get_button_lb()) == 1:
                print("[Gamepad] LB pressed: stopping.")
                break

            robot.spool_command_buffer.target_torque_nm[0] = torque_nm if armed else 0.0
            tension_n = sensor.get_cable_tension()
            if tension_n is not None:
                angle_rad = float(robot.spool_state_buffer.position[0])
                elapsed_s = time.perf_counter() - t0
                writer.writerow(
                    [
                        datetime.now().isoformat(timespec="milliseconds"),
                        f"{elapsed_s:.6f}",
                        int(armed),
                        f"{(torque_nm if armed else 0.0):.9f}",
                        f"{angle_rad:.9f}",
                        f"{math.degrees(angle_rad):.9f}",
                        f"{float(tension_n):.9f}",
                    ]
                )
                rows += 1
                plotter.update(elapsed_s, float(tension_n))
                if rows % int(max(args.sample_rate, 1.0)) == 0:
                    csv_file.flush()

            next_sample += period
            sleep_s = next_sample - time.perf_counter()
            if sleep_s > 0:
                time.sleep(sleep_s)
            else:
                next_sample = time.perf_counter()

    except KeyboardInterrupt:
        print("\nStopping...")
        return 0
    finally:
        if csv_file is not None:
            csv_file.flush()
            csv_file.close()
        if robot is not None:
            try:
                robot.spool_command_buffer.target_torque_nm[0] = 0.0
                time.sleep(0.05)
            except Exception:
                pass
        if sensor is not None:
            sensor.stop()
        if gamepad is not None:
            gamepad.stop()
        if robot is not None:
            robot.stop()
        print(f"Saved data to: {output_path}")


if __name__ == "__main__":
    sys.exit(main())
