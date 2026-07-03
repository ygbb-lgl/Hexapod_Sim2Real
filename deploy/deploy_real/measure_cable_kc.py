import argparse
import csv
import math
import os
import sys
import time
from datetime import datetime

from hexapod_tethered_utils.cable_tension_sensor import CableTensionSensor
from hexapod_tethered_utils.joystick_reader import Gamepad
from motor_igh_sdk.deploy_real_el4090_pysoem_spool_position import RL_Real_PySOEM_WithSpoolPosition


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_OUTPUT_DIR = os.path.join(SCRIPT_DIR, "cable_data", "K_C")


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
            self.fig.canvas.manager.set_window_title("Cable tension K/C")
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


def init_joint_damping(robot, kp: float, kd: float) -> None:
    for i in range(robot.num_dofs):
        robot.motor_command_buffer.kp[i] = float(kp)
        robot.motor_command_buffer.kd[i] = float(kd)
        robot.motor_command_buffer.target_position[i] = 0.0
        robot.motor_command_buffer.target_velocity[i] = 0.0
        robot.motor_command_buffer.feedforward_torque[i] = 0.0


def get_spool_encoder_deg(robot) -> float:
    motor_msg = robot.motorData.getRxMotorMsg(robot.spool_slave_idx, int(robot.spool_passage))
    if int(motor_msg.motor_id) == int(robot.spool_motor_id):
        return float(motor_msg.angle_actual_float)
    return math.degrees(float(robot.spool_state_buffer.position[0]))


def wait_for_tension(sensor: CableTensionSensor, timeout_s: float) -> bool:
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        if sensor.get_cable_tension() is not None:
            return True
        time.sleep(0.05)
    return False


def run_auto_preload(
    robot,
    sensor: CableTensionSensor,
    min_tension_n: float,
    step_deg: float,
    direction: float,
    settle_time: float,
    max_delta_deg: float,
    timeout_s: float,
) -> float:
    start_deg = get_spool_encoder_deg(robot)
    target_deg = start_deg
    robot.spool_command_buffer.target_position_deg[0] = target_deg
    robot.spool_command_buffer.command_enabled[0] = True
    time.sleep(settle_time)

    deadline = time.time() + timeout_s
    print(f"Auto preload: require tension >= {min_tension_n:.3f} N, start_encoder={start_deg:.6f} deg")
    while time.time() < deadline:
        tension_n = sensor.get_cable_tension()
        if tension_n is None:
            time.sleep(0.05)
            continue

        print(f"[preload] tension={tension_n:.3f} N cmd={target_deg:.6f} deg")
        if float(tension_n) >= float(min_tension_n):
            return get_spool_encoder_deg(robot)

        target_deg += float(direction) * abs(float(step_deg))
        if abs(target_deg - start_deg) > abs(float(max_delta_deg)):
            raise RuntimeError(
                f"Auto preload exceeded max_delta_deg={max_delta_deg}. "
                "Check --preload-direction or mechanical setup."
            )

        robot.spool_command_buffer.target_position_deg[0] = target_deg
        time.sleep(settle_time)

    raise TimeoutError(f"Auto preload did not reach {min_tension_n:.3f} N after {timeout_s:.1f}s")


def _label_float(value: float) -> str:
    return f"{float(value):.3f}".replace("-", "m").replace(".", "p")


def build_output_path(output_dir: str, amplitude_deg: float, frequency_hz: float) -> str:
    os.makedirs(output_dir, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    amp_label = _label_float(amplitude_deg)
    freq_label = _label_float(frequency_hz)
    return os.path.join(output_dir, f"cable_kc_sine_{timestamp}_A{amp_label}deg_f{freq_label}Hz.csv")


def main() -> int:
    parser = argparse.ArgumentParser(description="Collect sine cable tension data for K/C estimation.")
    parser.add_argument("--ifname", default="enp86s0")
    parser.add_argument("--sensor-port", default="/dev/ttyUSB_cable_tension")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--output-dir", default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--duration", type=float, default=60.0)
    parser.add_argument("--sample-rate", type=float, default=150.0)
    parser.add_argument("--amplitude-deg", type=float, default=1.0)
    parser.add_argument("--frequency-hz", type=float, default=0.1)
    parser.add_argument("--degree0-deg", type=float, default=0.0)
    parser.add_argument("--speed-limit-01rpm", type=int, default=10)
    parser.add_argument("--current-limit-01a", type=int, default=60)
    parser.add_argument("--joint-kp", type=float, default=0.0)
    parser.add_argument("--joint-kd", type=float, default=0.0)
    parser.add_argument("--preload-min-tension-n", type=float, default=25.0)
    parser.add_argument("--preload-step-deg", type=float, default=0.1)
    parser.add_argument("--preload-direction", type=float, default=1.0)
    parser.add_argument("--preload-settle-time", type=float, default=0.2)
    parser.add_argument("--preload-max-delta-deg", type=float, default=10.0)
    parser.add_argument("--preload-timeout", type=float, default=30.0)
    parser.add_argument("--sensor-timeout", type=float, default=5.0)
    parser.add_argument("--plot", action="store_true")
    args = parser.parse_args()

    if args.duration <= 0:
        raise ValueError("--duration must be > 0")
    if args.sample_rate <= 0:
        raise ValueError("--sample-rate must be > 0")
    if args.frequency_hz <= 0:
        raise ValueError("--frequency-hz must be > 0")

    output_path = build_output_path(args.output_dir, args.amplitude_deg, args.frequency_hz)
    print(f"Saving K/C data to: {output_path}")
    print(
        "Gamepad control: press A -> preload until tension threshold, "
        "press Y -> start sine sampling, press LB -> exit."
    )

    robot = None
    sensor = None
    gamepad = None
    try:
        gamepad = Gamepad()

        robot = RL_Real_PySOEM_WithSpoolPosition(args.ifname)
        init_joint_damping(robot, args.joint_kp, args.joint_kd)
        robot.spool_command_buffer.speed_limit_01rpm[0] = int(args.speed_limit_01rpm)
        robot.spool_command_buffer.current_limit_01a[0] = int(args.current_limit_01a)
        robot.spool_command_buffer.ack_status[0] = 2

        if not robot.start():
            print("Failed to start EtherCAT.")
            return 1

        sensor = CableTensionSensor(port=args.sensor_port, baudrate=args.baudrate)
        if not sensor.start():
            print("Failed to start cable tension sensor.")
            return 1

        if not wait_for_tension(sensor, args.sensor_timeout):
            print("[WARNING] Timed out waiting for initial cable tension data.")

        print("Waiting for gamepad A to start preload...")
        while True:
            if gamepad.consume_lb_click() or int(gamepad.get_button_lb()) == 1:
                print("[Gamepad] LB pressed before preload: exit.")
                return 0
            if gamepad.consume_a_click():
                print("[Gamepad] A pressed: start preload.")
                break
            time.sleep(0.02)

        preload_encoder_deg = run_auto_preload(
            robot,
            sensor,
            min_tension_n=args.preload_min_tension_n,
            step_deg=args.preload_step_deg,
            direction=args.preload_direction,
            settle_time=args.preload_settle_time,
            max_delta_deg=args.preload_max_delta_deg,
            timeout_s=args.preload_timeout,
        )

        degree0_deg = float(args.degree0_deg)
        start_encoder_deg = get_spool_encoder_deg(robot)
        omega = 2.0 * math.pi * float(args.frequency_hz)
        print(
            f"Using command sine center degree0={degree0_deg:.6f} deg; "
            f"preload_encoder={preload_encoder_deg:.6f} deg, start_encoder={start_encoder_deg:.6f} deg, "
            f"amplitude={args.amplitude_deg:.6f} deg, "
            f"frequency={args.frequency_hz:.6f} Hz"
        )

        print("Waiting for gamepad Y to start sine sampling...")
        while True:
            if gamepad.consume_lb_click() or int(gamepad.get_button_lb()) == 1:
                print("[Gamepad] LB pressed before sine sampling: exit.")
                return 0
            if gamepad.consume_y_click():
                print("[Gamepad] Y pressed: start sine sampling.")
                break
            time.sleep(0.02)

        with open(output_path, "w", newline="") as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(
                [
                    "timestamp",
                    "elapsed_s",
                    "target_position_deg",
                    "target_delta_deg",
                    "theoretical_velocity_deg_s",
                    "theoretical_velocity_rad_s",
                    "actual_encoder_angle_deg",
                    "actual_encoder_angle_rad",
                    "actual_encoder_velocity_deg_s",
                    "actual_encoder_velocity_rad_s",
                    "cable_tension_n",
                ]
            )

            t0 = time.perf_counter()
            period = 1.0 / float(args.sample_rate)
            next_sample = t0
            rows = 0
            plotter = TensionPlotter(enabled=bool(args.plot))
            prev_angle_deg = None
            prev_elapsed_s = None

            while True:
                if gamepad.consume_lb_click() or int(gamepad.get_button_lb()) == 1:
                    print("[Gamepad] LB pressed: stopping sine sampling.")
                    break

                now = time.perf_counter()
                elapsed_s = now - t0
                if elapsed_s > float(args.duration):
                    break

                target_delta_deg = float(args.amplitude_deg) * math.sin(omega * elapsed_s)
                target_position_deg = degree0_deg + target_delta_deg
                theoretical_velocity_deg_s = float(args.amplitude_deg) * omega * math.cos(omega * elapsed_s)
                theoretical_velocity_rad_s = math.radians(theoretical_velocity_deg_s)

                robot.spool_command_buffer.target_position_deg[0] = target_position_deg

                tension_n = sensor.get_cable_tension()
                if tension_n is not None:
                    angle_deg = get_spool_encoder_deg(robot)
                    if prev_angle_deg is None or prev_elapsed_s is None:
                        actual_velocity_deg_s = 0.0
                    else:
                        dt = elapsed_s - prev_elapsed_s
                        if dt > 1e-9:
                            actual_velocity_deg_s = (angle_deg - prev_angle_deg) / dt
                        else:
                            actual_velocity_deg_s = 0.0
                    actual_velocity_rad_s = math.radians(actual_velocity_deg_s)

                    writer.writerow(
                        [
                            datetime.now().isoformat(timespec="milliseconds"),
                            f"{elapsed_s:.6f}",
                            f"{target_position_deg:.6f}",
                            f"{target_delta_deg:.6f}",
                            f"{theoretical_velocity_deg_s:.9f}",
                            f"{theoretical_velocity_rad_s:.9f}",
                            f"{angle_deg:.9f}",
                            f"{math.radians(angle_deg):.9f}",
                            f"{actual_velocity_deg_s:.9f}",
                            f"{actual_velocity_rad_s:.9f}",
                            f"{float(tension_n):.9f}",
                        ]
                    )
                    rows += 1
                    prev_angle_deg = angle_deg
                    prev_elapsed_s = elapsed_s
                    plotter.update(elapsed_s, float(tension_n))

                next_sample += period
                sleep_s = next_sample - time.perf_counter()
                if sleep_s > 0:
                    time.sleep(sleep_s)

        print(f"Done. Saved {rows} samples to: {output_path}")
        return 0

    except KeyboardInterrupt:
        print("Stopping...")
        return 130
    finally:
        if sensor is not None:
            sensor.stop()
        if gamepad is not None:
            gamepad.stop()
        if robot is not None:
            robot.stop()


if __name__ == "__main__":
    sys.exit(main())
