import argparse
import csv
import math
import os
import sys
import time
from datetime import datetime

from hexapod_tethered_utils.cable_tension_sensor import CableTensionSensor
from motor_igh_sdk.deploy_real_el4090_pysoem_spool_position import RL_Real_PySOEM_WithSpoolPosition


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_OUTPUT_DIR = os.path.join(SCRIPT_DIR, "cable_data", "K")


def parse_positions(text: str) -> list[float]:
    return [float(item.strip()) for item in text.split(",") if item.strip()]


def default_positions() -> list[float]:
    return [round(0.1 * i, 3) for i in range(1, 11)]


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


def prompt_preload(sensor: CableTensionSensor, target_n: float, skip_prompt: bool) -> None:
    latest = sensor.get_cable_tension()
    if latest is None:
        print("[WARNING] No cable tension data yet.")
    else:
        print(f"Current cable tension: {latest:.3f} N")

    if skip_prompt:
        return

    input(f"Adjust pretension to about {target_n:.1f} N, then press Enter to start static K sampling...")


def build_output_path(output_dir: str) -> str:
    os.makedirs(output_dir, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return os.path.join(output_dir, f"cable_k_static_{timestamp}.csv")


def main() -> int:
    parser = argparse.ArgumentParser(description="Collect static cable tension data for K estimation.")
    parser.add_argument("--ifname", default="enp86s0")
    parser.add_argument("--sensor-port", default="/dev/ttyUSB_cable_tension")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--output-dir", default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--positions-deg", default=",".join(str(v) for v in default_positions()))
    parser.add_argument("--settle-time", type=float, default=2.0)
    parser.add_argument("--sample-time", type=float, default=1.0)
    parser.add_argument("--sample-rate", type=float, default=50.0)
    parser.add_argument("--speed-limit-01rpm", type=int, default=10)
    parser.add_argument("--current-limit-01a", type=int, default=60)
    parser.add_argument("--initial-position-deg", type=float, default=0.0)
    parser.add_argument("--joint-kp", type=float, default=0.0)
    parser.add_argument("--joint-kd", type=float, default=0.0)
    parser.add_argument("--preload-target-n", type=float, default=50.0)
    parser.add_argument("--sensor-timeout", type=float, default=5.0)
    parser.add_argument("--skip-preload-prompt", action="store_true")
    args = parser.parse_args()

    positions_deg = parse_positions(args.positions_deg)
    if not positions_deg:
        raise ValueError("--positions-deg must contain at least one position")

    output_path = build_output_path(args.output_dir)
    print(f"Saving K data to: {output_path}")

    robot = None
    sensor = None
    try:
        robot = RL_Real_PySOEM_WithSpoolPosition(args.ifname)
        init_joint_damping(robot, args.joint_kp, args.joint_kd)
        robot.spool_command_buffer.target_position_deg[0] = float(args.initial_position_deg)
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

        prompt_preload(sensor, args.preload_target_n, args.skip_preload_prompt)

        base_encoder_deg = get_spool_encoder_deg(robot)
        print(f"Using current motor encoder angle as static baseline: {base_encoder_deg:.6f} deg")

        with open(output_path, "w", newline="") as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(
                [
                    "timestamp",
                    "elapsed_s",
                    "step_index",
                    "target_delta_deg",
                    "target_position_deg",
                    "actual_encoder_angle_deg",
                    "actual_encoder_angle_rad",
                    "cable_tension_n",
                ]
            )

            t0 = time.perf_counter()
            period = 1.0 / float(args.sample_rate)

            for step_index, delta_deg in enumerate([0.0] + positions_deg):
                target_deg = base_encoder_deg + float(delta_deg)
                robot.spool_command_buffer.target_position_deg[0] = target_deg
                print(
                    f"[K] step={step_index:02d} delta={delta_deg:.3f} deg "
                    f"target={target_deg:.6f} deg, settling {args.settle_time:.2f}s"
                )
                time.sleep(args.settle_time)

                sample_end = time.perf_counter() + float(args.sample_time)
                next_sample = time.perf_counter()
                rows = 0
                while time.perf_counter() < sample_end:
                    tension_n = sensor.get_cable_tension()
                    if tension_n is not None:
                        angle_deg = get_spool_encoder_deg(robot)
                        elapsed_s = time.perf_counter() - t0
                        writer.writerow(
                            [
                                datetime.now().isoformat(timespec="milliseconds"),
                                f"{elapsed_s:.6f}",
                                step_index,
                                f"{delta_deg:.6f}",
                                f"{target_deg:.6f}",
                                f"{angle_deg:.9f}",
                                f"{math.radians(angle_deg):.9f}",
                                f"{float(tension_n):.9f}",
                            ]
                        )
                        rows += 1

                    next_sample += period
                    sleep_s = next_sample - time.perf_counter()
                    if sleep_s > 0:
                        time.sleep(sleep_s)

                csv_file.flush()
                print(f"[K] step={step_index:02d} saved {rows} samples")

        print(f"Done. Data saved to: {output_path}")
        return 0

    except KeyboardInterrupt:
        print("Stopping...")
        return 130
    finally:
        if sensor is not None:
            sensor.stop()
        if robot is not None:
            robot.stop()


if __name__ == "__main__":
    sys.exit(main())
