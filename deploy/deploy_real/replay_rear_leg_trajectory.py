#!/usr/bin/env python3
"""Replay recorded joint targets on one rear leg.

Only the selected rear leg is position-PD controlled.  All other joints stay
in damping mode for the entire experiment.

Examples:
    # Right-back leg: motor IDs 1/3/2 (policy indices 6/7/8)
    sudo python replay_rear_leg_trajectory.py rb

    # Left-back leg: motor IDs 4/6/5 (policy indices 15/16/17)
    sudo python replay_rear_leg_trajectory.py lb

State-machine controls:
    Y  -> move the selected leg to config.default_angles
    A  -> start/restart trajectory replay from the first point
    LB -> immediately put all 18 joints in damping and exit

Ctrl+C has the same safe-exit behavior as LB.
"""

import argparse
import csv
import math
import re
import statistics
import sys
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, TextIO, Tuple


SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_TRAJECTORY_DIR = (
    SCRIPT_DIR / "0729-data" / "motor_logs_20260729_105910"
)
DEFAULT_OUTPUT_ROOT = SCRIPT_DIR / "single_leg_experiment_logs"

# Motor order follows HAA, HFE, KFE for each leg.
LEG_MOTOR_IDS = {
    "rb": (1, 3, 2),
    "lb": (4, 6, 5),
}
LEG_NAMES = {
    "rb": "right_back",
    "lb": "left_back",
}

NUM_JOINTS = 18
DEFAULT_KP = 100.0
DEFAULT_KD = 1.2
DEFAULT_DAMPING_KD = 5.0

CSV_NAME_RE = re.compile(
    r"^motor_(?P<motor_id>\d+)_policy_idx_(?P<policy_idx>\d+)\.csv$"
)


class DampingExitRequested(Exception):
    """Request immediate transition to all-joint damping and shutdown."""


@dataclass(frozen=True)
class MotorTrajectory:
    motor_id: int
    policy_idx: int
    source_path: Path
    time_s: Tuple[float, ...]
    target_pos_rad: Tuple[float, ...]


@dataclass(frozen=True)
class TrajectoryBundle:
    leg: str
    motor_ids: Tuple[int, int, int]
    trajectories: Dict[int, MotorTrajectory]
    sample_count: int
    source_dt_s: float


def _finite_float(value: str, field: str, path: Path, row_number: int) -> float:
    try:
        parsed = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(
            f"{path}: row {row_number} has invalid {field}={value!r}"
        ) from exc
    if not math.isfinite(parsed):
        raise ValueError(
            f"{path}: row {row_number} has non-finite {field}={value!r}"
        )
    return parsed


def load_motor_trajectory(path: Path, expected_motor_id: int) -> MotorTrajectory:
    match = CSV_NAME_RE.match(path.name)
    if match is None:
        raise ValueError(f"Unexpected trajectory filename: {path.name}")

    motor_id = int(match.group("motor_id"))
    policy_idx = int(match.group("policy_idx"))
    if motor_id != expected_motor_id:
        raise ValueError(
            f"{path}: filename motor ID {motor_id} does not match "
            f"expected motor ID {expected_motor_id}"
        )

    time_values: List[float] = []
    targets: List[float] = []
    with path.open("r", newline="") as csv_file:
        reader = csv.DictReader(csv_file)
        required = {"time_step", "target_pos"}
        missing = required.difference(reader.fieldnames or ())
        if missing:
            raise ValueError(
                f"{path}: missing required columns: {sorted(missing)}"
            )

        for row_number, row in enumerate(reader, start=2):
            time_values.append(
                _finite_float(row["time_step"], "time_step", path, row_number)
            )
            targets.append(
                _finite_float(row["target_pos"], "target_pos", path, row_number)
            )

    if len(targets) < 2:
        raise ValueError(f"{path}: at least two trajectory rows are required")

    for index in range(1, len(time_values)):
        if time_values[index] <= time_values[index - 1]:
            raise ValueError(
                f"{path}: time_step must be strictly increasing "
                f"(rows {index + 1} and {index + 2})"
            )

    return MotorTrajectory(
        motor_id=motor_id,
        policy_idx=policy_idx,
        source_path=path,
        time_s=tuple(time_values),
        target_pos_rad=tuple(targets),
    )


def load_trajectory_bundle(trajectory_dir: Path, leg: str) -> TrajectoryBundle:
    trajectory_dir = trajectory_dir.resolve()
    if not trajectory_dir.is_dir():
        raise FileNotFoundError(f"Trajectory directory not found: {trajectory_dir}")

    motor_ids = LEG_MOTOR_IDS[leg]
    trajectories: Dict[int, MotorTrajectory] = {}
    for motor_id in motor_ids:
        matches = sorted(
            trajectory_dir.glob(f"motor_{motor_id:02d}_policy_idx_*.csv")
        )
        if len(matches) != 1:
            raise ValueError(
                f"Expected exactly one trajectory CSV for motor {motor_id}, "
                f"found {len(matches)} in {trajectory_dir}"
            )
        trajectories[motor_id] = load_motor_trajectory(matches[0], motor_id)

    reference = trajectories[motor_ids[0]]
    sample_count = len(reference.target_pos_rad)
    reference_times = reference.time_s

    for motor_id in motor_ids[1:]:
        trajectory = trajectories[motor_id]
        if len(trajectory.target_pos_rad) != sample_count:
            raise ValueError(
                f"Trajectory length mismatch: motor {reference.motor_id} has "
                f"{sample_count} rows, motor {motor_id} has "
                f"{len(trajectory.target_pos_rad)} rows"
            )
        max_time_error = max(
            abs(lhs - rhs)
            for lhs, rhs in zip(reference_times, trajectory.time_s)
        )
        if max_time_error > 1e-6:
            raise ValueError(
                f"Trajectory time_step mismatch for motor {motor_id}; "
                f"maximum difference is {max_time_error:.9f} s"
            )

    intervals = [
        reference_times[index] - reference_times[index - 1]
        for index in range(1, sample_count)
    ]
    source_dt_s = float(statistics.median(intervals))
    max_dt_error = max(abs(interval - source_dt_s) for interval in intervals)
    if max_dt_error > max(1e-6, source_dt_s * 0.05):
        raise ValueError(
            "The source trajectory does not have a sufficiently uniform sample "
            f"period: median={source_dt_s:.9f} s, "
            f"maximum deviation={max_dt_error:.9f} s"
        )

    return TrajectoryBundle(
        leg=leg,
        motor_ids=motor_ids,
        trajectories=trajectories,
        sample_count=sample_count,
        source_dt_s=source_dt_s,
    )


def set_joint_damping(robot, policy_idx: int, damping_kd: float) -> None:
    robot.motor_command_buffer.kp[policy_idx] = 0.0
    robot.motor_command_buffer.kd[policy_idx] = damping_kd
    robot.motor_command_buffer.target_position[policy_idx] = 0.0
    robot.motor_command_buffer.target_velocity[policy_idx] = 0.0
    robot.motor_command_buffer.feedforward_torque[policy_idx] = 0.0


def set_all_damping(
    robot,
    damping_kd: float,
    excluded_policy_indices: Iterable[int] = (),
) -> None:
    excluded = set(excluded_policy_indices)
    for policy_idx in range(NUM_JOINTS):
        if policy_idx not in excluded:
            set_joint_damping(robot, policy_idx, damping_kd)


def set_position_pd(
    robot,
    policy_idx: int,
    target_pos_rad: float,
    kp: float,
    kd: float,
) -> None:
    robot.motor_command_buffer.kp[policy_idx] = kp
    robot.motor_command_buffer.kd[policy_idx] = kd
    robot.motor_command_buffer.target_position[policy_idx] = target_pos_rad
    robot.motor_command_buffer.target_velocity[policy_idx] = 0.0
    robot.motor_command_buffer.feedforward_torque[policy_idx] = 0.0


class ExperimentLogger:
    FIELDNAMES = (
        "time_step",
        "counter",
        "phase",
        "cycle",
        "trajectory_index",
        "trajectory_time_s",
        "motor_id",
        "policy_idx",
        "target_pos",
        "actual_pos",
        "error",
        "actual_current_a",
        "actual_velocity_rad_s",
        "kp",
        "kd",
    )

    def __init__(
        self,
        output_dir: Path,
        bundle: TrajectoryBundle,
        flush_every: int,
    ) -> None:
        self.output_dir = output_dir.resolve()
        self.output_dir.mkdir(parents=True, exist_ok=False)
        self.flush_every = max(int(flush_every), 1)
        self.files: Dict[int, TextIO] = {}
        self.writers: Dict[int, csv.DictWriter] = {}
        self.counter = 0

        try:
            for motor_id in bundle.motor_ids:
                trajectory = bundle.trajectories[motor_id]
                path = self.output_dir / (
                    f"motor_{motor_id:02d}_policy_idx_"
                    f"{trajectory.policy_idx:02d}.csv"
                )
                csv_file = path.open("w", newline="")
                writer = csv.DictWriter(csv_file, fieldnames=self.FIELDNAMES)
                writer.writeheader()
                self.files[motor_id] = csv_file
                self.writers[motor_id] = writer
        except Exception:
            self.close()
            raise

    def write_sample(
        self,
        robot,
        bundle: TrajectoryBundle,
        targets: Dict[int, float],
        elapsed_s: float,
        phase: str,
        cycle: int,
        trajectory_index: int,
        trajectory_time_s: float,
        kp: float,
        kd: float,
    ) -> None:
        self.counter += 1
        for motor_id in bundle.motor_ids:
            trajectory = bundle.trajectories[motor_id]
            policy_idx = trajectory.policy_idx
            target_pos = float(targets[motor_id])
            actual_pos = float(robot.motor_state_buffer.position[policy_idx])
            actual_current = float(robot.motor_state_buffer.torque[policy_idx])
            actual_velocity = float(robot.motor_state_buffer.velocity[policy_idx])

            self.writers[motor_id].writerow(
                {
                    "time_step": f"{elapsed_s:.9f}",
                    "counter": self.counter,
                    "phase": phase,
                    "cycle": cycle,
                    "trajectory_index": trajectory_index,
                    "trajectory_time_s": f"{trajectory_time_s:.9f}",
                    "motor_id": motor_id,
                    "policy_idx": policy_idx,
                    "target_pos": f"{target_pos:.9f}",
                    "actual_pos": f"{actual_pos:.9f}",
                    "error": f"{actual_pos - target_pos:.9f}",
                    # The reference SDK stores current_actual_float in the
                    # state buffer field named "torque".
                    "actual_current_a": f"{actual_current:.9f}",
                    "actual_velocity_rad_s": f"{actual_velocity:.9f}",
                    "kp": f"{kp:.6f}",
                    "kd": f"{kd:.6f}",
                }
            )

        if self.counter % self.flush_every == 0:
            self.flush()

    def flush(self) -> None:
        for csv_file in self.files.values():
            csv_file.flush()

    def close(self) -> None:
        for csv_file in self.files.values():
            try:
                csv_file.close()
            except Exception:
                pass
        self.files.clear()
        self.writers.clear()


def _wait_until(deadline: float) -> float:
    remaining = deadline - time.perf_counter()
    if remaining > 0.0:
        time.sleep(remaining)
    return time.perf_counter()


def _command_targets(
    robot,
    bundle: TrajectoryBundle,
    targets: Dict[int, float],
    kp: float,
    kd: float,
    damping_kd: float,
) -> None:
    active_indices = {
        bundle.trajectories[motor_id].policy_idx
        for motor_id in bundle.motor_ids
    }
    set_all_damping(robot, damping_kd, excluded_policy_indices=active_indices)
    for motor_id in bundle.motor_ids:
        set_position_pd(
            robot,
            bundle.trajectories[motor_id].policy_idx,
            targets[motor_id],
            kp,
            kd,
        )


def _check_runtime_or_raise(robot, gamepad) -> None:
    if not bool(gamepad.is_running):
        raise DampingExitRequested("gamepad reader stopped or disconnected")
    if int(gamepad.get_button_lb()) == 1:
        raise DampingExitRequested("LB pressed")
    if not robot.running:
        raise RuntimeError("EtherCAT process-data thread stopped unexpectedly")


def _wait_for_y(
    robot,
    gamepad,
    control_dt: float,
    damping_kd: float,
) -> None:
    print(
        "[State] All 18 joints are in damping. "
        "Press Y to move the selected leg to its initial position; "
        "press LB to exit."
    )
    while True:
        _check_runtime_or_raise(robot, gamepad)
        set_all_damping(robot, damping_kd)
        if int(gamepad.get_button_y()) == 1:
            return
        time.sleep(control_dt)


def _move_to_initial(
    robot,
    gamepad,
    bundle: TrajectoryBundle,
    initial_targets: Dict[int, float],
    logger: ExperimentLogger,
    experiment_start: float,
    ramp_seconds: float,
    control_dt: float,
    kp: float,
    kd: float,
    damping_kd: float,
) -> None:
    start_positions = {
        motor_id: float(
            robot.motor_state_buffer.position[
                bundle.trajectories[motor_id].policy_idx
            ]
        )
        for motor_id in bundle.motor_ids
    }
    ramp_steps = max(int(math.ceil(ramp_seconds / control_dt)), 1)
    next_deadline = time.perf_counter()

    print(
        f"[State] Y: moving selected leg to its configured initial position over "
        f"{ramp_steps} steps ({ramp_steps * control_dt:.2f} s)."
    )
    for step in range(ramp_steps):
        now = _wait_until(next_deadline)
        _check_runtime_or_raise(robot, gamepad)
        alpha = float(step + 1) / float(ramp_steps)
        targets = {
            motor_id: (
                start_positions[motor_id] * (1.0 - alpha)
                + initial_targets[motor_id] * alpha
            )
            for motor_id in bundle.motor_ids
        }
        _command_targets(robot, bundle, targets, kp, kd, damping_kd)
        logger.write_sample(
            robot=robot,
            bundle=bundle,
            targets=targets,
            elapsed_s=now - experiment_start,
            phase="return_to_initial",
            cycle=-1,
            trajectory_index=0,
            trajectory_time_s=0.0,
            kp=kp,
            kd=kd,
        )
        next_deadline += control_dt
        if now - next_deadline > control_dt:
            next_deadline = now + control_dt


def _hold_initial_until_a(
    robot,
    gamepad,
    bundle: TrajectoryBundle,
    initial_targets: Dict[int, float],
    logger: ExperimentLogger,
    experiment_start: float,
    control_dt: float,
    kp: float,
    kd: float,
    damping_kd: float,
) -> None:
    next_deadline = time.perf_counter()
    print(
        "[State] Holding the selected leg at the initial position. "
        "Press A to start motion; press LB to exit."
    )

    while True:
        now = _wait_until(next_deadline)
        _check_runtime_or_raise(robot, gamepad)
        _command_targets(
            robot, bundle, initial_targets, kp, kd, damping_kd
        )
        logger.write_sample(
            robot=robot,
            bundle=bundle,
            targets=initial_targets,
            elapsed_s=now - experiment_start,
            phase="hold_initial",
            cycle=-1,
            trajectory_index=0,
            trajectory_time_s=0.0,
            kp=kp,
            kd=kd,
        )
        if int(gamepad.get_button_a()) == 1:
            print("[State] A: starting trajectory replay from point 0.")
            return

        next_deadline += control_dt
        if now - next_deadline > control_dt:
            next_deadline = now + control_dt


def _run_replay_loop(
    robot,
    gamepad,
    bundle: TrajectoryBundle,
    logger: ExperimentLogger,
    experiment_start: float,
    control_dt: float,
    kp: float,
    kd: float,
    damping_kd: float,
    cycles: int,
) -> bool:
    """Replay until Y is pressed or the requested cycle count completes.

    Returns True when Y requested a return to the initial position, otherwise
    False when a finite cycle count completed.
    """
    cycle = 0
    next_deadline = time.perf_counter()
    overrun_count = 0

    print(
        f"[Replay] Started: {bundle.sample_count} points/cycle, "
        f"dt={control_dt:.6f} s. Y=return, LB=exit."
    )
    while cycles == 0 or cycle < cycles:
        for trajectory_index in range(bundle.sample_count):
            now = _wait_until(next_deadline)
            _check_runtime_or_raise(robot, gamepad)
            if int(gamepad.get_button_y()) == 1:
                print("[State] Y: returning to the initial position.")
                return True

            targets = {
                motor_id: bundle.trajectories[motor_id].target_pos_rad[
                    trajectory_index
                ]
                for motor_id in bundle.motor_ids
            }
            _command_targets(robot, bundle, targets, kp, kd, damping_kd)
            logger.write_sample(
                robot=robot,
                bundle=bundle,
                targets=targets,
                elapsed_s=now - experiment_start,
                phase="replay",
                cycle=cycle,
                trajectory_index=trajectory_index,
                trajectory_time_s=bundle.trajectories[
                    bundle.motor_ids[0]
                ].time_s[trajectory_index],
                kp=kp,
                kd=kd,
            )

            next_deadline += control_dt
            if now - next_deadline > control_dt:
                # Preserve every trajectory point, but do not issue several
                # points back-to-back in an attempt to catch up.
                overrun_count += 1
                next_deadline = now + control_dt
                if overrun_count <= 5 or overrun_count % 100 == 0:
                    print(
                        f"[Timing warning] control loop overrun "
                        f"(count={overrun_count})"
                    )

        cycle += 1
        print(f"[Replay] Completed cycle {cycle}")

    return False


def _validate_sdk_mapping(robot, bundle: TrajectoryBundle) -> None:
    sdk_mapping = tuple(int(value) for value in robot.policy_to_motor_id_)
    for motor_id in bundle.motor_ids:
        policy_idx = bundle.trajectories[motor_id].policy_idx
        if not 0 <= policy_idx < len(sdk_mapping):
            raise ValueError(
                f"Invalid policy index {policy_idx} for motor {motor_id}"
            )
        if sdk_mapping[policy_idx] != motor_id:
            raise ValueError(
                f"Trajectory/SDK mapping mismatch at policy index {policy_idx}: "
                f"trajectory motor={motor_id}, SDK motor={sdk_mapping[policy_idx]}"
            )


def _default_output_dir(leg: str) -> Path:
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return DEFAULT_OUTPUT_ROOT / f"{timestamp}_{LEG_NAMES[leg]}"


def run_hardware_experiment(
    args: argparse.Namespace,
    bundle: TrajectoryBundle,
    control_dt: float,
) -> Path:
    # Keep validation usable on machines without pysoem by importing the
    # hardware SDK only when an actual experiment is requested.
    from config_hexapod_tethered import Config
    from hexapod_tethered_utils.joystick_reader import Gamepad
    from motor_igh_sdk.deploy_real_el4090_pysoem import RL_Real_PySOEM

    output_dir = (
        args.output_dir.resolve()
        if args.output_dir is not None
        else _default_output_dir(bundle.leg).resolve()
    )
    robot = None
    gamepad = None
    logger: Optional[ExperimentLogger] = None
    started = False

    try:
        gamepad = Gamepad(joystick_id=args.joystick_id)
        if getattr(gamepad, "_joystick", None) is None:
            raise RuntimeError(
                f"No gamepad connected at joystick ID {args.joystick_id}; "
                "EtherCAT was not started"
            )

        robot = RL_Real_PySOEM(args.ifname)
        _validate_sdk_mapping(robot, bundle)
        config = Config(
            str(SCRIPT_DIR / "configs" / "hexapod_tethered.yaml")
        )
        initial_targets = {
            motor_id: float(
                config.default_angles[bundle.trajectories[motor_id].policy_idx]
            )
            for motor_id in bundle.motor_ids
        }

        # The process-data thread sees damping commands from its first cycle.
        set_all_damping(robot, args.damping_kd)
        if not robot.start():
            raise RuntimeError("Failed to start EtherCAT")
        started = True

        logger = ExperimentLogger(
            output_dir=output_dir,
            bundle=bundle,
            flush_every=args.flush_every,
        )
        print(f"[Logging] Output directory: {output_dir}")
        print(
            f"[Control] {LEG_NAMES[bundle.leg]} motors "
            f"{bundle.motor_ids}; all other motors remain in damping."
        )
        print(
            "[Control] Configured initial targets (rad): "
            + ", ".join(
                f"motor {motor_id:02d}={initial_targets[motor_id]:.6f}"
                for motor_id in bundle.motor_ids
            )
        )

        time.sleep(0.2)
        if not robot.running:
            raise RuntimeError("EtherCAT process-data thread is not running")

        _wait_for_y(
            robot=robot,
            gamepad=gamepad,
            control_dt=control_dt,
            damping_kd=args.damping_kd,
        )
        experiment_start = time.perf_counter()
        while True:
            _move_to_initial(
                robot=robot,
                gamepad=gamepad,
                bundle=bundle,
                initial_targets=initial_targets,
                logger=logger,
                experiment_start=experiment_start,
                ramp_seconds=args.ramp_seconds,
                control_dt=control_dt,
                kp=args.kp,
                kd=args.kd,
                damping_kd=args.damping_kd,
            )
            _hold_initial_until_a(
                robot=robot,
                gamepad=gamepad,
                bundle=bundle,
                initial_targets=initial_targets,
                logger=logger,
                experiment_start=experiment_start,
                control_dt=control_dt,
                kp=args.kp,
                kd=args.kd,
                damping_kd=args.damping_kd,
            )
            y_requested = _run_replay_loop(
                robot=robot,
                gamepad=gamepad,
                bundle=bundle,
                logger=logger,
                experiment_start=experiment_start,
                control_dt=control_dt,
                kp=args.kp,
                kd=args.kd,
                damping_kd=args.damping_kd,
                cycles=args.cycles,
            )
            if not y_requested:
                break
    except DampingExitRequested as exc:
        print(f"\n[Stop] {exc}.")
    except KeyboardInterrupt:
        print("\n[Stop] Ctrl+C received.")
    finally:
        if robot is not None:
            print("[Safety] Setting all 18 joints to damping mode...")
            set_all_damping(robot, args.damping_kd)
        if logger is not None:
            logger.flush()
            logger.close()
        if robot is not None:
            if started and robot.running:
                time.sleep(0.2)
            robot.stop()
        if gamepad is not None:
            try:
                gamepad.stop()
            except Exception:
                pass
        if logger is not None:
            print(f"[Logging] CSV files saved in: {output_dir}")
        else:
            print("[Logging] No output CSV was created.")

    return output_dir


def _positive_float(value: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed) or parsed <= 0.0:
        raise argparse.ArgumentTypeError("value must be a finite number > 0")
    return parsed


def _nonnegative_float(value: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed) or parsed < 0.0:
        raise argparse.ArgumentTypeError("value must be a finite number >= 0")
    return parsed


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Replay the recorded trajectory on exactly one rear leg while "
            "keeping the other 15 motors in damping mode."
        )
    )
    parser.add_argument(
        "leg",
        choices=tuple(LEG_MOTOR_IDS),
        help="rb: right-back motors 1/3/2; lb: left-back motors 4/6/5",
    )
    parser.add_argument("--ifname", default="enp86s0", help="EtherCAT interface")
    parser.add_argument(
        "--joystick-id",
        type=int,
        default=0,
        help="pygame joystick ID (default: 0)",
    )
    parser.add_argument(
        "--trajectory-dir",
        type=Path,
        default=DEFAULT_TRAJECTORY_DIR,
        help=f"source trajectory directory (default: {DEFAULT_TRAJECTORY_DIR})",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help=(
            "exact output directory; by default a timestamped directory is "
            f"created under {DEFAULT_OUTPUT_ROOT}"
        ),
    )
    parser.add_argument(
        "--control-dt",
        type=_positive_float,
        default=None,
        help="playback period in seconds (default: derive from source CSV)",
    )
    parser.add_argument("--kp", type=_nonnegative_float, default=DEFAULT_KP)
    parser.add_argument("--kd", type=_nonnegative_float, default=DEFAULT_KD)
    parser.add_argument(
        "--damping-kd",
        type=_nonnegative_float,
        default=DEFAULT_DAMPING_KD,
        help="KD for all non-controlled motors (default: 5.0)",
    )
    parser.add_argument(
        "--ramp-seconds",
        type=_nonnegative_float,
        default=2.0,
        help="smooth transition duration to the first target (default: 2.0)",
    )
    parser.add_argument(
        "--cycles",
        type=int,
        default=0,
        help="number of complete cycles; 0 means repeat until Ctrl+C",
    )
    parser.add_argument(
        "--flush-every",
        type=int,
        default=50,
        help="flush each output CSV every N samples (default: 50)",
    )
    parser.add_argument(
        "--validate-only",
        action="store_true",
        help="validate inputs and print the mapping without accessing hardware",
    )
    return parser


def _validate_control_args(parser: argparse.ArgumentParser, args) -> None:
    if not 0.0 <= args.kp <= 500.0:
        parser.error("--kp must be in the SDK range [0, 500]")
    if not 0.0 <= args.kd <= 5.0:
        parser.error("--kd must be in the SDK range [0, 5]")
    if not 0.0 <= args.damping_kd <= 5.0:
        parser.error("--damping-kd must be in the SDK range [0, 5]")
    if args.cycles < 0:
        parser.error("--cycles must be >= 0")
    if args.flush_every < 1:
        parser.error("--flush-every must be >= 1")
    if args.joystick_id < 0:
        parser.error("--joystick-id must be >= 0")


def _print_bundle_summary(
    bundle: TrajectoryBundle,
    control_dt: float,
) -> None:
    print(f"Leg: {LEG_NAMES[bundle.leg]}")
    print(f"Source sample count: {bundle.sample_count}")
    print(f"Source dt: {bundle.source_dt_s:.9f} s")
    print(f"Playback dt: {control_dt:.9f} s")
    print(
        f"Playback cycle duration: "
        f"{bundle.sample_count * control_dt:.3f} s"
    )
    print("Controlled motor mapping:")
    for motor_id in bundle.motor_ids:
        trajectory = bundle.trajectories[motor_id]
        print(
            f"  motor {motor_id:02d} -> policy_idx "
            f"{trajectory.policy_idx:02d} <- {trajectory.source_path.name}"
        )


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = build_arg_parser()
    args = parser.parse_args(argv)
    _validate_control_args(parser, args)

    try:
        bundle = load_trajectory_bundle(args.trajectory_dir, args.leg)
    except (FileNotFoundError, ValueError) as exc:
        parser.error(str(exc))

    control_dt = (
        float(args.control_dt)
        if args.control_dt is not None
        else bundle.source_dt_s
    )
    _print_bundle_summary(bundle, control_dt)

    if args.validate_only:
        print("Validation passed; hardware was not accessed.")
        return 0

    run_hardware_experiment(args, bundle, control_dt)
    return 0


if __name__ == "__main__":
    sys.exit(main())
