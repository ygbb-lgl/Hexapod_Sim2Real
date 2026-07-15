"""Online candidate-torque selection with the ``pre_tension.py`` model.

The neural network predicts force deltas; it never emits a torque command
directly.  This module owns the causal 64-frame history, evaluates all torque
candidates in one batch, and returns a command that the caller still sends
through the existing hardware interface.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from pathlib import Path
from time import perf_counter
from typing import Deque, Dict, Optional, Sequence, Tuple

import numpy as np
import torch

try:
    from pre_tension import DataConfig, FeatureLayout, ModelConfig, TCNGRUTensionPredictor
except ImportError:  # Package-style import from the repository root.
    from deploy.deploy_real.pre_tension import (
        DataConfig,
        FeatureLayout,
        ModelConfig,
        TCNGRUTensionPredictor,
    )

# true是只读配置
@dataclass(frozen=True)
class TensionSelectorConfig:
    """Deployment-only constants; the YAML contains only the checkpoint path."""

    control_dt_s: float = 0.02
    candidate_count: int = 5
    candidate_hold_steps: int = 5
    candidate_radius_torque_std: float = 2.0
    torque_slew_per_step_std: float = 1.0
    minimum_torque_slew_nm: float = 0.5
    # The training script did not save exact min/max coverage.  Restrict shadow
    # evaluation to the central normalizer range instead of searching arbitrarily
    # far outside the data distribution.
    training_coverage_std: float = 3.0
    force_scale_n: float = 50.0
    lambda_high: float = 1.0
    lambda_low: float = 1.0
    lambda_delta_torque: float = 0.05
    lambda_base: float = 0.02
    max_inference_ms: float = 18.0
    timestamp_tolerance: float = 0.35
    duplicate_tolerance_nm: float = 1e-5


def _clip(value: float, low: float, high: float) -> float:
    return low if value < low else high if value > high else value


class LearnedTensionTorqueSelector:
    """Causal, batched short-horizon tension predictor and torque selector.

    ``step`` receives only values already available at the current control
    instant.  The caller must pass ``torque_command_prev_nm`` as the command
    that was actually sent over the preceding interval.
    """

    def __init__(
        self,
        checkpoint_path: str,
        *,
        config: Optional[TensionSelectorConfig] = None,
    ) -> None:
        self.cfg = config or TensionSelectorConfig()
        self.device = torch.device("cpu")
        path = Path(checkpoint_path)
        if not path.is_file():
            raise FileNotFoundError(f"tension predictor checkpoint not found: {path}")

        try:
            checkpoint = torch.load(path, map_location=self.device, weights_only=True)
        except TypeError:  # PyTorch versions before weights_only support.
            checkpoint = torch.load(path, map_location=self.device)

        required = {
            "model_state_dict", "data_config", "model_config", "normalization",
            "feature_names", "preview_names",
        }
        missing = sorted(required - set(checkpoint))
        if missing:
            raise ValueError(f"checkpoint is missing required fields: {missing}")

        self.data_cfg = DataConfig(**checkpoint["data_config"])
        self.model_cfg = ModelConfig(**checkpoint["model_config"])
        self.feature_names = list(checkpoint["feature_names"])
        self.preview_names = list(checkpoint["preview_names"])

        # 检查输入维度之类的是否和pt一样
        self._validate_checkpoint_layout()

        self.model = TCNGRUTensionPredictor(self.data_cfg, self.model_cfg).to(self.device)
        self.model.load_state_dict(checkpoint["model_state_dict"], strict=True)
        self.model.eval()

        normalization = checkpoint["normalization"]
        self.history_mean = np.asarray(normalization["history_mean"], dtype=np.float32)
        self.history_std = np.asarray(normalization["history_std"], dtype=np.float32)
        self.preview_mean = np.asarray(normalization["preview_mean"], dtype=np.float32)
        self.preview_std = np.asarray(normalization["preview_std"], dtype=np.float32)
        self.torque_mean = float(normalization["torque_mean"])
        self.torque_std = float(normalization["torque_std"])
        self._validate_normalization()

        self._history: Deque[np.ndarray] = deque(maxlen=self.data_cfg.history_len)
        self._queue_len = max(self.data_cfg.action_delay_steps - 1, 0)
        self._delay_queue: Deque[float] = deque(maxlen=self._queue_len)
        self._last_timestamp_ns: Optional[int] = None
        self._last_sent_torque_nm: Optional[float] = None
        self.last_debug: Dict[str, object] = {}

        self.training_torque_min_nm = (
            self.torque_mean - self.cfg.training_coverage_std * self.torque_std
        )
        self.training_torque_max_nm = (
            self.torque_mean + self.cfg.training_coverage_std * self.torque_std
        )
        self._warm_up_model()

        print(
            f"[TensionSelector] loaded {path}; history={self.data_cfg.history_len}x"
            f"{len(self.feature_names)}, horizon={self.data_cfg.horizon}, "
            f"preview={len(self.preview_names)}, delay={self.data_cfg.action_delay_steps}"
        )

    @property
    def ready(self) -> bool:
        return len(self._history) == self.data_cfg.history_len

    @property
    def history_size(self) -> int:
        return len(self._history)

    def reset(self, *, initial_torque_nm: float = 0.0) -> None:
        """Discard stale temporal state after stops, mode switches, or time gaps."""

        initial = float(initial_torque_nm)
        self._history.clear()
        self._delay_queue.clear()
        for _ in range(self._queue_len):
            self._delay_queue.append(initial)
        self._last_timestamp_ns = None
        self._last_sent_torque_nm = initial
        self.last_debug = {}

    def _validate_checkpoint_layout(self) -> None:
        c = self.data_cfg
        expected = {
            "history_len": 64,
            "horizon": 5,
            "velocity_dim": 3,
            "policy_action_dim": 18,
            "joint_dim": 18,
        }
        actual = {name: getattr(c, name) for name in expected}
        if actual != expected:
            raise ValueError(f"incompatible tension checkpoint data_config: {actual}; expected {expected}")
        if abs(float(c.expected_dt_s) - self.cfg.control_dt_s) > 1e-9:
            raise ValueError(
                f"checkpoint dt={c.expected_dt_s}, deployment dt={self.cfg.control_dt_s}"
            )
        if not (c.use_actual_torque and c.use_body_linear_acceleration and c.use_policy_action):
            raise ValueError("checkpoint disabled one or more required 71-D history features")
        if c.action_delay_steps < 1:
            raise ValueError("action_delay_steps must be at least 1")

        raw_names = FeatureLayout(c).names
        filtered_names = [
            "force_filtered_n" if name == "force_raw_n" else
            name.replace("body_linear_acceleration_", "body_linear_acceleration_filtered_")
            if name.startswith("body_linear_acceleration_") else name
            for name in raw_names
        ]
        if self.feature_names not in (raw_names, filtered_names):
            raise ValueError(
                "checkpoint feature_names do not match either supported pre_tension.py "
                f"71-D layout: {self.feature_names}"
            )

        expected_preview = (
            ["force_reference_hold"]
            + [f"velocity_command_hold_{i}" for i in range(3)]
            + [f"policy_action_hold_{i}" for i in range(18)]
        )
        if self.preview_names != expected_preview:
            raise ValueError(
                f"checkpoint preview_names mismatch: {self.preview_names}; "
                f"expected {expected_preview}"
            )

    def _validate_normalization(self) -> None:
        expected_history_dim = len(self.feature_names)
        expected_preview_dim = len(self.preview_names)
        if self.history_mean.shape != (expected_history_dim,):
            raise ValueError(f"history_mean shape is {self.history_mean.shape}")
        if self.history_std.shape != (expected_history_dim,):
            raise ValueError(f"history_std shape is {self.history_std.shape}")
        if self.preview_mean.shape != (expected_preview_dim,):
            raise ValueError(f"preview_mean shape is {self.preview_mean.shape}")
        if self.preview_std.shape != (expected_preview_dim,):
            raise ValueError(f"preview_std shape is {self.preview_std.shape}")
        arrays = (self.history_mean, self.history_std, self.preview_mean, self.preview_std)
        if any(not np.all(np.isfinite(array)) for array in arrays):
            raise ValueError("checkpoint normalizer contains NaN or Inf")
        if np.any(self.history_std <= 0.0) or np.any(self.preview_std <= 0.0):
            raise ValueError("checkpoint normalizer contains non-positive std")
        if not np.isfinite(self.torque_mean) or not np.isfinite(self.torque_std):
            raise ValueError("checkpoint torque normalizer contains NaN or Inf")
        if self.torque_std <= 0.0:
            raise ValueError("checkpoint torque_std must be positive")

    def _warm_up_model(self) -> None:
        batch = self.cfg.candidate_count
        history = torch.zeros(
            (1, self.data_cfg.history_len, len(self.feature_names)), dtype=torch.float32
        )
        torque = torch.zeros((batch, self.data_cfg.horizon, 1), dtype=torch.float32)
        preview = torch.zeros(
            (batch, self.data_cfg.horizon, len(self.preview_names)), dtype=torch.float32
        )
        queue = torch.zeros((batch, self._queue_len, 1), dtype=torch.float32)
        with torch.inference_mode():
            z_k = self.model.encode_history(history)
            self.model.rollout_from_latent(z_k.expand(batch, -1), torque, preview, queue)

    @staticmethod
    def _vector(value: Sequence[float], size: int, name: str) -> np.ndarray:
        array = np.asarray(value, dtype=np.float32).reshape(-1)
        if array.shape != (size,):
            raise ValueError(f"{name} must have shape ({size},), got {array.shape}")
        return array

    def _append_history(
        self,
        *,
        model_force_n: float,
        torque_actual_nm: float,
        torque_command_prev_nm: float,
        motor_position_rad: float,
        force_reference_n: float,
        velocity_command: np.ndarray,
        joint_position: np.ndarray,
        joint_velocity: np.ndarray,
        body_gravity: np.ndarray,
        body_angular_velocity: np.ndarray,
        body_linear_acceleration: np.ndarray,
        scaled_policy_action: np.ndarray,
    ) -> None:
        # Position slot 3 temporarily stores the absolute value.  It is changed
        # to a window-relative value only on the inference copy.
        frame = np.concatenate(
            (
                np.asarray(
                    [model_force_n, torque_actual_nm, torque_command_prev_nm,
                     motor_position_rad, force_reference_n],
                    dtype=np.float32,
                ),
                velocity_command,
                body_gravity,
                body_angular_velocity,
                body_linear_acceleration,
                joint_position,
                joint_velocity,
                scaled_policy_action,
            )
        )
        if frame.shape != (len(self.feature_names),):
            raise RuntimeError(f"constructed history frame has shape {frame.shape}")
        self._history.append(frame)

    # 记录卷扬机的相对转角
    def _normalized_history(self) -> np.ndarray:
        history = np.stack(tuple(self._history), axis=0).astype(np.float32, copy=True)
        history[:, 3] -= history[0, 3]
        history -= self.history_mean
        history /= self.history_std
        return history

    # 候选力矩的选取范围
    def _candidate_interval(
        self,
        baseline_torque_nm: float,
        previous_torque_nm: float,
        hardware_min_nm: float,
        hardware_max_nm: float,
    ) -> Tuple[float, float, float, float]:
        radius = max(self.cfg.candidate_radius_torque_std * self.torque_std, 1e-6)
        slew = max(
            self.cfg.torque_slew_per_step_std * self.torque_std,
            self.cfg.minimum_torque_slew_nm,
        )
        lower = max(
            hardware_min_nm,
            self.training_torque_min_nm,
            baseline_torque_nm - radius,
            previous_torque_nm - slew,
        )
        upper = min(
            hardware_max_nm,
            self.training_torque_max_nm,
            baseline_torque_nm + radius,
            previous_torque_nm + slew,
        )
        return float(lower), float(upper), float(radius), float(slew)

    # 生成候选力矩的列表
    def _generate_candidates(
        self,
        lower: float,
        upper: float,
        baseline_torque_nm: float,
        previous_torque_nm: float,
    ) -> np.ndarray:
        if lower > upper:
            return np.empty(0, dtype=np.float32)
        if abs(upper - lower) <= self.cfg.duplicate_tolerance_nm:
            return np.asarray([0.5 * (lower + upper)], dtype=np.float32)

        anchors = [
            lower,
            _clip(baseline_torque_nm, lower, upper),
            _clip(previous_torque_nm, lower, upper),
            0.5 * (lower + upper),
            upper,
        ]
        candidates = []
        for value in sorted(anchors):
            if not candidates or abs(value - candidates[-1]) > self.cfg.duplicate_tolerance_nm:
                candidates.append(float(value))

        while len(candidates) < self.cfg.candidate_count:
            gaps = np.diff(candidates)
            if gaps.size == 0 or float(gaps.max()) <= self.cfg.duplicate_tolerance_nm:
                break
            index = int(np.argmax(gaps))
            candidates.insert(index + 1, 0.5 * (candidates[index] + candidates[index + 1]))

        if len(candidates) > self.cfg.candidate_count:
            # Five physical anchors are the maximum for the current controller.
            candidates = candidates[:self.cfg.candidate_count]
        return np.asarray(candidates, dtype=np.float32)

    def _record_sent(self, torque_nm: float) -> None:
        sent = float(torque_nm)
        if self._queue_len:
            self._delay_queue.append(sent)
        self._last_sent_torque_nm = sent

    def _finish(
        self,
        debug: Dict[str, object],
        *,
        tau_sent: float,
        start_time: float,
        fallback_reason: str = "",
        clear_history: bool = False,
    ) -> Tuple[float, Dict[str, object]]:
        sent = float(tau_sent)
        if clear_history:
            self._history.clear()
        self._record_sent(sent)
        debug["tau_sent"] = sent
        debug["fallback_active"] = bool(fallback_reason)
        debug["fallback_reason"] = fallback_reason
        debug["history_size"] = len(self._history)
        debug["total_latency_ms"] = (perf_counter() - start_time) * 1000.0
        self.last_debug = debug
        return sent, debug

    def step(
        self,
        *,
        timestamp_ns: int,
        force_n: float,
        torque_actual_nm: float,
        torque_command_prev_nm: float,
        motor_position_rad: float,
        joint_position: Sequence[float],
        joint_velocity: Sequence[float],
        body_gravity: Sequence[float],
        body_angular_velocity: Sequence[float],
        body_linear_acceleration: Sequence[float],
        velocity_command: Sequence[float],
        scaled_policy_action: Sequence[float],
        force_reference_n: float,
        baseline_torque_nm: float,
        hardware_torque_min_nm: float,
        hardware_torque_max_nm: float,
        alpha: float,
        shadow_mode: bool,
        data_valid: bool,
        emergency: bool = False,
        force_lower_bound_n: Optional[float] = None,
        force_upper_bound_n: Optional[float] = None,
    ) -> Tuple[float, Dict[str, object]]:
        """Append the current causal frame, predict candidates, and select torque."""

        started = perf_counter()
        previous = float(torque_command_prev_nm)
        hardware_low = float(min(hardware_torque_min_nm, hardware_torque_max_nm))
        hardware_high = float(max(hardware_torque_min_nm, hardware_torque_max_nm))
        baseline = _clip(float(baseline_torque_nm), hardware_low, hardware_high)
        shadow_enabled = bool(shadow_mode)
        takeover_alpha = _clip(float(alpha), 0.0, 1.0)

        model_force = float(force_n)
        debug: Dict[str, object] = {
            "ready": False,
            "shadow_mode": shadow_enabled,
            "alpha": takeover_alpha,
            "current_force": model_force,
            "force_feature_name": self.feature_names[0],
            "force_reference": float(force_reference_n),
            "tau_base": baseline,
            "tau_prev": previous,
            "tau_model": baseline,
            "tau_blended": baseline,
            "candidate_lower_bound": None,
            "candidate_upper_bound": None,
            "candidates": [],
            "predicted_delta_force": [],
            "predicted_force": [],
            "tracking_cost": [],
            "high_force_cost": [],
            "low_force_cost": [],
            "delta_torque_cost": [],
            "base_deviation_cost": [],
            "total_cost": [],
            "best_index": None,
            "action_delay_steps": self.data_cfg.action_delay_steps,
            "encode_latency_ms": 0.0,
            "rollout_latency_ms": 0.0,
            "training_torque_min_estimate_nm": self.training_torque_min_nm,
            "training_torque_max_estimate_nm": self.training_torque_max_nm,
        }

        scalar_values = np.asarray(
            [force_n, torque_actual_nm, previous, motor_position_rad,
             force_reference_n, baseline, hardware_low, hardware_high, model_force],
            dtype=np.float64,
        )
        if not np.all(np.isfinite(scalar_values)) or hardware_low > hardware_high:
            return self._finish(
                debug, tau_sent=_clip(previous if np.isfinite(previous) else 0.0,
                                      hardware_low, hardware_high),
                start_time=started, fallback_reason="invalid_scalar_input", clear_history=True,
            )
        if emergency:
            return self._finish(
                debug, tau_sent=baseline, start_time=started,
                fallback_reason="emergency", clear_history=True,
            )
        if not data_valid:
            return self._finish(
                debug, tau_sent=baseline, start_time=started,
                fallback_reason="invalid_sensor_input", clear_history=True,
            )
        try:
            q = self._vector(joint_position, 18, "joint_position")
            dq = self._vector(joint_velocity, 18, "joint_velocity")
            gravity = self._vector(body_gravity, 3, "body_gravity")
            angular_velocity = self._vector(body_angular_velocity, 3, "body_angular_velocity")
            linear_acceleration = self._vector(
                body_linear_acceleration, 3, "body_linear_acceleration"
            )
            command = self._vector(velocity_command, 3, "velocity_command")
            policy_action = self._vector(scaled_policy_action, 18, "scaled_policy_action")
        except ValueError as exc:
            debug["input_error"] = str(exc)
            return self._finish(
                debug, tau_sent=baseline, start_time=started,
                fallback_reason="invalid_vector_shape", clear_history=True,
            )
        vectors = (q, dq, gravity, angular_velocity, linear_acceleration, command, policy_action)
        if any(not np.all(np.isfinite(vector)) for vector in vectors):
            return self._finish(
                debug, tau_sent=baseline, start_time=started,
                fallback_reason="nonfinite_vector_input", clear_history=True,
            )

        timestamp = int(timestamp_ns)
        if self._last_timestamp_ns is not None:
            dt_s = (timestamp - self._last_timestamp_ns) * 1e-9
            tolerance = self.cfg.timestamp_tolerance
            if not (self.cfg.control_dt_s * (1.0 - tolerance)
                    <= dt_s <= self.cfg.control_dt_s * (1.0 + tolerance)):
                self._history.clear()
                debug["history_reset_dt_s"] = dt_s
        self._last_timestamp_ns = timestamp

        if self._last_sent_torque_nm is not None and abs(
            previous - self._last_sent_torque_nm
        ) > 1e-4:
            # An external safety layer changed the command.  Synchronize the
            # delay pipeline and rebuild history from subsequent real samples.
            self._history.clear()
            self._delay_queue.clear()
            for _ in range(self._queue_len):
                self._delay_queue.append(previous)
            debug["external_torque_override_detected"] = True

        self._append_history(
            model_force_n=model_force,
            torque_actual_nm=float(torque_actual_nm),
            torque_command_prev_nm=previous,
            motor_position_rad=float(motor_position_rad),
            force_reference_n=float(force_reference_n),
            velocity_command=command,
            joint_position=q,
            joint_velocity=dq,
            body_gravity=gravity,
            body_angular_velocity=angular_velocity,
            body_linear_acceleration=linear_acceleration,
            scaled_policy_action=policy_action,
        )
        debug["history_size"] = len(self._history)
        if not self.ready:
            return self._finish(
                debug, tau_sent=baseline, start_time=started,
                fallback_reason="history_warmup",
            )

        lower, upper, radius, slew = self._candidate_interval(
            baseline, previous, hardware_low, hardware_high
        )
        debug["candidate_lower_bound"] = lower
        debug["candidate_upper_bound"] = upper
        debug["candidate_radius_nm"] = radius
        debug["torque_slew_per_step_nm"] = slew
        if lower > upper:
            return self._finish(
                debug, tau_sent=baseline, start_time=started,
                fallback_reason="candidate_interval_invalid",
            )
        candidates = self._generate_candidates(lower, upper, baseline, previous)
        if candidates.size == 0:
            return self._finish(
                debug, tau_sent=baseline, start_time=started,
                fallback_reason="no_candidates",
            )
        debug["candidates"] = candidates.tolist()

        history = self._normalized_history()
        preview_values = np.concatenate(
            (np.asarray([force_reference_n], dtype=np.float32), command, policy_action)
        )
        preview = np.repeat(preview_values[None, :], self.data_cfg.horizon, axis=0)
        preview = (preview - self.preview_mean) / self.preview_std

        candidate_count = int(candidates.size)
        future_torque = np.full(
            (candidate_count, self.data_cfg.horizon, 1), baseline, dtype=np.float32
        )
        hold_steps = min(max(self.cfg.candidate_hold_steps, 1), self.data_cfg.horizon)
        future_torque[:, :hold_steps, 0] = candidates[:, None]
        future_torque = (future_torque - self.torque_mean) / self.torque_std

        if self._queue_len:
            if len(self._delay_queue) != self._queue_len:
                return self._finish(
                    debug, tau_sent=baseline, start_time=started,
                    fallback_reason="delay_queue_not_ready", clear_history=True,
                )
            queue = np.asarray(tuple(self._delay_queue), dtype=np.float32)[:, None]
            queue = (queue - self.torque_mean) / self.torque_std
        else:
            queue = np.empty((0, 1), dtype=np.float32)

        history_tensor = torch.from_numpy(history).unsqueeze(0).to(self.device)
        torque_tensor = torch.from_numpy(future_torque).to(self.device)
        preview_tensor = (
            torch.from_numpy(preview).unsqueeze(0).expand(candidate_count, -1, -1).to(self.device)
        )
        queue_tensor = (
            torch.from_numpy(queue).unsqueeze(0).expand(candidate_count, -1, -1).to(self.device)
        )

        try:
            with torch.inference_mode():
                encode_started = perf_counter()
                z_k = self.model.encode_history(history_tensor)
                encode_finished = perf_counter()
                pred_delta_tensor = self.model.rollout_from_latent(
                    z_k.expand(candidate_count, -1),
                    torque_tensor,
                    preview_tensor,
                    queue_tensor,
                )
                rollout_finished = perf_counter()
            pred_delta = pred_delta_tensor.cpu().numpy()
        except Exception as exc:
            debug["inference_error"] = repr(exc)
            return self._finish(
                debug, tau_sent=baseline, start_time=started,
                fallback_reason="model_inference_error",
            )

        encode_ms = (encode_finished - encode_started) * 1000.0
        rollout_ms = (rollout_finished - encode_finished) * 1000.0
        debug["encode_latency_ms"] = encode_ms
        debug["rollout_latency_ms"] = rollout_ms
        if encode_ms + rollout_ms > self.cfg.max_inference_ms:
            return self._finish(
                debug, tau_sent=baseline, start_time=started,
                fallback_reason="model_inference_timeout",
            )
        pred_force = model_force + pred_delta
        if not np.all(np.isfinite(pred_delta)) or not np.all(np.isfinite(pred_force)):
            return self._finish(
                debug, tau_sent=baseline, start_time=started,
                fallback_reason="nonfinite_model_output",
            )

        weights = np.ones(self.data_cfg.horizon, dtype=np.float32)
        if self.data_cfg.action_delay_steps > 1:
            weights[:self.data_cfg.action_delay_steps - 1] = 0.25
        weights /= float(weights.sum())
        force_scale = max(abs(float(self.cfg.force_scale_n)), 1e-6)
        torque_scale = max(abs(self.torque_std), 1e-6)

        tracking = np.sum(
            weights[None, :] * ((pred_force - float(force_reference_n)) / force_scale) ** 2,
            axis=1,
        )
        if force_upper_bound_n is None:
            high = np.zeros(candidate_count, dtype=np.float32)
        else:
            high = self.cfg.lambda_high * np.sum(
                weights[None, :]
                * np.maximum((pred_force - float(force_upper_bound_n)) / force_scale, 0.0) ** 2,
                axis=1,
            )
        if force_lower_bound_n is None:
            low = np.zeros(candidate_count, dtype=np.float32)
        else:
            low = self.cfg.lambda_low * np.sum(
                weights[None, :]
                * np.maximum((float(force_lower_bound_n) - pred_force) / force_scale, 0.0) ** 2,
                axis=1,
            )
        delta_torque = self.cfg.lambda_delta_torque * (
            (candidates - previous) / torque_scale
        ) ** 2
        base_deviation = self.cfg.lambda_base * (
            (candidates - baseline) / torque_scale
        ) ** 2
        total = tracking + high + low + delta_torque + base_deviation
        if not np.any(np.isfinite(total)):
            return self._finish(
                debug, tau_sent=baseline, start_time=started,
                fallback_reason="all_candidate_costs_invalid",
            )

        finite_total = np.where(np.isfinite(total), total, np.inf)
        best_index = int(np.argmin(finite_total))
        tau_model = float(candidates[best_index])
        if shadow_enabled:
            tau_blended = baseline
        else:
            tau_blended = (1.0 - takeover_alpha) * baseline + takeover_alpha * tau_model
        tau_sent = _clip(float(tau_blended), hardware_low, hardware_high)

        debug.update(
            {
                "ready": True,
                "predicted_delta_force": pred_delta.tolist(),
                "predicted_force": pred_force.tolist(),
                "tracking_cost": tracking.tolist(),
                "high_force_cost": high.tolist(),
                "low_force_cost": low.tolist(),
                "delta_torque_cost": delta_torque.tolist(),
                "base_deviation_cost": base_deviation.tolist(),
                "total_cost": total.tolist(),
                "best_index": best_index,
                "tau_model": tau_model,
                "tau_blended": float(tau_blended),
            }
        )
        saturated = abs(tau_sent - hardware_low) <= 1e-6 or abs(tau_sent - hardware_high) <= 1e-6
        return self._finish(
            debug,
            tau_sent=tau_sent,
            start_time=started,
            clear_history=saturated,
        )
