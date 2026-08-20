"""Train an action-conditioned short-horizon tether-tension predictor.

CSV INPUT FORMAT (one row per configured-rate *pre-action* observation)
======================================================================
The CSV must be sorted by ``trajectory_id, timestamp_ns`` (the loader sorts and
validates it again).  A vector ``name`` of length D is stored as D scalar columns
``name_0 ... name_{D-1}``.  Do not put Python lists or JSON strings in cells.

Required scalar columns::

    timestamp_ns, trajectory_id,
    force_raw_n, torque_actual_nm,
    torque_command_prev_nm, torque_command_issued_nm,
    motor_position_rad, force_reference_n,
    saturation_flag, emergency_flag, data_valid_flag

Required vector columns (default dimensions shown)::

    velocity_command_0 ... velocity_command_{velocity_dim-1}     (default 3)
    policy_action_0 ... policy_action_17                          (18,
        already multiplied by action_scale; physical position increment)
    joint_position_0 ... joint_position_17                        (18)
    joint_velocity_0 ... joint_velocity_17                        (18)
    body_gravity_vector_0 ... body_gravity_vector_2               (3)
    body_angular_velocity_0 ... body_angular_velocity_2           (3)
    body_linear_acceleration_0 ... _2                             (3)

Optional drive fields may be present but are not used by this first-stage model.
Missing/NaN required values make a row invalid; windows that contain invalid,
emergency, or saturated rows are rejected.  A training window never crosses a
trajectory boundary.

Strict timing of each CSV row k::

    torque_command_prev_nm   = u_{k-1}, already executed in [t_{k-1}, t_k)
    force_raw_n              = F_k, measured at t_k
    torque_command_issued_nm = u_k, computed after observing F_k

Thus a sample centered at k contains history [o_{k-N+1},...,o_k], future action
[u_k,...,u_{k+H-1}], and target force [F_{k+1},...,F_{k+H}].  Never copy the
issued command into the previous-command column of the same row.

Future policy preview is deliberately built only from row k: F_ref, velocity
command and the scaled policy action are zero-order-held. Logged future policy
outputs are never read into preview.

Inspect and train from the repository root (current logger is 50 Hz)::

    conda run -n isaacgym python deploy/deploy_real/pre_tension.py inspect-csv \
        --csv deploy/deploy_real/pre_data/merged_training_data.csv \
        --expected-dt-s 0.02

    conda run -n isaacgym python deploy/deploy_real/pre_tension.py train \
        --csv deploy/deploy_real/pre_data/merged_training_data_0729_model9500.csv \
        --output deploy/pre_train/hexapod_tethered/tension_model_0729_model9500.pt \
        --expected-dt-s 0.02

For a true 250 Hz dataset, use ``--expected-dt-s 0.004`` instead.  Do not
train 50 Hz data while labeling it as 250 Hz.

The checkpoint contains the model, configuration, train-only normalization,
trajectory split and metrics needed by later deployment code.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import random
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

import numpy as np
import torch
from torch import Tensor, nn
from torch.nn import functional as F
from torch.utils.data import DataLoader, Dataset

# 写了这个@自动生成init
@dataclass
class DataConfig:
    history_len: int = 64
    horizon: int = 5
    velocity_dim: int = 3
    policy_action_dim: int = 18
    joint_dim: int = 18
    action_delay_steps: int = 1
    expected_dt_s: float = 0.02
    timestamp_tolerance: float = 0.35
    use_body_linear_acceleration: bool = True
    use_actual_torque: bool = True
    use_policy_action: bool = True


@dataclass
class ModelConfig:
    frame_embed_dim: int = 32
    tcn_channels: int = 32
    latent_dim: int = 64
    gru_hidden_dim: int = 64
    kernel_size: int = 3
    group_hidden_dim: int = 32


@dataclass
class TrainConfig:
    seed: int = 7
    epochs: int = 200
    batch_size: int = 4096
    learning_rate: float = 3e-4
    weight_decay: float = 1e-5
    grad_clip_norm: float = 1.0
    num_workers: int = 0
    val_fraction: float = 0.15
    test_fraction: float = 0.15
    patience: int = 100
    huber_delta: float = 1.0
    lambda_force: float = 1.0
    lambda_trend: float = 0.2
    lambda_peak: float = 0.1
    lambda_minimum: float = 0.1
    lambda_nonnegative: float = 0.01
    horizon_weights: Optional[List[float]] = None


class RunningStats:
    """Numerically stable vector statistics, fit on training trajectories only."""

    def __init__(self, dim: int) -> None:
        self.n = 0
        self.mean = np.zeros(dim, np.float64)
        self.m2 = np.zeros(dim, np.float64)

    def update(self, x: np.ndarray) -> None:
        x = np.asarray(x, np.float64).reshape(-1, self.mean.size)
        if not len(x):
            return
        batch_n = len(x)
        batch_mean = x.mean(0)
        batch_m2 = ((x - batch_mean) ** 2).sum(0)
        delta = batch_mean - self.mean
        total = self.n + batch_n
        self.mean += delta * batch_n / total
        self.m2 += batch_m2 + delta**2 * self.n * batch_n / total
        self.n = total

    def result(self) -> Tuple[np.ndarray, np.ndarray]:
        var = self.m2 / max(self.n - 1, 1)
        std = np.sqrt(np.maximum(var, 1e-12))
        std[std < 1e-6] = 1.0
        return self.mean.astype(np.float32), std.astype(np.float32)


@dataclass
class Normalization:
    history_mean: np.ndarray
    history_std: np.ndarray
    preview_mean: np.ndarray
    preview_std: np.ndarray
    torque_mean: float
    torque_std: float

    def state_dict(self) -> Dict[str, object]:
        return {k: (v.tolist() if isinstance(v, np.ndarray) else v)
                for k, v in self.__dict__.items()}


def _vector_columns(name: str, dim: int) -> List[str]:
    return [f"{name}_{i}" for i in range(dim)]


class FeatureLayout:
    """Defines deterministic group slices in the flattened history frame."""

    def __init__(self, cfg: DataConfig) -> None:
        groups: Dict[str, List[str]] = {
            "tether": ["force_raw_n"],
            "body": _vector_columns("velocity_command", cfg.velocity_dim)
                    + _vector_columns("body_gravity_vector", 3)
                    + _vector_columns("body_angular_velocity", 3),
            "leg": _vector_columns("joint_position", cfg.joint_dim)
                   + _vector_columns("joint_velocity", cfg.joint_dim),
        }
        if cfg.use_actual_torque:
            groups["tether"].append("torque_actual_nm")
        groups["tether"] += ["torque_command_prev_nm",
                             "motor_position_relative", "force_reference_n"]
        if cfg.use_body_linear_acceleration:
            groups["body"] += _vector_columns("body_linear_acceleration", 3)
        if cfg.use_policy_action:
            groups["leg"] += _vector_columns("policy_action", cfg.policy_action_dim)
        self.groups = groups
        self.names = sum(groups.values(), [])
        offset = 0
        self.slices: Dict[str, slice] = {}
        for name, fields in groups.items():
            self.slices[name] = slice(offset, offset + len(fields))
            offset += len(fields)
        self.obs_dim = offset


# 读取CSV文件，按trajectory_id分组，并按时间戳排序
class CsvTrajectories:
    REQUIRED_SCALARS = [
        "timestamp_ns", "trajectory_id", "force_raw_n", "torque_actual_nm",
        "torque_command_prev_nm", "torque_command_issued_nm", "motor_position_rad",
        "force_reference_n",
        "saturation_flag", "emergency_flag", "data_valid_flag",
    ]

    def __init__(self, path: Path, cfg: DataConfig) -> None:
        self.cfg = cfg
        layout = FeatureLayout(cfg)
        required = set(self.REQUIRED_SCALARS)
        # Derived/model-only fields are excluded here.
        required.update(n for n in layout.names if n != "motor_position_relative")
        required.update(_vector_columns("joint_position", cfg.joint_dim))
        with path.open("r", newline="") as f:
            reader = csv.DictReader(f)
            present = set(reader.fieldnames or [])
            aliases = {
                f"body_linear_acceleration_{i}": f"body_linear_acceleration_filtered_{i}"
                for i in range(3)
            }
            missing = sorted(
                key for key in required
                if key not in present and aliases.get(key) not in present
            )
            if missing:
                raise ValueError(f"CSV missing {len(missing)} required columns: {missing}")
            rows = list(reader)
        if not rows:
            raise ValueError("CSV is empty")
        by_id: Dict[str, List[dict]] = {}
        numeric = sorted(required - {"trajectory_id"})
        for line, row in enumerate(rows, 2):
            try:
                parsed = {}
                for key in numeric:
                    if key == "timestamp_ns":
                        continue
                    source = key if key in row else aliases.get(key, key)
                    value = row[source]
                    # A logger can be stopped while writing its final row. Keep
                    # that row as invalid so windows touching it are rejected,
                    # matching the documented missing/NaN behavior.
                    parsed[key] = (
                        float("nan")
                        if value is None or not value.strip()
                        else float(value)
                    )
                timestamp = row["timestamp_ns"]
                if timestamp is None or not timestamp.strip():
                    raise ValueError("empty timestamp_ns")
                parsed["timestamp_ns"] = int(float(timestamp))
                trajectory_id = row["trajectory_id"]
                if trajectory_id is None or not trajectory_id.strip():
                    raise ValueError("empty trajectory_id")
                parsed["trajectory_id"] = trajectory_id.strip()
            except (ValueError, TypeError) as exc:
                raise ValueError(f"invalid numeric value at CSV line {line}: {exc}") from exc
            parsed["_finite"] = all(np.isfinite(parsed[k]) for k in numeric)
            by_id.setdefault(parsed["trajectory_id"], []).append(parsed)
        self.trajectories: Dict[str, List[dict]] = {}
        for tid, trajectory in by_id.items():
            trajectory.sort(key=lambda r: r["timestamp_ns"])
            self._validate_timestamps(tid, trajectory)
            self.trajectories[tid] = trajectory

    def _validate_timestamps(self, tid: str, rows: List[dict]) -> None:
        ts = np.array([r["timestamp_ns"] for r in rows], np.int64)
        if len(ts) > 1 and np.any(np.diff(ts) <= 0):
            raise ValueError(f"trajectory {tid!r} has duplicate/non-increasing timestamps")


def split_trajectory_ids(ids: Sequence[str], train_cfg: TrainConfig
                         ) -> Tuple[List[str], List[str], List[str]]:
    ids = sorted(ids)
    if len(ids) < 3:
        raise ValueError("at least 3 independent trajectory_id values are required")
    rng = random.Random(train_cfg.seed)
    rng.shuffle(ids)
    n_test = max(1, round(len(ids) * train_cfg.test_fraction))
    n_val = max(1, round(len(ids) * train_cfg.val_fraction))
    if n_test + n_val >= len(ids):
        n_test = n_val = 1
    return ids[n_test + n_val:], ids[n_test:n_test + n_val], ids[:n_test]


# 切训练样本，组和一组历史和未来信息数据
class WindowBuilder:
    def __init__(self, data: CsvTrajectories, cfg: DataConfig) -> None:
        self.data, self.cfg = data, cfg
        self.layout = FeatureLayout(cfg)
        self.preview_names = (["force_reference_hold"]
            + _vector_columns("velocity_command_hold", cfg.velocity_dim)
            + (_vector_columns("policy_action_hold", cfg.policy_action_dim)
               if cfg.use_policy_action else []))
        self.preview_dim = len(self.preview_names)

    def _row_is_valid(self, row: Mapping[str, float]) -> bool:
        return (bool(row["_finite"])
                and bool(row["data_valid_flag"])
                and not bool(row["emergency_flag"])
                and not bool(row["saturation_flag"]))

    def _timing_bounds_ns(self) -> Tuple[float, float]:
        dt_ns = self.cfg.expected_dt_s * 1e9
        return (dt_ns * (1 - self.cfg.timestamp_tolerance),
                dt_ns * (1 + self.cfg.timestamp_tolerance))

    def valid_centers(self, ids: Iterable[str]) -> List[Tuple[str, int]]:
        out = []
        n, h = self.cfg.history_len, self.cfg.horizon
        min_dt, max_dt = self._timing_bounds_ns()
        for tid in ids:
            rows = self.data.trajectories[tid]
            for k in range(n - 1, len(rows) - h):
                segment = rows[k - n + 1:k + h + 1]
                if any(not self._row_is_valid(r) for r in segment):
                    continue
                dts = np.diff([r["timestamp_ns"] for r in segment])
                if np.any(dts < min_dt) or np.any(dts > max_dt):
                    continue
                out.append((tid, k))
        return out

    def trajectory_diagnostics(self, tid: str) -> Dict[str, object]:
        """Explain why a trajectory does or does not produce training windows."""
        rows = self.data.trajectories[tid]
        required_rows = self.cfg.history_len + self.cfg.horizon
        candidate_windows = max(len(rows) - required_rows + 1, 0)
        row_valid = np.asarray([self._row_is_valid(row) for row in rows], dtype=bool)
        timestamps = np.asarray([row["timestamp_ns"] for row in rows], dtype=np.int64)
        dts_ns = np.diff(timestamps)
        min_dt, max_dt = self._timing_bounds_ns()
        timing_valid = (dts_ns >= min_dt) & (dts_ns <= max_dt)

        # Number of consecutive usable rows. A valid interval connects the new
        # row to the preceding one; invalid sensor/status rows also break a run.
        longest_run = current_run = 0
        for index, valid in enumerate(row_valid):
            connected = index == 0 or bool(timing_valid[index - 1])
            if valid and connected:
                current_run += 1
            elif valid:
                current_run = 1
            else:
                current_run = 0
            longest_run = max(longest_run, current_run)

        dt_ms = dts_ns.astype(np.float64) * 1e-6
        timing = {
            "expected_dt_ms": 1000.0 * self.cfg.expected_dt_s,
            "allowed_dt_ms": [min_dt * 1e-6, max_dt * 1e-6],
            "median_dt_ms": float(np.median(dt_ms)) if len(dt_ms) else None,
            "p95_dt_ms": float(np.percentile(dt_ms, 95)) if len(dt_ms) else None,
            "max_dt_ms": float(np.max(dt_ms)) if len(dt_ms) else None,
            "out_of_tolerance_intervals": int((~timing_valid).sum()),
        }
        return {
            "rows": len(rows),
            "required_consecutive_rows_per_window": required_rows,
            "candidate_windows_before_filtering": candidate_windows,
            "valid_windows": len(self.valid_centers([tid])),
            "invalid_status_rows": int((~row_valid).sum()),
            "longest_valid_contiguous_run_rows": longest_run,
            "timing": timing,
        }

    @staticmethod
    def _values(row: Mapping[str, float], prefix: str, dim: int) -> List[float]:
        return [float(row[f"{prefix}_{i}"]) for i in range(dim)]

    def history(self, tid: str, k: int) -> np.ndarray:
        rows = self.data.trajectories[tid][k-self.cfg.history_len+1:k+1]
        motor_origin = float(rows[0]["motor_position_rad"])
        output = np.empty((len(rows), self.layout.obs_dim), np.float32)
        for i, r in enumerate(rows):
            values: Dict[str, float] = dict(r)
            values["motor_position_relative"] = float(r["motor_position_rad"]) - motor_origin
            output[i] = [values[name] for name in self.layout.names]
        return output

    def preview(self, tid: str, k: int) -> np.ndarray:
        r, c = self.data.trajectories[tid][k], self.cfg
        held = ([float(r["force_reference_n"])]
                + self._values(r, "velocity_command", c.velocity_dim)
                + (self._values(r, "policy_action", c.policy_action_dim)
                   if c.use_policy_action else []))
        return np.repeat(np.asarray(held, np.float32)[None, :], c.horizon, axis=0)

    def raw_actions_targets(self, tid: str, k: int
                            ) -> Tuple[np.ndarray, np.ndarray, float, np.ndarray]:
        rows, c = self.data.trajectories[tid], self.cfg
        actions = np.array([rows[k+h]["torque_command_issued_nm"]
                            for h in range(c.horizon)], np.float32)[:, None]
        target = np.array([rows[k+h+1]["force_raw_n"]
                           for h in range(c.horizon)], np.float32)
        current = float(rows[k]["force_raw_n"])
        q_len = max(c.action_delay_steps - 1, 0)
        queue = np.array([rows[k-q_len+1+i]["torque_command_prev_nm"]
                          for i in range(q_len)], np.float32)[:, None]
        return actions, target, current, queue


def fit_normalization(builder: WindowBuilder, centers: Sequence[Tuple[str, int]]) -> Normalization:
    hs, ps = RunningStats(builder.layout.obs_dim), RunningStats(builder.preview_dim)
    torque = RunningStats(1)
    for tid, k in centers:
        hs.update(builder.history(tid, k))
        ps.update(builder.preview(tid, k))
        actions, _, _, queue = builder.raw_actions_targets(tid, k)
        torque.update(actions)
        torque.update(queue)
    hm, hstd = hs.result()
    pm, pstd = ps.result()
    tm, tstd = torque.result()
    return Normalization(hm, hstd, pm, pstd, float(tm[0]), float(tstd[0]))


class TensionWindowDataset(Dataset):
    def __init__(self, builder: WindowBuilder, centers: Sequence[Tuple[str, int]],
                 norm: Normalization) -> None:
        self.builder, self.centers, self.norm = builder, list(centers), norm

    def __len__(self) -> int:
        return len(self.centers)

    def __getitem__(self, index: int) -> Dict[str, Tensor]:
        tid, k = self.centers[index]
        history = (self.builder.history(tid, k) - self.norm.history_mean) / self.norm.history_std
        preview = (self.builder.preview(tid, k) - self.norm.preview_mean) / self.norm.preview_std
        actions, target, current, queue = self.builder.raw_actions_targets(tid, k)
        actions = (actions - self.norm.torque_mean) / self.norm.torque_std
        queue = (queue - self.norm.torque_mean) / self.norm.torque_std
        return {
            "history": torch.from_numpy(history),
            "future_torque": torch.from_numpy(actions),
            "future_preview": torch.from_numpy(preview),
            "target_force": torch.from_numpy(target),
            "current_force": torch.tensor([current], dtype=torch.float32),
            "delay_queue_state": torch.from_numpy(queue),
        }


class MLP(nn.Sequential):
    def __init__(self, input_dim: int, hidden: int, output_dim: int) -> None:
        super().__init__(nn.Linear(input_dim, hidden), nn.SiLU(),
                         nn.Linear(hidden, output_dim), nn.SiLU())


class GroupedFrameEncoder(nn.Module):
    def __init__(self, layout: FeatureLayout, cfg: ModelConfig) -> None:
        super().__init__()
        self.slices = layout.slices
        self.encoders = nn.ModuleDict({
            name: MLP(sl.stop - sl.start, cfg.group_hidden_dim, cfg.group_hidden_dim)
            for name, sl in self.slices.items()
        })
        self.fuse = nn.Sequential(nn.Linear(len(self.slices) * cfg.group_hidden_dim,
                                            cfg.frame_embed_dim),
                                  nn.SiLU())

    def forward(self, x: Tensor) -> Tensor:
        shape = x.shape[:-1]
        encoded = [self.encoders[name](x[..., sl]) for name, sl in self.slices.items()]
        return self.fuse(torch.cat(encoded, -1)).reshape(*shape, -1)


class CausalResidualBlock(nn.Module):
    def __init__(self, channels: int, kernel_size: int, dilation: int) -> None:
        super().__init__()
        self.left_padding = (kernel_size - 1) * dilation
        self.conv1 = nn.Conv1d(channels, channels, kernel_size, dilation=dilation)
        self.conv2 = nn.Conv1d(channels, channels, kernel_size, dilation=dilation)
        # LayerNorm is applied per time step; GroupNorm over [C,T] would mix
        # temporal statistics and break the strict causal interpretation.
        self.norm1 = nn.LayerNorm(channels)
        self.norm2 = nn.LayerNorm(channels)

    def _conv(self, x: Tensor, conv: nn.Conv1d) -> Tensor:
        return conv(F.pad(x, (self.left_padding, 0)))

    @staticmethod
    def _norm(x: Tensor, norm: nn.LayerNorm) -> Tensor:
        return norm(x.transpose(1, 2)).transpose(1, 2)

    def forward(self, x: Tensor) -> Tensor:
        residual = x
        x = F.silu(self._norm(self._conv(x, self.conv1), self.norm1))
        x = self._norm(self._conv(x, self.conv2), self.norm2)
        return F.silu(x + residual)


class CausalTCNEncoder(nn.Module):
    def __init__(self, input_dim: int, channels: int, kernel_size: int,
                 history_len: int) -> None:
        super().__init__()
        if kernel_size < 2:
            raise ValueError("kernel_size must be >= 2")
        dilations, receptive = [], 1
        dilation = 1
        # Each residual block has two causal convolutions.
        while receptive < history_len:
            dilations.append(dilation)
            receptive += 2 * (kernel_size - 1) * dilation
            dilation *= 2
        if receptive < history_len:
            raise ValueError(f"TCN receptive field {receptive} < history_len {history_len}")
        self.receptive_field = receptive
        self.input_projection = nn.Conv1d(input_dim, channels, 1)
        self.blocks = nn.Sequential(*[
            CausalResidualBlock(channels, kernel_size, d) for d in dilations
        ])

    def forward(self, x: Tensor) -> Tensor:
        # [B,N,C] -> [B,C,N], only the final causal output represents z_k.
        return self.blocks(self.input_projection(x.transpose(1, 2)))[:, :, -1]


class ForceDeltaHead(nn.Sequential):
    def __init__(self, latent_dim: int) -> None:
        super().__init__(nn.Linear(latent_dim, 32), nn.SiLU(), nn.Linear(32, 1))

# 核心模型
class TCNGRUTensionPredictor(nn.Module):
    def __init__(self, data_cfg: DataConfig, model_cfg: ModelConfig) -> None:
        super().__init__()
        if data_cfg.action_delay_steps < 1:
            raise ValueError("action_delay_steps must be >= 1")
        if data_cfg.action_delay_steps > data_cfg.history_len:
            raise ValueError("action_delay_steps cannot exceed history_len")
        if model_cfg.latent_dim != model_cfg.gru_hidden_dim:
            raise ValueError("first-stage model requires latent_dim == gru_hidden_dim")
        self.data_cfg = data_cfg
        self.layout = FeatureLayout(data_cfg)
        self.frame_encoder = GroupedFrameEncoder(self.layout, model_cfg)
        self.tcn = CausalTCNEncoder(model_cfg.frame_embed_dim, model_cfg.tcn_channels,
                                    model_cfg.kernel_size, data_cfg.history_len)
        self.latent_projection = nn.Sequential(
            nn.Linear(model_cfg.tcn_channels, model_cfg.latent_dim), nn.Tanh())
        # Formula mirrors WindowBuilder.preview_names without requiring data.
        pdim = (1 + data_cfg.velocity_dim
                + (data_cfg.policy_action_dim if data_cfg.use_policy_action else 0))
        self.gru = nn.GRUCell(1 + pdim, model_cfg.gru_hidden_dim)
        self.force_delta_head = ForceDeltaHead(model_cfg.gru_hidden_dim)

    def encode_history(self, history: Tensor) -> Tensor:
        return self.latent_projection(self.tcn(self.frame_encoder(history)))

    @staticmethod
    def effective_torque(future_torque: Tensor, delay_queue_state: Tensor) -> Tensor:
        if delay_queue_state.ndim != 3 or future_torque.ndim != 3:
            raise ValueError("future_torque and delay_queue_state must be [B,T,1]")
        combined = torch.cat((delay_queue_state, future_torque), dim=1)
        return combined[:, :future_torque.shape[1]]

    def rollout_from_latent(self, z_k: Tensor, future_torque: Tensor,
                            future_preview: Tensor,
                            delay_queue_state: Tensor) -> Tensor:
        effective = self.effective_torque(future_torque, delay_queue_state)
        z, deltas = z_k, []
        for h in range(future_torque.shape[1]):
            z = self.gru(torch.cat((effective[:, h], future_preview[:, h]), -1), z)
            deltas.append(self.force_delta_head(z))
        return torch.cat(deltas, dim=1)

    def forward(self, history: Tensor, future_torque: Tensor, future_preview: Tensor,
                delay_queue_state: Tensor) -> Tensor:
        return self.rollout_from_latent(self.encode_history(history), future_torque,
                                        future_preview, delay_queue_state)



# 损失函数
class TensionLoss(nn.Module):
    def __init__(self, cfg: TrainConfig, horizon: int, action_delay_steps: int) -> None:
        super().__init__()
        weights = cfg.horizon_weights or [1.0] * horizon
        if len(weights) != horizon:
            raise ValueError("horizon_weights length must equal horizon")
        # Delay step d means candidate first affects force prediction index d-1.
        for i in range(min(action_delay_steps - 1, horizon)):
            weights[i] *= 0.25
        self.register_buffer("weights", torch.tensor(weights)[None, :])
        self.cfg = cfg

    def forward(self, pred_delta: Tensor, target: Tensor,
                current: Tensor) -> Tuple[Tensor, Dict[str, Tensor]]:
        target_delta = target - current
        pred_force = current + pred_delta
        point = F.huber_loss(pred_delta, target_delta, reduction="none",
                             delta=self.cfg.huber_delta)
        force_loss = (point * self.weights).sum() / self.weights.sum() / pred_delta.shape[0]
        if pred_force.shape[1] > 1:
            trend = F.huber_loss(torch.diff(pred_force, dim=1),
                                 torch.diff(target, dim=1), delta=self.cfg.huber_delta)
        else:
            trend = pred_force.new_zeros(())
        peak = (pred_force.max(1).values - target.max(1).values).abs().mean()
        minimum = (pred_force.min(1).values - target.min(1).values).abs().mean()
        nonnegative = F.relu(-pred_force).square().mean()
        parts = {"force": force_loss, "trend": trend, "peak": peak,
                 "minimum": minimum, "nonnegative": nonnegative}
        total = (self.cfg.lambda_force * force_loss + self.cfg.lambda_trend * trend
                 + self.cfg.lambda_peak * peak + self.cfg.lambda_minimum * minimum
                 + self.cfg.lambda_nonnegative * nonnegative)
        return total, parts


def _to_device(batch: Mapping[str, Tensor], device: torch.device) -> Dict[str, Tensor]:
    return {k: v.to(device, non_blocking=True) for k, v in batch.items()}


def evaluate(model: nn.Module, loader: DataLoader, loss_fn: TensionLoss,
             device: torch.device, dt_s: float) -> Dict[str, float]:
    model.eval()
    losses, predictions, targets = [], [], []
    with torch.inference_mode():
        for raw in loader:
            b = _to_device(raw, device)
            delta = model(b["history"], b["future_torque"], b["future_preview"],
                          b["delay_queue_state"])
            force = b["current_force"] + delta
            loss, _ = loss_fn(delta, b["target_force"], b["current_force"])
            losses.append(loss.item() * len(force))
            predictions.append(force.cpu())
            targets.append(b["target_force"].cpu())
    if not predictions:
        raise ValueError("evaluation split has no valid windows")
    pred, target = torch.cat(predictions), torch.cat(targets)
    error = pred - target
    metrics = {"loss": sum(losses) / len(pred),
               "rmse": error.square().mean().sqrt().item()}
    for h in range(pred.shape[1]):
        milliseconds = int(round(1000.0 * dt_s * (h + 1)))
        metrics[f"mae_{milliseconds}ms"] = error[:, h].abs().mean().item()
    if pred.shape[1] > 1:
        metrics["trend_sign_accuracy"] = (torch.sign(torch.diff(pred, dim=1)) ==
                                           torch.sign(torch.diff(target, dim=1))).float().mean().item()
    metrics["peak_magnitude_error"] = (pred.max(1).values-target.max(1).values).abs().mean().item()
    metrics["minimum_force_error"] = (pred.min(1).values-target.min(1).values).abs().mean().item()
    return metrics


def train(args: argparse.Namespace) -> None:
    data_cfg, model_cfg, train_cfg = DataConfig(), ModelConfig(), TrainConfig()
    for cfg in (data_cfg, model_cfg, train_cfg):
        for key in cfg.__dataclass_fields__:
            if hasattr(args, key) and getattr(args, key) is not None:
                setattr(cfg, key, getattr(args, key))
    random.seed(train_cfg.seed); np.random.seed(train_cfg.seed); torch.manual_seed(train_cfg.seed)
    data = CsvTrajectories(Path(args.csv), data_cfg)
    train_ids, val_ids, test_ids = split_trajectory_ids(list(data.trajectories), train_cfg)
    builder = WindowBuilder(data, data_cfg)
    centers = {"train": builder.valid_centers(train_ids),
               "val": builder.valid_centers(val_ids),
               "test": builder.valid_centers(test_ids)}
    if any(not v for v in centers.values()):
        split = {"train": train_ids, "val": val_ids, "test": test_ids}
        diagnostics = {
            tid: builder.trajectory_diagnostics(tid)
            for ids in split.values() for tid in ids
        }
        report = {
            "trajectory_split": split,
            "window_counts": {key: len(value) for key, value in centers.items()},
            "trajectory_diagnostics": diagnostics,
        }
        raise ValueError(
            "trajectory train/val/test splitting succeeded, but one or more splits "
            "contain no valid windows. A window needs "
            f"{data_cfg.history_len + data_cfg.horizon} consecutive valid rows with "
            "timestamps inside the configured tolerance. Diagnostics:\n"
            + json.dumps(report, indent=2)
        )
    norm = fit_normalization(builder, centers["train"])
    datasets = {k: TensionWindowDataset(builder, v, norm) for k, v in centers.items()}
    loaders = {k: DataLoader(ds, batch_size=train_cfg.batch_size, shuffle=(k == "train"),
                             num_workers=train_cfg.num_workers, pin_memory=torch.cuda.is_available())
               for k, ds in datasets.items()}
    device = torch.device(args.device or ("cuda" if torch.cuda.is_available() else "cpu"))
    model = TCNGRUTensionPredictor(data_cfg, model_cfg).to(device)
    loss_fn = TensionLoss(train_cfg, data_cfg.horizon, data_cfg.action_delay_steps).to(device)
    optimizer = torch.optim.AdamW(model.parameters(), lr=train_cfg.learning_rate,
                                  weight_decay=train_cfg.weight_decay)
    best, best_state, stale = math.inf, None, 0
    for epoch in range(1, train_cfg.epochs + 1):
        model.train()
        running = 0.0
        for raw in loaders["train"]:
            b = _to_device(raw, device)
            optimizer.zero_grad(set_to_none=True)
            delta = model(b["history"], b["future_torque"], b["future_preview"],
                          b["delay_queue_state"])
            loss, _ = loss_fn(delta, b["target_force"], b["current_force"])
            loss.backward()
            nn.utils.clip_grad_norm_(model.parameters(), train_cfg.grad_clip_norm)
            optimizer.step()
            running += loss.item() * len(b["history"])
        val = evaluate(model, loaders["val"], loss_fn, device, data_cfg.expected_dt_s)
        print(f"epoch={epoch:03d} train_loss={running/len(datasets['train']):.6f} "
              f"val_loss={val['loss']:.6f} val_rmse={val['rmse']:.4f}")
        if val["loss"] < best:
            best, stale = val["loss"], 0
            best_state = {k: v.detach().cpu().clone() for k, v in model.state_dict().items()}
        else:
            stale += 1
            if stale >= train_cfg.patience:
                print(f"early stopping after {epoch} epochs")
                break
    assert best_state is not None
    model.load_state_dict(best_state)
    test_metrics = evaluate(model, loaders["test"], loss_fn, device,
                            data_cfg.expected_dt_s)
    checkpoint = {
        "model_state_dict": best_state, "data_config": asdict(data_cfg),
        "model_config": asdict(model_cfg), "train_config": asdict(train_cfg),
        "normalization": norm.state_dict(),
        "trajectory_split": {"train": train_ids, "val": val_ids, "test": test_ids},
        "window_counts": {k: len(v) for k, v in centers.items()},
        "test_metrics": test_metrics,
        "feature_names": builder.layout.names, "preview_names": builder.preview_names,
    }
    Path(args.output).parent.mkdir(parents=True, exist_ok=True)
    torch.save(checkpoint, args.output)
    print(json.dumps({"saved": args.output, "test": test_metrics,
                      "windows": checkpoint["window_counts"]}, indent=2))


def inspect_csv(args: argparse.Namespace) -> None:
    cfg = DataConfig(history_len=args.history_len, horizon=args.horizon,
                     action_delay_steps=args.action_delay_steps,
                     expected_dt_s=args.expected_dt_s,
                     timestamp_tolerance=args.timestamp_tolerance)
    data = CsvTrajectories(Path(args.csv), cfg)
    builder = WindowBuilder(data, cfg)
    diagnostics = {
        tid: builder.trajectory_diagnostics(tid) for tid in data.trajectories
    }
    counts = {tid: int(item["valid_windows"])
              for tid, item in diagnostics.items()}
    print(json.dumps({"trajectories": len(counts), "rows": sum(map(len, data.trajectories.values())),
                      "valid_windows_by_trajectory": counts,
                      "trajectory_diagnostics": diagnostics,
                      "obs_dim": builder.layout.obs_dim,
                      "preview_dim": builder.preview_dim}, indent=2))


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument("--csv", required=True)
    common.add_argument("--history-len", type=int, default=64)
    common.add_argument("--horizon", type=int, default=5)
    common.add_argument("--action-delay-steps", type=int, default=1)
    common.add_argument("--expected-dt-s", type=float, default=0.02)
    common.add_argument("--timestamp-tolerance", type=float, default=0.35)
    check = sub.add_parser("inspect-csv", parents=[common])
    check.set_defaults(func=inspect_csv)
    run = sub.add_parser("train", parents=[common])
    run.add_argument("--output", required=True)
    run.add_argument("--device", default=None)
    run.add_argument("--epochs", type=int, default=None)
    run.add_argument("--batch-size", type=int, default=None)
    run.add_argument("--learning-rate", type=float, default=None)
    run.add_argument("--num-workers", type=int, default=None)
    run.add_argument("--seed", type=int, default=None)
    run.add_argument("--patience", type=int, default=None)
    run.set_defaults(func=train)
    return parser


if __name__ == "__main__":
    cli_args = build_parser().parse_args()
    cli_args.func(cli_args)
