"""Deployment-only recurrent MoE policy and 68-D observation contract.

This module intentionally has no dependency on RSL-RL, TensorDict, JAX, or
MuJoCo.  The same file can therefore be used by sim2sim and copied into the
real-robot project.  Only the deployable half of ``MoEActorCriticGRU`` is
reconstructed: estimator GRU, actor GRU, shared gate, and six actor experts.
Training-only oracle encoders and the critic are never evaluated here.
"""

from __future__ import annotations

from collections.abc import Sequence
from pathlib import Path
from typing import Any

import numpy as np
import torch
from torch import nn

STATE_DIM = 68
ACTION_DIM = 19
EXPLICIT_CONTEXT_DIM = 4
CONTACT_LATENT_DIM = 6
CONTEXT_DIM = EXPLICIT_CONTEXT_DIM + CONTACT_LATENT_DIM
HIDDEN_DIM = 256
NUM_EXPERTS = 6
RNN_NUM_LAYERS = 1
NORMALIZER_EPS = 1e-2


def unit_direction_from_yaw_pitch(yaw: float, pitch: float) -> np.ndarray:
    """Return body-frame [x-forward, y-left, z-up] tether direction."""
    horizontal = np.cos(float(pitch))
    direction = np.array(
        [
            horizontal * np.cos(float(yaw)),
            horizontal * np.sin(float(yaw)),
            np.sin(float(pitch)),
        ],
        dtype=np.float32,
    )
    return direction / max(float(np.linalg.norm(direction)), 1e-8)


def yaw_pitch_from_unit_direction(direction: np.ndarray) -> tuple[float, float]:
    """Inverse of :func:`unit_direction_from_yaw_pitch`."""
    unit = np.asarray(direction, dtype=np.float32).reshape(3)
    unit = unit / max(float(np.linalg.norm(unit)), 1e-8)
    yaw = float(np.arctan2(unit[1], unit[0]))
    pitch = float(np.arctan2(unit[2], np.hypot(unit[0], unit[1])))
    return yaw, pitch


def build_deployable_state(
    *,
    gyro: np.ndarray,
    gravity: np.ndarray,
    leg_joint_position_error: np.ndarray,
    leg_joint_velocity: np.ndarray,
    previous_action: np.ndarray,
    command: np.ndarray,
    cable_tension: float,
    tether_unit_direction_body: np.ndarray,
) -> np.ndarray:
    """Build the exact 68-D MoE state used during training.

    Layout:
      gyro(3), gravity(3), leg q error(18), leg qd(18),
      previous action(19), command(3), tension(1), tether unit vector(3).

    Linear velocity, friction, and contact force are deliberately absent.  The
    estimator GRU predicts their compact context from this state history.
    """

    direction = np.asarray(tether_unit_direction_body, dtype=np.float32).reshape(3)
    direction_norm = float(np.linalg.norm(direction))
    if not np.isfinite(direction_norm) or direction_norm < 1e-8:
        raise ValueError("tether direction must be a finite nonzero 3-D vector")
    direction = direction / direction_norm

    state = np.concatenate(
        (
            np.asarray(gyro, dtype=np.float32).reshape(3),
            np.asarray(gravity, dtype=np.float32).reshape(3),
            np.asarray(leg_joint_position_error, dtype=np.float32).reshape(18),
            np.asarray(leg_joint_velocity, dtype=np.float32).reshape(18),
            np.asarray(previous_action, dtype=np.float32).reshape(ACTION_DIM),
            np.asarray(command, dtype=np.float32).reshape(3),
            np.asarray([max(float(cable_tension), 0.0)], dtype=np.float32),
            direction,
        )
    ).astype(np.float32, copy=False)
    if state.shape != (STATE_DIM,):
        raise AssertionError(f"internal state layout error: {state.shape}")
    if not np.all(np.isfinite(state)):
        raise ValueError("MoE state contains NaN or Inf")
    return state


class _DenseNetwork(nn.Module):
    """Network whose module names match the training checkpoint exactly."""

    def __init__(
        self,
        input_dim: int,
        output_dim: int,
        hidden_dims: Sequence[int],
    ) -> None:
        super().__init__()
        layers: list[nn.Module] = []
        previous_dim = int(input_dim)
        for hidden_dim in hidden_dims:
            layers.extend((nn.Linear(previous_dim, int(hidden_dim)), nn.ELU()))
            previous_dim = int(hidden_dim)
        layers.append(nn.Linear(previous_dim, int(output_dim)))
        self.network = nn.Sequential(*layers)

    def forward(self, value: torch.Tensor) -> torch.Tensor:
        return self.network(value)


class _DenseMoE(nn.Module):

    def __init__(self) -> None:
        super().__init__()
        self.experts = nn.ModuleList(
            [
                _DenseNetwork(HIDDEN_DIM, ACTION_DIM, (256, 128, 128))
                for _ in range(NUM_EXPERTS)
            ]
        )

    def forward(
        self, value: torch.Tensor, gate_weights: torch.Tensor
    ) -> torch.Tensor:
        expert_outputs = torch.stack(
            [expert(value) for expert in self.experts], dim=-2
        )
        return torch.sum(expert_outputs * gate_weights.unsqueeze(-1), dim=-2)


class _FrozenNormalizer(nn.Module):

    def __init__(self, width: int) -> None:
        super().__init__()
        self.eps = float(NORMALIZER_EPS)
        self.register_buffer("_mean", torch.zeros(1, int(width)))
        self.register_buffer("_std", torch.ones(1, int(width)))

    def forward(self, value: torch.Tensor) -> torch.Tensor:
        return (value - self._mean) / (self._std + self.eps)


class MoEInferenceCore(nn.Module):
    """Stateless recurrent core with explicit estimator/actor GRU states."""

    def __init__(self, gate_temperature: float = 1.0) -> None:
        super().__init__()
        if gate_temperature <= 0.0:
            raise ValueError("gate_temperature must be positive")
        self.gate_temperature = float(gate_temperature)

        self.estimator_gru = nn.GRU(STATE_DIM, HIDDEN_DIM, RNN_NUM_LAYERS)
        self.actor_gru = nn.GRU(
            STATE_DIM + CONTEXT_DIM, HIDDEN_DIM, RNN_NUM_LAYERS
        )
        self.explicit_estimator = _DenseNetwork(
            HIDDEN_DIM, EXPLICIT_CONTEXT_DIM, (256, 128)
        )
        self.contact_latent_estimator = _DenseNetwork(
            HIDDEN_DIM, CONTACT_LATENT_DIM, (256, 128)
        )
        self.shared_gate = _DenseNetwork(HIDDEN_DIM, NUM_EXPERTS, (128,))
        self.actor = _DenseMoE()
        self.actor_obs_normalizer = _FrozenNormalizer(STATE_DIM)
        self.estimator_input_normalizer = _FrozenNormalizer(STATE_DIM)

    def forward(
        self,
        state: torch.Tensor,
        estimator_hidden: torch.Tensor,
        actor_hidden: torch.Tensor,
    ) -> tuple[
        torch.Tensor,
        torch.Tensor,
        torch.Tensor,
        torch.Tensor,
        torch.Tensor,
    ]:
        """Run one control frame and return action, states, context, gate."""
        estimator_input = self.estimator_input_normalizer(state)
        temporal, estimator_hidden_next = self.estimator_gru(
            estimator_input.unsqueeze(0), estimator_hidden.contiguous()
        )
        temporal = temporal.squeeze(0)
        context = torch.cat(
            (
                self.explicit_estimator(temporal),
                self.contact_latent_estimator(temporal),
            ),
            dim=-1,
        )

        actor_input = torch.cat(
            (self.actor_obs_normalizer(state), context), dim=-1
        )
        actor_temporal, actor_hidden_next = self.actor_gru(
            actor_input.unsqueeze(0), actor_hidden.contiguous()
        )
        actor_temporal = actor_temporal.squeeze(0)
        gate_weights = torch.softmax(
            self.shared_gate(actor_temporal) / self.gate_temperature, dim=-1
        )
        action = self.actor(actor_temporal, gate_weights)
        return (
            action,
            estimator_hidden_next,
            actor_hidden_next,
            context,
            gate_weights,
        )


def _extract_state_dict(checkpoint: Any) -> dict[str, torch.Tensor]:
    if isinstance(checkpoint, dict) and isinstance(
        checkpoint.get("model_state_dict"), dict
    ):
        state_dict = checkpoint["model_state_dict"]
    elif isinstance(checkpoint, dict) and all(
        isinstance(value, torch.Tensor) for value in checkpoint.values()
    ):
        state_dict = checkpoint
    else:
        raise RuntimeError(
            "Expected an RSL-RL checkpoint containing model_state_dict or a "
            "bare tensor state_dict."
        )
    if state_dict and all(key.startswith("module.") for key in state_dict):
        prefix_length = len("module.")
        state_dict = {
            key[prefix_length:]: value for key, value in state_dict.items()
        }
    return state_dict


def load_inference_core(
    checkpoint_path: str | Path,
    *,
    device: str | torch.device = "cpu",
) -> MoEInferenceCore:
    """Reconstruct the deployable core from a raw RSL-RL MoE checkpoint."""
    map_location = torch.device(device)
    try:
        checkpoint = torch.load(
            str(checkpoint_path), map_location=map_location, weights_only=False
        )
    except TypeError:  # PyTorch versions before ``weights_only``.
        checkpoint = torch.load(str(checkpoint_path), map_location=map_location)
    state_dict = _extract_state_dict(checkpoint)

    core = MoEInferenceCore()
    expected = core.state_dict()
    required: dict[str, torch.Tensor] = {}
    missing: list[str] = []
    mismatched: list[str] = []
    for key, expected_value in expected.items():
        value = state_dict.get(key)
        if value is None:
            missing.append(key)
        elif tuple(value.shape) != tuple(expected_value.shape):
            mismatched.append(
                f"{key}: checkpoint {tuple(value.shape)} != expected "
                f"{tuple(expected_value.shape)}"
            )
        else:
            required[key] = value
    if missing or mismatched:
        details = []
        if missing:
            details.append(f"missing={missing}")
        if mismatched:
            details.append(f"shape_mismatch={mismatched}")
        raise RuntimeError(
            "Checkpoint does not match the configured 68-D/6-expert MoE "
            "architecture: " + "; ".join(details)
        )

    core.load_state_dict(required, strict=True)
    core.to(map_location)
    core.eval()
    return core


def export_inference_core(
    core: MoEInferenceCore, output_path: str | Path
) -> Path:
    """Export a recurrent TorchScript model with explicit hidden-state I/O."""
    output = Path(output_path).expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    scripted = torch.jit.script(core.cpu().eval())
    torch.jit.save(scripted, str(output))
    return output


class StatefulMoEPolicy:
    """Own the two online GRU states around a raw or TorchScript core."""

    def __init__(
        self,
        core: nn.Module,
        *,
        device: str | torch.device = "cpu",
    ) -> None:
        self.device = torch.device(device)
        self.core = core.to(self.device).eval()
        self.estimator_hidden = torch.zeros(
            RNN_NUM_LAYERS, 1, HIDDEN_DIM, device=self.device
        )
        self.actor_hidden = torch.zeros(
            RNN_NUM_LAYERS, 1, HIDDEN_DIM, device=self.device
        )
        self.last_context = np.zeros(CONTEXT_DIM, dtype=np.float32)
        self.last_gate = np.full(NUM_EXPERTS, 1.0 / NUM_EXPERTS, dtype=np.float32)

    def reset(self) -> None:
        self.estimator_hidden.zero_()
        self.actor_hidden.zero_()
        self.last_context.fill(0.0)
        self.last_gate.fill(1.0 / NUM_EXPERTS)

    def __call__(self, state: np.ndarray) -> np.ndarray:
        state_array = np.asarray(state, dtype=np.float32).reshape(-1)
        if state_array.shape != (STATE_DIM,):
            raise ValueError(
                f"MoE policy expects {STATE_DIM} values, got {state_array.shape}"
            )
        state_tensor = torch.from_numpy(state_array).unsqueeze(0).to(self.device)
        with torch.no_grad():
            output = self.core(
                state_tensor, self.estimator_hidden, self.actor_hidden
            )
        if not isinstance(output, (tuple, list)) or len(output) != 5:
            raise RuntimeError(
                "Recurrent MoE TorchScript must return "
                "(action, estimator_hidden, actor_hidden, context, gate)."
            )
        action, estimator_hidden, actor_hidden, context, gate = output
        self.estimator_hidden = estimator_hidden.detach()
        self.actor_hidden = actor_hidden.detach()
        self.last_context = context[0].detach().cpu().numpy().astype(np.float32)
        self.last_gate = gate[0].detach().cpu().numpy().astype(np.float32)
        result = action[0].detach().cpu().numpy().astype(np.float32)
        if result.shape != (ACTION_DIM,) or not np.all(np.isfinite(result)):
            raise RuntimeError(f"invalid MoE action: shape={result.shape}")
        return result


def load_moe_policy(
    policy_path: str | Path,
    *,
    device: str | torch.device = "cpu",
) -> StatefulMoEPolicy:
    """Load either a raw RSL-RL checkpoint or exported recurrent TorchScript."""
    path = Path(policy_path).expanduser().resolve()
    if not path.is_file():
        raise FileNotFoundError(f"MoE policy does not exist: {path}")
    map_location = torch.device(device)
    try:
        core: nn.Module = torch.jit.load(str(path), map_location=map_location)
        print(f"[MoE] loaded recurrent TorchScript: {path}")
    except (RuntimeError, ValueError):
        core = load_inference_core(path, device=map_location)
        print(f"[MoE] loaded raw RSL-RL checkpoint: {path}")
    policy = StatefulMoEPolicy(core, device=map_location)
    # A real forward call validates the TorchScript signature and all shapes.
    policy(np.zeros(STATE_DIM, dtype=np.float32))
    policy.reset()
    return policy
