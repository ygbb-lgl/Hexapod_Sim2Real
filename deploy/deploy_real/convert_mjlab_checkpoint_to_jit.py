#!/usr/bin/env python3
"""Convert the configured MJLab RSL-RL checkpoint to TorchScript.

The checkpoint and output paths are intentionally absolute and configured below.
Run this file with a Python environment that provides PyTorch::

    python convert_mjlab_checkpoint_to_jit.py

The exported policy accepts a float tensor of shape ``(..., 66)`` and returns
the deterministic 18-dimensional action produced by the actor mean network.
"""

from __future__ import annotations

import os
import tempfile
from pathlib import Path

import torch
import torch.nn.functional as F
from torch import nn

# ---------------------------------------------------------------------------
# Absolute input/output paths
# ---------------------------------------------------------------------------
CHECKPOINT_PATH = Path("/home/lgl/mjlab/logs/2026-10-14_15-04-12-best/model_9500.pt")
POLICY_OUTPUT_PATH = Path(
    "/home/lgl/Hexapod_Sim2Real/deploy/pre_train/hexapod/policy_mjlab_9500.pt"
)

# MJLab uses rsl-rl's EmpiricalNormalization. In rsl-rl 5.5.1, its
# constructor defaults to eps=1e-2 and MLPModel creates it without overriding
# eps. Its forward method is: (x - self._mean) / (self._std + self.eps).
OBS_NORMALIZATION_EPS = 1e-2

# The activation is not encoded in a state_dict. This matches
# mjlab/tasks/velocity/config/hexapod/rl_cfg.py for this training run.
ACTIVATION = "elu"

OVERWRITE_OUTPUT = True
VALIDATION_SEED = 20261016
VALIDATION_ATOL = 1e-6
VALIDATION_RTOL = 1e-6


class MjlabActorPolicy(nn.Module):
    """Deployment-only deterministic actor reconstructed from a checkpoint."""

    def __init__(
        self,
        mean: torch.Tensor,
        std: torch.Tensor,
        mlp: nn.Sequential,
    ) -> None:
        super().__init__()
        self.register_buffer("obs_mean", mean.detach().clone().float())
        self.register_buffer("obs_std", std.detach().clone().float())
        self.eps = OBS_NORMALIZATION_EPS
        self.mlp = mlp

    def forward(self, obs: torch.Tensor) -> torch.Tensor:
        normalized_obs = (obs - self.obs_mean) / (self.obs_std + self.eps)
        return self.mlp(normalized_obs)


def _linear_layer_ids(actor_state_dict: dict[str, torch.Tensor]) -> list[int]:
    """Return the ordered MLP linear-layer indices in an actor state dict."""
    layer_ids = []
    for key in actor_state_dict:
        if not key.startswith("mlp.") or not key.endswith(".weight"):
            continue
        parts = key.split(".")
        if len(parts) == 3 and parts[1].isdigit():
            layer_ids.append(int(parts[1]))

    layer_ids = sorted(set(layer_ids))
    if not layer_ids:
        raise RuntimeError("No 'mlp.<index>.weight' actor layers found")
    return layer_ids


def _build_policy(
    actor_state_dict: dict[str, torch.Tensor],
) -> tuple[MjlabActorPolicy, list[int]]:
    """Reconstruct the normalized deterministic actor from checkpoint tensors."""
    mean_key = "obs_normalizer._mean"
    std_key = "obs_normalizer._std"
    if mean_key not in actor_state_dict or std_key not in actor_state_dict:
        raise RuntimeError(
            "The actor checkpoint has no observation-normalizer mean/std"
        )
    if ACTIVATION != "elu":
        raise RuntimeError(f"Unsupported configured activation: {ACTIVATION}")

    mean = actor_state_dict[mean_key]
    std = actor_state_dict[std_key]
    if mean.ndim != 2 or mean.shape[0] != 1 or std.shape != mean.shape:
        raise RuntimeError(
            "Expected normalizer mean/std with matching shape (1, obs_dim), "
            f"got mean={tuple(mean.shape)}, std={tuple(std.shape)}"
        )

    layer_ids = _linear_layer_ids(actor_state_dict)
    layers: list[nn.Module] = []
    previous_out_features = int(mean.shape[1])

    for position, layer_id in enumerate(layer_ids):
        weight_key = f"mlp.{layer_id}.weight"
        bias_key = f"mlp.{layer_id}.bias"
        if bias_key not in actor_state_dict:
            raise RuntimeError(f"Missing actor parameter: {bias_key}")

        weight = actor_state_dict[weight_key]
        bias = actor_state_dict[bias_key]
        if weight.ndim != 2 or bias.ndim != 1:
            raise RuntimeError(f"Invalid Linear tensors for mlp.{layer_id}")

        out_features, in_features = map(int, weight.shape)
        if in_features != previous_out_features or bias.shape[0] != out_features:
            raise RuntimeError(
                f"Inconsistent shapes for mlp.{layer_id}: "
                f"weight={tuple(weight.shape)}, bias={tuple(bias.shape)}, "
                f"expected in_features={previous_out_features}"
            )

        linear = nn.Linear(in_features, out_features)
        with torch.no_grad():
            linear.weight.copy_(weight.float())
            linear.bias.copy_(bias.float())
        layers.append(linear)
        if position < len(layer_ids) - 1:
            layers.append(nn.ELU())
        previous_out_features = out_features

    policy = MjlabActorPolicy(mean, std, nn.Sequential(*layers))
    return policy.eval(), layer_ids


def _checkpoint_forward(
    obs: torch.Tensor,
    actor_state_dict: dict[str, torch.Tensor],
    layer_ids: list[int],
) -> torch.Tensor:
    """Calculate a reference result directly from raw checkpoint tensors."""
    mean = actor_state_dict["obs_normalizer._mean"].float()
    std = actor_state_dict["obs_normalizer._std"].float()
    value = (obs - mean) / (std + OBS_NORMALIZATION_EPS)

    for position, layer_id in enumerate(layer_ids):
        value = F.linear(
            value,
            actor_state_dict[f"mlp.{layer_id}.weight"].float(),
            actor_state_dict[f"mlp.{layer_id}.bias"].float(),
        )
        if position < len(layer_ids) - 1:
            value = F.elu(value)
    return value


def _validate_export(
    actor_state_dict: dict[str, torch.Tensor],
    scripted_path: Path,
    layer_ids: list[int],
) -> dict[str, float]:
    """Compare raw-checkpoint and reloaded-JIT outputs on identical inputs."""
    scripted = torch.jit.load(str(scripted_path), map_location="cpu").eval()
    obs_dim = int(actor_state_dict["obs_normalizer._mean"].shape[1])
    mean = actor_state_dict["obs_normalizer._mean"].float()
    generator = torch.Generator(device="cpu").manual_seed(VALIDATION_SEED)
    test_inputs = {
        "zeros": torch.zeros(1, obs_dim),
        "running_mean": mean.clone(),
        "random_single": torch.randn(1, obs_dim, generator=generator),
        "random_batch_32": torch.randn(32, obs_dim, generator=generator),
    }

    max_abs_errors: dict[str, float] = {}
    with torch.inference_mode():
        for name, obs in test_inputs.items():
            reference = _checkpoint_forward(obs, actor_state_dict, layer_ids)
            actual = scripted(obs)
            if reference.shape != actual.shape:
                raise RuntimeError(
                    f"{name} output shape differs: checkpoint={tuple(reference.shape)}, "
                    f"JIT={tuple(actual.shape)}"
                )
            max_error = float(torch.max(torch.abs(reference - actual)).item())
            max_abs_errors[name] = max_error
            try:
                torch.testing.assert_close(
                    actual,
                    reference,
                    atol=VALIDATION_ATOL,
                    rtol=VALIDATION_RTOL,
                )
            except AssertionError as exc:
                raise RuntimeError(
                    f"JIT validation failed for {name}; max_abs_error={max_error:.3e}"
                ) from exc
    return max_abs_errors


def main() -> None:
    checkpoint_path = CHECKPOINT_PATH.expanduser().resolve()
    output_path = POLICY_OUTPUT_PATH.expanduser().resolve()

    if not checkpoint_path.is_file():
        raise FileNotFoundError(f"Checkpoint does not exist: {checkpoint_path}")
    if checkpoint_path == output_path:
        raise ValueError("Checkpoint and output paths must be different")
    if output_path.exists() and not OVERWRITE_OUTPUT:
        raise FileExistsError(
            f"Output already exists: {output_path}. "
            "Set OVERWRITE_OUTPUT=True to replace it."
        )

    print(f"Loading MJLab checkpoint: {checkpoint_path}")
    checkpoint = torch.load(
        checkpoint_path,
        map_location="cpu",
        weights_only=True,
    )
    if "actor_state_dict" not in checkpoint:
        raise RuntimeError("Checkpoint has no 'actor_state_dict'")
    actor_state_dict = checkpoint["actor_state_dict"]

    policy, layer_ids = _build_policy(actor_state_dict)
    obs_dim = int(actor_state_dict["obs_normalizer._mean"].shape[1])
    last_weight = actor_state_dict[f"mlp.{layer_ids[-1]}.weight"]
    action_dim = int(last_weight.shape[0])
    hidden_dims = [
        int(actor_state_dict[f"mlp.{layer_id}.weight"].shape[0])
        for layer_id in layer_ids[:-1]
    ]
    print(
        "Reconstructed deterministic actor: "
        f"obs={obs_dim}, hidden={hidden_dims}, action={action_dim}, "
        f"activation={ACTIVATION}, normalization_eps={OBS_NORMALIZATION_EPS}"
    )

    output_path.parent.mkdir(parents=True, exist_ok=True)
    temporary_path: Path | None = None
    try:
        with tempfile.NamedTemporaryFile(
            prefix=f".{output_path.stem}.",
            suffix=".tmp.pt",
            dir=output_path.parent,
            delete=False,
        ) as temporary_file:
            temporary_path = Path(temporary_file.name)

        scripted = torch.jit.script(policy)
        torch.jit.save(scripted, str(temporary_path))
        max_errors = _validate_export(
            actor_state_dict,
            temporary_path,
            layer_ids,
        )
        os.replace(temporary_path, output_path)
        output_path.chmod(0o644)
        temporary_path = None
    finally:
        if temporary_path is not None:
            temporary_path.unlink(missing_ok=True)

    error_summary = ", ".join(
        f"{name}={error:.3e}" for name, error in max_errors.items()
    )
    print(f"Validation passed; max absolute errors: {error_summary}")
    print(f"Saved TorchScript policy: {output_path}")
    print(f"File size: {output_path.stat().st_size / (1024 * 1024):.2f} MiB")


if __name__ == "__main__":
    main()
