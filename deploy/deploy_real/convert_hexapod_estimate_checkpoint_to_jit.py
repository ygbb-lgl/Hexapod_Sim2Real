#!/usr/bin/env python3
"""Convert a dimension-generic MJLab GRU-estimator Actor to TorchScript.

Edit the two absolute paths below, then run this file. Observation, GRU,
estimate, hidden-layer and action dimensions are inferred from the checkpoint.
The supported topology remains the MJLab HexapodEstimate topology:

  normalize -> one-layer GRU -> estimator head -> concatenate -> ELU MLP

The output is replaced only after a reloaded JIT model matches the reconstructed
raw-checkpoint policy over a deterministic recurrent sequence and after reset.
"""

import os
import tempfile
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path

import torch
import torch.nn as nn

# Paste absolute paths here. No command-line arguments are required.
CHECKPOINT_PATH = Path(
  "/home/lgl/Hexapod_Sim2Real/deploy/pre_train/hexapod_tethered_estimate/model_24500.pt"
)
POLICY_OUTPUT_PATH = Path(
  "/home/lgl/Hexapod_Sim2Real/deploy/pre_train/"
  "hexapod_tethered_estimate/policy_24500.pt"
)

OBS_NORMALIZATION_EPS = 1.0e-2
OVERWRITE_OUTPUT = True
VALIDATION_STEPS = 12
VALIDATION_SEED = 20261024
VALIDATION_ATOL = 1.0e-6
VALIDATION_RTOL = 1.0e-5


@dataclass(frozen=True)
class Architecture:
  obs_dim: int
  estimator_input_dim: int
  rnn_hidden_dim: int
  estimator_output_dim: int
  policy_input_dim: int
  hidden_dims: tuple[int, ...]
  action_dim: int


class EstimatePolicy(nn.Module):
  """Stateful deployment form of the MJLab HexapodEstimate Actor."""

  obs_mean: torch.Tensor
  obs_std: torch.Tensor
  hidden_state: torch.Tensor

  def __init__(
    self,
    mean: torch.Tensor,
    std: torch.Tensor,
    rnn: nn.GRU,
    estimator_head: nn.Linear,
    mlp: nn.Sequential,
    estimator_input_dim: int,
  ) -> None:
    super().__init__()
    self.register_buffer("obs_mean", mean.detach().clone().float())
    self.register_buffer("obs_std", std.detach().clone().float())
    self.rnn = rnn
    self.estimator_head = estimator_head
    self.mlp = mlp
    self.estimator_input_dim = int(estimator_input_dim)
    self.eps = OBS_NORMALIZATION_EPS
    self.register_buffer(
      "hidden_state",
      torch.zeros(rnn.num_layers, 1, rnn.hidden_size),
    )

  def forward(self, obs: torch.Tensor) -> torch.Tensor:
    normalized_obs = (obs - self.obs_mean) / (self.obs_std + self.eps)
    estimator_obs = normalized_obs[:, : self.estimator_input_dim].unsqueeze(0)
    latent, next_hidden = self.rnn(estimator_obs, self.hidden_state)
    self.hidden_state[:] = next_hidden
    estimate = self.estimator_head(latent.squeeze(0))
    policy_input = torch.cat((normalized_obs, estimate), dim=-1)
    return self.mlp(policy_input)

  @torch.jit.export
  def reset(self) -> None:
    """Clear recurrent state at an episode or real-policy boundary."""
    self.hidden_state.zero_()


def _require_tensor(
  state_dict: Mapping[str, object],
  key: str,
) -> torch.Tensor:
  value = state_dict.get(key)
  if not isinstance(value, torch.Tensor):
    raise RuntimeError(f"Checkpoint is missing tensor {key!r}")
  if not value.is_floating_point():
    raise RuntimeError(f"Checkpoint tensor {key!r} must be floating point")
  if not torch.isfinite(value).all().item():
    raise RuntimeError(f"Checkpoint tensor {key!r} contains NaN or infinity")
  return value


def _normalizer_tensors(
  state_dict: Mapping[str, object],
) -> tuple[torch.Tensor, torch.Tensor, int]:
  mean = _require_tensor(state_dict, "obs_normalizer._mean")
  std = _require_tensor(state_dict, "obs_normalizer._std")
  if mean.shape != std.shape or mean.ndim not in (1, 2):
    raise RuntimeError(
      "Observation normalizer mean/std must have the same shape "
      f"(obs_dim,) or (1, obs_dim), got {tuple(mean.shape)} and "
      f"{tuple(std.shape)}"
    )
  if mean.ndim == 2 and mean.shape[0] != 1:
    raise RuntimeError(
      f"Observation normalizer supports one feature row, got shape {tuple(mean.shape)}"
    )
  if mean.numel() < 1:
    raise RuntimeError("Observation dimension must be positive")
  if torch.any(std < 0).item():
    raise RuntimeError("Observation normalizer standard deviation is negative")
  obs_dim = int(mean.numel())
  return mean.reshape(1, obs_dim), std.reshape(1, obs_dim), obs_dim


def _linear_layer_ids(state_dict: Mapping[str, object]) -> list[int]:
  layer_ids = []
  for key in state_dict:
    if not key.startswith("mlp.") or not key.endswith(".weight"):
      continue
    parts = key.split(".")
    if len(parts) == 3 and parts[1].isdigit():
      layer_ids.append(int(parts[1]))
  layer_ids = sorted(set(layer_ids))
  if not layer_ids:
    raise RuntimeError("Checkpoint has no mlp.<index>.weight tensors")
  expected_ids = list(range(0, 2 * len(layer_ids), 2))
  if layer_ids != expected_ids:
    raise RuntimeError(
      "Only Linear/ELU/Linear MJLab MLPs are supported; expected layer "
      f"indices {expected_ids}, got {layer_ids}"
    )
  return layer_ids


def _build_policy(
  state_dict: Mapping[str, object],
) -> tuple[EstimatePolicy, Architecture]:
  mean, std, obs_dim = _normalizer_tensors(state_dict)

  expected_gru_keys = {
    "rnn.rnn.weight_ih_l0",
    "rnn.rnn.weight_hh_l0",
    "rnn.rnn.bias_ih_l0",
    "rnn.rnn.bias_hh_l0",
  }
  actual_gru_keys = {key for key in state_dict if key.startswith("rnn.rnn.")}
  if actual_gru_keys != expected_gru_keys:
    missing = sorted(expected_gru_keys - actual_gru_keys)
    unsupported = sorted(actual_gru_keys - expected_gru_keys)
    raise RuntimeError(
      "Only a one-layer, unidirectional GRU is supported; "
      f"missing={missing}, unsupported={unsupported}"
    )

  weight_ih = _require_tensor(state_dict, "rnn.rnn.weight_ih_l0")
  weight_hh = _require_tensor(state_dict, "rnn.rnn.weight_hh_l0")
  bias_ih = _require_tensor(state_dict, "rnn.rnn.bias_ih_l0")
  bias_hh = _require_tensor(state_dict, "rnn.rnn.bias_hh_l0")
  if weight_ih.ndim != 2 or weight_hh.ndim != 2:
    raise RuntimeError("GRU input and recurrent weights must be matrices")

  rnn_hidden_dim = int(weight_hh.shape[1])
  estimator_input_dim = int(weight_ih.shape[1])
  expected_gate_dim = 3 * rnn_hidden_dim
  if rnn_hidden_dim < 1 or estimator_input_dim < 1:
    raise RuntimeError("GRU input and hidden dimensions must be positive")
  if estimator_input_dim > obs_dim:
    raise RuntimeError(
      f"GRU input dimension {estimator_input_dim} exceeds obs_dim {obs_dim}"
    )
  if tuple(weight_ih.shape) != (expected_gate_dim, estimator_input_dim):
    raise RuntimeError(
      "Invalid GRU input weight shape: "
      f"got {tuple(weight_ih.shape)}, expected "
      f"({expected_gate_dim}, {estimator_input_dim})"
    )
  if tuple(weight_hh.shape) != (expected_gate_dim, rnn_hidden_dim):
    raise RuntimeError(
      "Invalid GRU recurrent weight shape: "
      f"got {tuple(weight_hh.shape)}, expected "
      f"({expected_gate_dim}, {rnn_hidden_dim})"
    )
  if tuple(bias_ih.shape) != (expected_gate_dim,) or tuple(bias_hh.shape) != (
    expected_gate_dim,
  ):
    raise RuntimeError(
      "GRU biases must both have shape "
      f"({expected_gate_dim},), got {tuple(bias_ih.shape)} and "
      f"{tuple(bias_hh.shape)}"
    )

  estimator_weight = _require_tensor(state_dict, "estimator_head.weight")
  estimator_bias = _require_tensor(state_dict, "estimator_head.bias")
  if estimator_weight.ndim != 2:
    raise RuntimeError("estimator_head.weight must be a matrix")
  estimator_output_dim = int(estimator_weight.shape[0])
  if tuple(estimator_weight.shape) != (estimator_output_dim, rnn_hidden_dim):
    raise RuntimeError(
      "Estimator head input must match GRU hidden dimension; got "
      f"{tuple(estimator_weight.shape)}, hidden={rnn_hidden_dim}"
    )
  if estimator_output_dim < 1 or tuple(estimator_bias.shape) != (estimator_output_dim,):
    raise RuntimeError(
      "Invalid estimator-head bias shape: "
      f"weight={tuple(estimator_weight.shape)}, "
      f"bias={tuple(estimator_bias.shape)}"
    )

  layer_ids = _linear_layer_ids(state_dict)
  mlp_layers: list[nn.Module] = []
  mlp_widths: list[int] = []
  policy_input_dim: int | None = None
  previous_out_dim: int | None = None
  for position, layer_id in enumerate(layer_ids):
    weight = _require_tensor(state_dict, f"mlp.{layer_id}.weight")
    bias = _require_tensor(state_dict, f"mlp.{layer_id}.bias")
    if weight.ndim != 2 or bias.ndim != 1:
      raise RuntimeError(f"mlp.{layer_id} must contain Linear tensors")
    out_dim, in_dim = map(int, weight.shape)
    if out_dim < 1 or in_dim < 1 or tuple(bias.shape) != (out_dim,):
      raise RuntimeError(
        f"Invalid mlp.{layer_id} shapes: weight={tuple(weight.shape)}, "
        f"bias={tuple(bias.shape)}"
      )
    if position == 0:
      policy_input_dim = in_dim
    elif in_dim != previous_out_dim:
      raise RuntimeError(
        f"MLP connection mismatch at mlp.{layer_id}: input={in_dim}, "
        f"previous output={previous_out_dim}"
      )

    linear = nn.Linear(in_dim, out_dim)
    with torch.no_grad():
      linear.weight.copy_(weight.float())
      linear.bias.copy_(bias.float())
    mlp_layers.append(linear)
    if position < len(layer_ids) - 1:
      mlp_layers.append(nn.ELU())
      mlp_widths.append(out_dim)
    previous_out_dim = out_dim

  assert policy_input_dim is not None
  assert previous_out_dim is not None
  action_dim = previous_out_dim
  expected_policy_input_dim = obs_dim + estimator_output_dim
  if policy_input_dim != expected_policy_input_dim:
    raise RuntimeError(
      "Actor input must concatenate normalized observations and the "
      f"estimate: got {policy_input_dim}, expected obs_dim + estimate_dim "
      f"= {obs_dim} + {estimator_output_dim} = {expected_policy_input_dim}"
    )

  for distribution_key in (
    "distribution.std_param",
    "distribution.log_std_param",
  ):
    if distribution_key not in state_dict:
      continue
    distribution_value = _require_tensor(state_dict, distribution_key)
    if distribution_value.numel() not in (1, action_dim):
      raise RuntimeError(
        f"{distribution_key} has {distribution_value.numel()} values, "
        f"incompatible with action_dim={action_dim}"
      )

  rnn = nn.GRU(estimator_input_dim, rnn_hidden_dim, num_layers=1)
  rnn.load_state_dict(
    {
      key.removeprefix("rnn.rnn."): _require_tensor(state_dict, key).float()
      for key in sorted(expected_gru_keys)
    },
    strict=True,
  )
  estimator_head = nn.Linear(rnn_hidden_dim, estimator_output_dim)
  with torch.no_grad():
    estimator_head.weight.copy_(estimator_weight.float())
    estimator_head.bias.copy_(estimator_bias.float())

  policy = EstimatePolicy(
    mean,
    std,
    rnn,
    estimator_head,
    nn.Sequential(*mlp_layers),
    estimator_input_dim,
  ).eval()
  architecture = Architecture(
    obs_dim=obs_dim,
    estimator_input_dim=estimator_input_dim,
    rnn_hidden_dim=rnn_hidden_dim,
    estimator_output_dim=estimator_output_dim,
    policy_input_dim=policy_input_dim,
    hidden_dims=tuple(mlp_widths),
    action_dim=action_dim,
  )
  return policy, architecture


def _validate_export(
  eager: EstimatePolicy,
  scripted_path: Path,
  architecture: Architecture,
) -> float:
  """Compare a recurrent sequence and reset behavior for eager and JIT."""
  obs_dim = architecture.obs_dim
  action_dim = architecture.action_dim
  scripted = torch.jit.load(str(scripted_path), map_location="cpu").eval()
  if not hasattr(scripted, "reset"):
    raise RuntimeError("Exported JIT model has no reset() method")

  generator = torch.Generator(device="cpu").manual_seed(VALIDATION_SEED)
  sequence = [
    torch.zeros(1, obs_dim),
    eager.obs_mean.detach().clone(),
  ]
  sequence.extend(
    torch.randn(1, obs_dim, generator=generator)
    for _ in range(max(VALIDATION_STEPS - len(sequence), 0))
  )

  eager.reset()
  scripted.reset()
  max_abs_error = 0.0
  first_scripted_output: torch.Tensor | None = None
  with torch.inference_mode():
    for frame, observation in enumerate(sequence):
      expected = eager(observation)
      actual = scripted(observation)
      if tuple(expected.shape) != (1, action_dim):
        raise RuntimeError(
          f"Eager frame {frame} returned {tuple(expected.shape)}, "
          f"expected (1, {action_dim})"
        )
      if tuple(actual.shape) != (1, action_dim):
        raise RuntimeError(
          f"JIT frame {frame} returned {tuple(actual.shape)}, "
          f"expected (1, {action_dim})"
        )
      if (
        not torch.isfinite(expected).all().item()
        or not torch.isfinite(actual).all().item()
      ):
        raise RuntimeError(f"Frame {frame} produced NaN or infinity")
      error = float(torch.max(torch.abs(expected - actual)).item())
      max_abs_error = max(max_abs_error, error)
      try:
        torch.testing.assert_close(
          actual,
          expected,
          atol=VALIDATION_ATOL,
          rtol=VALIDATION_RTOL,
        )
      except AssertionError as exc:
        raise RuntimeError(
          f"JIT mismatch at recurrent frame {frame}; max_abs_error={error:.3e}"
        ) from exc
      if frame == 0:
        first_scripted_output = actual.detach().clone()

    scripted.reset()
    reset_output = scripted(sequence[0])

  assert first_scripted_output is not None
  try:
    torch.testing.assert_close(
      reset_output,
      first_scripted_output,
      atol=VALIDATION_ATOL,
      rtol=VALIDATION_RTOL,
    )
  except AssertionError as exc:
    raise RuntimeError("JIT reset() did not restore the initial GRU state") from exc
  scripted.reset()
  return max_abs_error


def _load_actor_state(checkpoint_path: Path) -> Mapping[str, object]:
  try:
    checkpoint = torch.load(
      checkpoint_path,
      map_location="cpu",
      weights_only=True,
    )
  except TypeError:
    checkpoint = torch.load(checkpoint_path, map_location="cpu")
  if not isinstance(checkpoint, Mapping):
    raise RuntimeError("Checkpoint must be a mapping")
  actor_state = checkpoint.get("actor_state_dict")
  if not isinstance(actor_state, Mapping):
    raise RuntimeError("Checkpoint does not contain an actor_state_dict mapping")
  return actor_state


def main() -> None:
  checkpoint_path = CHECKPOINT_PATH.expanduser().resolve()
  output_path = POLICY_OUTPUT_PATH.expanduser().resolve()
  if not checkpoint_path.is_file():
    raise FileNotFoundError(f"Checkpoint does not exist: {checkpoint_path}")
  if checkpoint_path == output_path:
    raise ValueError("Checkpoint and output paths must be different")
  if output_path.exists() and not OVERWRITE_OUTPUT:
    raise FileExistsError(
      f"Output already exists: {output_path}. Set OVERWRITE_OUTPUT=True to replace it."
    )

  print(f"Loading checkpoint from: {checkpoint_path}")
  actor_state = _load_actor_state(checkpoint_path)
  policy, architecture = _build_policy(actor_state)
  print(
    "Inferred architecture: "
    f"obs={architecture.obs_dim}, "
    f"GRU={architecture.estimator_input_dim}->"
    f"{architecture.rnn_hidden_dim}, "
    f"estimate={architecture.estimator_output_dim}, "
    f"Actor={architecture.policy_input_dim}->"
    f"{list(architecture.hidden_dims)}->{architecture.action_dim}"
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
    scripted.reset()
    torch.jit.save(scripted, str(temporary_path))
    max_abs_error = _validate_export(policy, temporary_path, architecture)
    os.replace(temporary_path, output_path)
    output_path.chmod(0o644)
    temporary_path = None
  finally:
    if temporary_path is not None:
      temporary_path.unlink(missing_ok=True)

  print(
    "Validation passed over "
    f"{VALIDATION_STEPS} recurrent frames and reset; "
    f"max_abs_error={max_abs_error:.3e}"
  )
  print(f"Saved JIT model to: {output_path}")
  print(f"File size: {output_path.stat().st_size / (1024 * 1024):.2f} MiB")


if __name__ == "__main__":
  main()
