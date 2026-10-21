"""Stateful inference wrapper for an MJLab Hexapod estimate policy."""

from __future__ import annotations

from collections.abc import Mapping
from pathlib import Path
from typing import Literal, cast
from zipfile import ZipFile, is_zipfile

import numpy as np
import torch
from numpy.typing import ArrayLike, NDArray
from torch import nn

from .observation import (
  ACTION_DIM,
  ESTIMATOR_INPUT_DIM,
  ESTIMATOR_OUTPUT_DIM,
  OBSERVATION_DIM,
)

_HIDDEN_DIM = 128
_POLICY_INPUT_DIM = OBSERVATION_DIM + ESTIMATOR_OUTPUT_DIM
_NORMALIZER_EPS = 0.01

CheckpointFormat = Literal["auto", "mjlab", "torchscript"]
ActionClip = float | tuple[float, float] | None


class _RawEstimateCore(nn.Module):
  """Network topology used by ``HexapodEstimateActor`` at deployment."""

  obs_mean: torch.Tensor
  obs_std: torch.Tensor

  def __init__(self) -> None:
    super().__init__()
    self.register_buffer("obs_mean", torch.zeros(1, OBSERVATION_DIM))
    self.register_buffer("obs_std", torch.ones(1, OBSERVATION_DIM))
    self.gru = nn.GRU(
      input_size=ESTIMATOR_INPUT_DIM,
      hidden_size=_HIDDEN_DIM,
      num_layers=1,
    )
    self.estimator_head = nn.Linear(_HIDDEN_DIM, ESTIMATOR_OUTPUT_DIM)
    self.mlp = nn.Sequential(
      nn.Linear(_POLICY_INPUT_DIM, 512),
      nn.ELU(),
      nn.Linear(512, 256),
      nn.ELU(),
      nn.Linear(256, 128),
      nn.ELU(),
      nn.Linear(128, ACTION_DIM),
    )

  def forward(
    self,
    observation: torch.Tensor,
    hidden_state: torch.Tensor,
  ) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
    normalized = (observation - self.obs_mean) / (self.obs_std + _NORMALIZER_EPS)
    recurrent_input = normalized[..., :ESTIMATOR_INPUT_DIM].unsqueeze(0)
    recurrent_output, next_hidden = self.gru(recurrent_input, hidden_state)
    estimate = self.estimator_head(recurrent_output.squeeze(0))
    policy_input = torch.cat((normalized, estimate), dim=-1)
    action = self.mlp(policy_input)
    return action, estimate, next_hidden


_EXPECTED_SHAPES: dict[str, tuple[int, ...]] = {
  "obs_normalizer._mean": (1, OBSERVATION_DIM),
  "obs_normalizer._std": (1, OBSERVATION_DIM),
  "rnn.rnn.weight_ih_l0": (3 * _HIDDEN_DIM, ESTIMATOR_INPUT_DIM),
  "rnn.rnn.weight_hh_l0": (3 * _HIDDEN_DIM, _HIDDEN_DIM),
  "rnn.rnn.bias_ih_l0": (3 * _HIDDEN_DIM,),
  "rnn.rnn.bias_hh_l0": (3 * _HIDDEN_DIM,),
  "estimator_head.weight": (ESTIMATOR_OUTPUT_DIM, _HIDDEN_DIM),
  "estimator_head.bias": (ESTIMATOR_OUTPUT_DIM,),
  "mlp.0.weight": (512, _POLICY_INPUT_DIM),
  "mlp.0.bias": (512,),
  "mlp.2.weight": (256, 512),
  "mlp.2.bias": (256,),
  "mlp.4.weight": (128, 256),
  "mlp.4.bias": (128,),
  "mlp.6.weight": (ACTION_DIM, 128),
  "mlp.6.bias": (ACTION_DIM,),
}


def _load_torch_payload(path: Path, device: torch.device) -> object:
  try:
    return torch.load(path, map_location=device, weights_only=True)
  except TypeError:
    # ``weights_only`` is unavailable on older PyTorch releases.
    return torch.load(path, map_location=device)


def _detect_checkpoint_format(path: Path) -> Literal["mjlab", "torchscript"]:
  """Inspect archive records without loading the policy or trusting its name."""
  if is_zipfile(path):
    with ZipFile(path) as archive:
      if any(
        name == "constants.pkl" or name.endswith("/constants.pkl")
        for name in archive.namelist()
      ):
        return "torchscript"
  return "mjlab"


def _validate_actor_state(payload: object) -> Mapping[str, torch.Tensor]:
  if not isinstance(payload, Mapping):
    raise ValueError("MJLab checkpoint must be a mapping")
  if "actor_state_dict" not in payload:
    raise ValueError(
      "Checkpoint has no 'actor_state_dict'; pass a direct MJLab model_*.pt "
      "checkpoint, not an exported or ordinary Hexapod policy"
    )
  actor_state = payload["actor_state_dict"]
  if not isinstance(actor_state, Mapping):
    raise ValueError("'actor_state_dict' must be a mapping")

  validated: dict[str, torch.Tensor] = {}
  for key, expected_shape in _EXPECTED_SHAPES.items():
    if key not in actor_state:
      raise ValueError(f"Checkpoint Actor is missing required tensor {key!r}")
    value = actor_state[key]
    if not isinstance(value, torch.Tensor):
      raise ValueError(f"Checkpoint value {key!r} is not a tensor")
    if tuple(value.shape) != expected_shape:
      raise ValueError(
        f"Checkpoint tensor {key!r} must have shape {expected_shape}, "
        f"got {tuple(value.shape)}"
      )
    if not value.is_floating_point():
      raise ValueError(f"Checkpoint tensor {key!r} must be floating point")
    if not torch.isfinite(value).all().item():
      raise ValueError(f"Checkpoint tensor {key!r} contains NaN or infinity")
    validated[key] = value

  expected_namespaces = {
    "rnn.rnn.": {key for key in _EXPECTED_SHAPES if key.startswith("rnn.rnn.")},
    "estimator_head.": {
      key for key in _EXPECTED_SHAPES if key.startswith("estimator_head.")
    },
    "mlp.": {key for key in _EXPECTED_SHAPES if key.startswith("mlp.")},
  }
  actor_keys = {str(key) for key in actor_state}
  for prefix, expected_keys in expected_namespaces.items():
    actual_keys = {key for key in actor_keys if key.startswith(prefix)}
    unexpected = sorted(actual_keys - expected_keys)
    if unexpected:
      raise ValueError(
        f"Checkpoint Actor has an unsupported {prefix!r} layout: {unexpected}"
      )

  if torch.any(validated["obs_normalizer._std"] < 0).item():
    raise ValueError("Checkpoint observation standard deviation is negative")
  return validated


def _copy_prefixed_state(
  module: nn.Module,
  actor_state: Mapping[str, torch.Tensor],
  checkpoint_prefix: str,
) -> None:
  prefix_length = len(checkpoint_prefix)
  module_state = {
    key[prefix_length:]: value
    for key, value in actor_state.items()
    if key.startswith(checkpoint_prefix)
  }
  module.load_state_dict(module_state, strict=True)


def _build_raw_core(
  path: Path,
  device: torch.device,
) -> _RawEstimateCore:
  payload = _load_torch_payload(path, device)
  actor_state = _validate_actor_state(payload)
  core = _RawEstimateCore()
  with torch.no_grad():
    core.obs_mean.copy_(actor_state["obs_normalizer._mean"])
    core.obs_std.copy_(actor_state["obs_normalizer._std"])
  _copy_prefixed_state(core.gru, actor_state, "rnn.rnn.")
  _copy_prefixed_state(core.estimator_head, actor_state, "estimator_head.")
  _copy_prefixed_state(core.mlp, actor_state, "mlp.")
  return core.to(device).eval()


def _normalize_clip(action_clip: ActionClip) -> tuple[float, float] | None:
  if action_clip is None:
    return None
  if isinstance(action_clip, bool):
    raise ValueError("action_clip must not be a boolean")
  if isinstance(action_clip, (int, float)):
    magnitude = float(action_clip)
    if not np.isfinite(magnitude) or magnitude <= 0:
      raise ValueError("Scalar action_clip must be finite and positive")
    return (-magnitude, magnitude)
  if len(action_clip) != 2:
    raise ValueError("Tuple action_clip must contain (lower, upper)")
  lower, upper = (float(bound) for bound in action_clip)
  if not np.isfinite((lower, upper)).all() or lower > upper:
    raise ValueError("Action-clip bounds must be finite and lower <= upper")
  return lower, upper


class HexapodEstimatePolicy:
  """Run one stateful estimate policy for one robot environment.

  ``auto`` detects a raw checkpoint or TorchScript archive from its contents.
  ``mjlab`` loads a raw training checkpoint containing ``actor_state_dict``.
  ``torchscript`` loads MJLab's stateful export and requires its exported
  ``reset()`` method. Call :meth:`reset` at every simulation or robot episode
  boundary. The policy output remains unclipped unless ``action_clip`` is set.
  """

  def __init__(
    self,
    checkpoint: str | Path,
    *,
    device: str | torch.device = "cpu",
    checkpoint_format: CheckpointFormat = "auto",
    action_clip: ActionClip = None,
  ) -> None:
    self.path = Path(checkpoint).expanduser().resolve()
    if not self.path.is_file():
      raise FileNotFoundError(f"Policy checkpoint not found: {self.path}")
    if checkpoint_format == "auto":
      checkpoint_format = _detect_checkpoint_format(self.path)
    self.device = torch.device(device)
    self.checkpoint_format = checkpoint_format
    self._clip_bounds = _normalize_clip(action_clip)
    self._raw_core: _RawEstimateCore | None = None
    self._scripted: torch.jit.ScriptModule | None = None
    self._last_estimate: NDArray[np.float32] | None = None
    self._hidden = torch.zeros(1, 1, _HIDDEN_DIM, device=self.device)

    if checkpoint_format == "mjlab":
      self._raw_core = _build_raw_core(self.path, self.device)
    elif checkpoint_format == "torchscript":
      scripted = torch.jit.load(str(self.path), map_location=self.device)
      scripted = scripted.to(self.device).eval()
      if not hasattr(scripted, "reset"):
        raise ValueError(
          "The selected TorchScript policy is not a stateful HexapodEstimate "
          "export: reset() is missing. Use a HexapodEstimate model with a "
          "63-D observation and a GRU; an ordinary Hexapod policy is incompatible."
        )
      self._scripted = scripted
      self._validate_scripted_contract()
    else:
      raise ValueError(
        "checkpoint_format must be 'auto', 'mjlab', or 'torchscript', got "
        f"{checkpoint_format!r}"
      )
    self.reset()

  @property
  def last_estimate(self) -> NDArray[np.float32] | None:
    """Return a copy of ``[vx, vy, vz, h_RF, ..., h_LB]`` for raw models."""
    if self._last_estimate is None:
      return None
    return self._last_estimate.copy()

  def _validate_scripted_contract(self) -> None:
    assert self._scripted is not None
    try:
      with torch.inference_mode():
        self._scripted.reset()
        output = self._scripted(torch.zeros(1, OBSERVATION_DIM, device=self.device))
        self._scripted.reset()
    except Exception as exc:
      raise ValueError(
        "TorchScript policy does not satisfy the 63-D stateful estimate policy contract"
      ) from exc
    if not isinstance(output, torch.Tensor):
      raise ValueError("TorchScript policy output must be a tensor")
    if tuple(output.shape) != (1, ACTION_DIM):
      raise ValueError(
        "TorchScript policy output must have shape "
        f"(1, {ACTION_DIM}), got {tuple(output.shape)}"
      )
    if not torch.isfinite(output).all().item():
      raise ValueError("TorchScript policy produced NaN or infinity")

  def reset(self) -> None:
    """Clear the GRU memory and the last exposed estimate."""
    self._hidden.zero_()
    self._last_estimate = None
    if self._scripted is not None:
      with torch.inference_mode():
        self._scripted.reset()

  def predict(self, observation: ArrayLike) -> NDArray[np.float32]:
    """Advance the GRU once and return one raw 18-dimensional action."""
    try:
      observation_array = np.asarray(observation, dtype=np.float32)
    except (TypeError, ValueError) as exc:
      raise ValueError("observation must be convertible to float32") from exc
    if observation_array.shape != (OBSERVATION_DIM,):
      raise ValueError(
        f"observation must have shape ({OBSERVATION_DIM},), "
        f"got {observation_array.shape}"
      )
    if not np.isfinite(observation_array).all():
      raise ValueError("observation contains a NaN or infinity")

    tensor = torch.from_numpy(observation_array).to(self.device).unsqueeze(0)
    with torch.inference_mode():
      if self._raw_core is not None:
        action, estimate, next_hidden = self._raw_core(tensor, self._hidden)
        self._hidden.copy_(next_hidden)
        self._last_estimate = cast(
          NDArray[np.float32],
          estimate.squeeze(0).cpu().numpy().astype(np.float32, copy=True),
        )
      else:
        assert self._scripted is not None
        action = self._scripted(tensor)

    if not isinstance(action, torch.Tensor):
      raise RuntimeError("Policy output is not a tensor")
    if tuple(action.shape) != (1, ACTION_DIM):
      raise RuntimeError(
        f"Policy output must have shape (1, {ACTION_DIM}), got {tuple(action.shape)}"
      )
    if not torch.isfinite(action).all().item():
      raise RuntimeError("Policy output contains a NaN or infinity")
    result = cast(
      NDArray[np.float32],
      action.squeeze(0).cpu().numpy().astype(np.float32, copy=True),
    )
    if self._clip_bounds is not None:
      lower, upper = self._clip_bounds
      np.clip(result, lower, upper, out=result)
    return result

  def warm_up(self, steps: int = 10) -> None:
    """Exercise inference without retaining artificial recurrent history."""
    if isinstance(steps, bool) or not isinstance(steps, int) or steps < 1:
      raise ValueError("steps must be a positive integer")
    try:
      observation = np.zeros(OBSERVATION_DIM, dtype=np.float32)
      for _ in range(steps):
        self.predict(observation)
    finally:
      self.reset()
