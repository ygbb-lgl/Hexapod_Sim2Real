"""Canonical observation layout shared by simulation and real deployment."""

from __future__ import annotations

from collections.abc import Sequence

import numpy as np
from numpy.typing import ArrayLike, NDArray

JOINT_NAMES = (
  "RF_HAA",
  "RF_HFE",
  "RF_KFE",
  "RM_HAA",
  "RM_HFE",
  "RM_KFE",
  "RB_HAA",
  "RB_HFE",
  "RB_KFE",
  "LF_HAA",
  "LF_HFE",
  "LF_KFE",
  "LM_HAA",
  "LM_HFE",
  "LM_KFE",
  "LB_HAA",
  "LB_HFE",
  "LB_KFE",
)

JOINT_DIM = len(JOINT_NAMES)
ACTION_DIM = JOINT_DIM
COMMAND_DIM = 3
OBSERVATION_DIM = 63
ESTIMATOR_INPUT_DIM = 60
ESTIMATOR_OUTPUT_DIM = 9


def _as_vector(name: str, value: ArrayLike, size: int) -> NDArray[np.float32]:
  try:
    vector = np.asarray(value, dtype=np.float32)
  except (TypeError, ValueError) as exc:
    raise ValueError(f"{name} must be convertible to float32") from exc
  if vector.shape != (size,):
    raise ValueError(f"{name} must have shape ({size},), got {vector.shape}")
  if not np.isfinite(vector).all():
    raise ValueError(f"{name} contains a NaN or infinity")
  return vector


def build_observation(  # noqa: PLR0917 - six fields define the wire contract.
  angular_velocity: ArrayLike,
  projected_gravity: ArrayLike,
  joint_position: ArrayLike,
  joint_velocity: ArrayLike,
  previous_action: ArrayLike,
  command: ArrayLike,
) -> NDArray[np.float32]:
  """Build the exact 63-value Actor observation used during MJLab training.

  The order is angular velocity, projected gravity, relative joint position,
  relative joint velocity, previous raw policy action, then velocity command.
  The first 60 values are the GRU input; the command is not recurrent input.
  """
  fields: Sequence[tuple[str, ArrayLike, int]] = (
    ("angular_velocity", angular_velocity, 3),
    ("projected_gravity", projected_gravity, 3),
    ("joint_position", joint_position, JOINT_DIM),
    ("joint_velocity", joint_velocity, JOINT_DIM),
    ("previous_action", previous_action, ACTION_DIM),
    ("command", command, COMMAND_DIM),
  )
  observation = np.concatenate(
    [_as_vector(name, value, size) for name, value, size in fields]
  )
  if observation.shape != (OBSERVATION_DIM,):
    raise RuntimeError(
      "Internal observation-layout error: expected "
      f"{OBSERVATION_DIM}, got {observation.shape}"
    )
  return observation.astype(np.float32, copy=False)


# Kept as a descriptive alias for callers that prefer the longer name.
build_actor_observation = build_observation
