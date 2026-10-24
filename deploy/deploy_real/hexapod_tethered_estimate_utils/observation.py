"""Canonical observation layout for tethered-estimator deployment."""

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
ACTION_DIM = JOINT_DIM + 1
COMMAND_DIM = 3
TETHER_DIRECTION_DIM = 3
OBSERVATION_DIM = 68
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


def tether_direction_from_yaw_pitch(
  yaw: float,
  pitch: float,
) -> NDArray[np.float32]:
  """Reconstruct the trunk-frame unit tether direction used in training.

  Positive yaw follows the ARM joint's ``(0, 0, -1)`` axis and therefore
  rotates trunk ``+x`` toward trunk ``-y``. Positive pitch points toward
  trunk ``+z``. Both inputs are radians.
  """
  yaw_value = float(yaw)
  pitch_value = float(pitch)
  if not np.isfinite((yaw_value, pitch_value)).all():
    raise ValueError("yaw and pitch must be finite")
  horizontal = np.cos(pitch_value)
  return np.asarray(
    (
      horizontal * np.cos(yaw_value),
      -horizontal * np.sin(yaw_value),
      np.sin(pitch_value),
    ),
    dtype=np.float32,
  )


def build_observation(  # noqa: PLR0917 - fields define the policy wire ABI.
  angular_velocity: ArrayLike,
  projected_gravity: ArrayLike,
  joint_position: ArrayLike,
  joint_velocity: ArrayLike,
  previous_action: ArrayLike,
  command: ArrayLike,
  cable_tension: float,
  tether_direction_body: ArrayLike,
) -> NDArray[np.float32]:
  """Build the exact 68-value Actor observation used during MJLab training.

  The order is angular velocity, projected gravity, relative leg position,
  leg velocity, previous 19-D raw policy action, velocity command, positive
  cable tension, and the trunk-frame tether direction. The GRU consumes only
  the first 60 values, ending after the 18 previous leg actions.
  """
  tension = float(cable_tension)
  if not np.isfinite(tension):
    raise ValueError("cable_tension must be finite")
  if tension < 0.0:
    raise ValueError("cable_tension must be non-negative")

  fields: Sequence[tuple[str, ArrayLike, int]] = (
    ("angular_velocity", angular_velocity, 3),
    ("projected_gravity", projected_gravity, 3),
    ("joint_position", joint_position, JOINT_DIM),
    ("joint_velocity", joint_velocity, JOINT_DIM),
    ("previous_action", previous_action, ACTION_DIM),
    ("command", command, COMMAND_DIM),
    ("cable_tension", np.asarray((tension,), dtype=np.float32), 1),
    ("tether_direction_body", tether_direction_body, TETHER_DIRECTION_DIM),
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


build_actor_observation = build_observation

if __name__ == "__main__":
  tether_direction = tether_direction_from_yaw_pitch(0.0, 0.0)
  print("tether_direction_from_yaw_pitch(0.0, 0.0) =", tether_direction)