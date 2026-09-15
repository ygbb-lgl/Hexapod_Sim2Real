"""Validated configuration for recurrent tethered-hexapod MoE deployment."""

from __future__ import annotations

from pathlib import Path
from typing import Dict

import numpy as np
import yaml
from config_hexapod_tethered import Config
from hexapod_tethered_utils.moe_policy import (
    ACTION_DIM,
    CONTACT_LATENT_DIM,
    EXPLICIT_CONTEXT_DIM,
    HIDDEN_DIM,
    NUM_EXPERTS,
    STATE_DIM,
)

EXPECTED_OBSERVATION_LAYOUT = {
    "gyro": 3,
    "gravity": 3,
    "leg_joint_position_error": 18,
    "leg_joint_velocity": 18,
    "previous_action": 19,
    "command": 3,
    "cable_tension": 1,
    "tether_unit_direction_body": 3,
}


class MoEConfig(Config):
    """Load the common hardware settings and validate the MoE contract."""

    def __init__(self, file_path: str) -> None:
        super().__init__(file_path)
        with Path(file_path).open("r", encoding="utf-8") as stream:
            raw = yaml.safe_load(stream)

        moe = raw.get("moe")
        if not isinstance(moe, dict):
            raise ValueError("MoE deployment YAML must contain a 'moe' mapping")

        self.policy_device = str(moe.get("policy_device", "cpu"))
        self.policy_action_clip = float(moe["policy_action_clip"])
        self.cable_action_range = np.asarray(
            moe["cable_action_range"], dtype=np.float32
        )
        self.estimator_input_dim = int(moe["estimator_input_dim"])
        self.explicit_context_dim = int(moe["explicit_context_dim"])
        self.contact_latent_dim = int(moe["contact_latent_dim"])
        self.actor_input_dim = int(moe["actor_input_dim"])
        self.gru_hidden_dim = int(moe["gru_hidden_dim"])
        self.num_experts = int(moe["num_experts"])
        self.tether_direction_frame = str(moe["tether_direction_frame"])
        self.tether_angle_unit = str(moe["tether_angle_unit"])
        self.observation_normalization = str(
            moe["observation_normalization"]
        )
        self.observation_layout = self._read_layout(moe)
        self.command_scale = np.asarray(self.command_scale, dtype=np.float32)

        self._validate_moe_contract()

    @staticmethod
    def _read_layout(moe: dict) -> Dict[str, int]:
        layout = moe.get("observation_layout")
        if not isinstance(layout, dict):
            raise ValueError("moe.observation_layout must be a mapping")
        return {str(name): int(width) for name, width in layout.items()}

    def _validate_moe_contract(self) -> None:
        expected_scalars = {
            "num_obs": STATE_DIM,
            "num_actions": ACTION_DIM,
            "num_leggeds_actions": ACTION_DIM - 1,
            "estimator_input_dim": STATE_DIM,
            "explicit_context_dim": EXPLICIT_CONTEXT_DIM,
            "contact_latent_dim": CONTACT_LATENT_DIM,
            "actor_input_dim": STATE_DIM
            + EXPLICIT_CONTEXT_DIM
            + CONTACT_LATENT_DIM,
            "gru_hidden_dim": HIDDEN_DIM,
            "num_experts": NUM_EXPERTS,
        }
        mismatches = []
        for name, expected in expected_scalars.items():
            actual = int(getattr(self, name))
            if actual != expected:
                mismatches.append(f"{name}={actual}, expected {expected}")

        if list(self.observation_layout.items()) != list(
            EXPECTED_OBSERVATION_LAYOUT.items()
        ):
            mismatches.append(
                "observation_layout does not match the ordered 68-D training state"
            )
        layout_width = sum(self.observation_layout.values())
        if layout_width != self.num_obs:
            mismatches.append(
                f"observation_layout sums to {layout_width}, num_obs={self.num_obs}"
            )
        if self.command_scale.shape != (3,) or not np.all(
            np.isfinite(self.command_scale)
        ):
            mismatches.append("command_scale must contain three finite values")
        if not np.isfinite(self.policy_action_clip) or self.policy_action_clip <= 0:
            mismatches.append("policy_action_clip must be finite and positive")
        if (
            self.cable_action_range.shape != (2,)
            or not np.all(np.isfinite(self.cable_action_range))
            or self.cable_action_range[0] >= self.cable_action_range[1]
        ):
            mismatches.append(
                "cable_action_range must contain finite increasing [min, max]"
            )
        elif not np.allclose(self.cable_action_range, [0.0, 2.0]):
            mismatches.append(
                "cable_action_range must match the training range [0.0, 2.0]"
            )
        if self.tether_direction_frame != "trunk":
            mismatches.append("tether_direction_frame must be 'trunk'")
        if self.tether_angle_unit != "radian":
            mismatches.append("tether_angle_unit must be 'radian'")
        if self.observation_normalization != "checkpoint":
            mismatches.append("observation_normalization must be 'checkpoint'")
        if float(self.action_scale) != 0.25:
            mismatches.append(
                f"action_scale={self.action_scale}, expected training value 0.25"
            )
        if float(self.tension_action_scale) != 200.0:
            mismatches.append(
                "tension_action_scale="
                f"{self.tension_action_scale}, expected training value 200"
            )

        if mismatches:
            raise ValueError("Invalid MoE deployment config: " + "; ".join(mismatches))
