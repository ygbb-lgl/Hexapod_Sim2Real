from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Optional


def _clip(value: float, low: float, high: float) -> float:
    return low if value < low else high if value > high else value


@dataclass(frozen=True)
class TensionSpeedControllerConfig:
    """Configuration for reference-tension tracking via spool speed.

    All gains are expressed in *RPM per unit* so that the output can be written
    directly to `robot.spool_command_buffer.target_speed_rpm[0]`.

    Conceptual law (paper-style P + feedforward):
        e = T_ref - T_meas
        speed_rpm = speed_sign * (ff(v, yaw) + kp(v, yaw) * e)

    Notes:
    - Units of `v`, `T_ref`, `T_meas` are user-defined; keep them consistent with gains.
    - `tension_lpf_alpha=1.0` disables filtering.
    """

    # Sign to match your hardware convention (positive RPM -> tension increase or decrease)
    speed_sign: float = 1.0

    # Feedback gains (paper-style): use different Kp for forward/backward motion.
    # Forward/backward is decided by the sign of v_proj = v * cos(yaw).
    k_p_forward_rpm_per_unit: float = 0.0
    k_p_backward_rpm_per_unit: float = 0.0

    # Feedforward (paper-style):
    #   ff_rpm = (ff_max_rpm / ff_max_speed_mps) * (v_mps * cos(yaw)) / ff_radius_m
    ff_enabled: bool = False
    ff_max_rpm: float = 157.0
    ff_max_speed_mps: float = 0.8
    ff_radius_m: float = 0.15

    # Robustness / safety
    speed_limit_rpm: float = 600.0
    speed_deadband_rpm: float = 0.1
    tension_deadband: float = 0.0

    # Filtering and integrator clamping
    tension_lpf_alpha: float = 1.0
    Kff: float = 1.0  # Feedforward scaling factor (paper-style), unitless multiplier on ff_rpm


class TensionSpeedController:
    """Lightweight tension->speed controller.

    This class does *not* access any sensors; pass the needed scalars in.
    """

    def __init__(self, cfg: TensionSpeedControllerConfig):
        self.cfg = cfg
        self._prev_e: Optional[float] = None
        self._tension_meas_f: Optional[float] = None

    def reset(self) -> None:
        self._prev_e = None
        self._tension_meas_f = None

    def step(
        self,
        *,
        speed_input: float,
        yaw: float = 0.0,
        tension_ref: float,
        tension_meas: float,
    ) -> float:
        """Compute spool target speed in RPM.

        Args:
            speed_input: A velocity-like scalar used for feedforward (e.g., cmd vx).
            tension_ref: Reference/desired cable tension (e.g., from policy output).
            tension_meas: Measured/actual cable tension.
            dt: Control timestep in seconds.

        Returns:
            target_speed_rpm (float)
        """

        # Clamp/filter tension measurement for stability.
        alpha = _clip(float(self.cfg.tension_lpf_alpha), 0.0, 1.0)
        tension_meas = float(tension_meas)
        if self._tension_meas_f is None or alpha >= 1.0:
            self._tension_meas_f = tension_meas
        else:
            self._tension_meas_f = alpha * tension_meas + (1.0 - alpha) * self._tension_meas_f

        tension_ref = float(tension_ref)
        e = tension_ref - float(self._tension_meas_f)

        # Optional deadband on tension error.
        if abs(e) < float(self.cfg.tension_deadband):
            e = 0.0

        self._prev_e = e

        # Compose speed command
        yaw_rad = float(yaw)

        v_mps = float(speed_input)
        v_proj = v_mps * math.cos(yaw_rad)

        # Select Kp based on forward/backward projected motion.
        if v_proj >= 0.0:
            kp = float(self.cfg.k_p_forward_rpm_per_unit)
        else:
            kp = float(self.cfg.k_p_backward_rpm_per_unit)

        feedback_rpm = kp * e

        Kff = float(self.cfg.Kff)
        # 决定是否启用 前馈
        if bool(self.cfg.ff_enabled):
            rad_rpm = (60 / (2 * math.pi))
            ff_rpm = (
                Kff
                *rad_rpm
                * v_proj
                / float(self.cfg.ff_radius_m)
            )
        else:
            ff_rpm = 0.0

        speed_rpm = ff_rpm + feedback_rpm

        speed_rpm *= float(self.cfg.speed_sign)

        # Deadband + saturation
        if abs(speed_rpm) < float(self.cfg.speed_deadband_rpm):
            speed_rpm = 0.0

        lim = abs(float(self.cfg.speed_limit_rpm))
        speed_rpm = _clip(speed_rpm, -lim, lim)
        return speed_rpm
