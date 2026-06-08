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
    k_d_forward_rpm_per_unit: float = 0.0
    k_d_backward_rpm_per_unit: float = 0.0

    # Integral gains: separate Ki for forward/backward projected motion.
    # Units: (RPM) per (tension_unit * second). The implementation integrates
    # error over time and multiplies by Ki.
    k_i_forward_rpm_per_unit: float = 0.0
    k_i_backward_rpm_per_unit: float = 0.0


    # Feedforward (paper-style):
    #   ff_rpm = (ff_max_rpm / ff_max_speed_mps) * (v_mps * cos(yaw)) / ff_radius_m
    ff_enabled: bool = False
    ff_max_rpm: float = 157.0
    ff_max_speed_mps: float = 0.8
    ff_radius_m: float = 0.07

    # Robustness / safety
    speed_limit_rpm: float = 157.0
    speed_deadband_rpm: float = 0.1
    tension_deadband: float = 0.0

    # Filtering and integrator clamping
    tension_lpf_alpha: float = 1.0
    Kff_forward: float = 0.0  # Feedforward scaling factor (paper-style), unitless multiplier on ff_rpm
    Kff_backward: float = 0.0  # Feedforward scaling factor (paper-style), unitless multiplier on ff_rpm

class TensionSpeedController:
    """Lightweight tension->speed controller.

    This class does *not* access any sensors; pass the needed scalars in.
    """

    def __init__(self, cfg: TensionSpeedControllerConfig):
        self.cfg = cfg
        self._prev_e: Optional[float] = None
        self._tension_meas_f: Optional[float] = None
        self._last_error: Optional[float] = None
        self._int_error: float = 0.0

    def reset(self) -> None:
        self._prev_e = None
        self._tension_meas_f = None
        self._last_error = None
        self._int_error = 0.0

    def step(
        self,
        *,
        cmd_speed_input: Optional[float] = None,
        imu_speed_input: Optional[float] = None,
        speed_input: Optional[float] = None,
        yaw: float = 0.0,
        tension_ref: float,
        tension_meas: float,
    ) -> float:
        """Compute spool target speed in RPM.

        Args:
            cmd_speed_input: Commanded forward speed used for low-speed feedforward.
            imu_speed_input: IMU-measured forward speed used after startup.
            speed_input: Backward-compatible velocity input. Used as command speed
                if cmd_speed_input is not provided.
            tension_ref: Reference/desired cable tension (e.g., from policy output).
            tension_meas: Measured/actual cable tension.

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

        if self._tension_meas_f <= 1.0:
            self._tension_meas_f = 0.0
            
        tension_ref = float(tension_ref)
        e = tension_ref - float(self._tension_meas_f)

        # Optional deadband on tension error.
        if abs(e) < float(self.cfg.tension_deadband):
            e = 0.0

        self._prev_e = e

        # Compose speed command
        yaw_rad = float(yaw)

        if cmd_speed_input is None:
            if speed_input is None:
                raise ValueError("cmd_speed_input or speed_input must be provided")
            cmd_speed_input = speed_input
        if imu_speed_input is None:
            imu_speed_input = cmd_speed_input

        cmd_vx = float(cmd_speed_input)
        imu_vx = float(imu_speed_input)
        if abs(imu_vx) < 0.1:
            v_mps = cmd_vx
        else:
            v_mps = 0.8 * imu_vx + 0.2 * cmd_vx
        v_proj = v_mps * math.cos(yaw_rad)

        # Select Kp&Kd based on forward/backward projected motion.
        if v_proj >= 0.0:
            kp = float(self.cfg.k_p_forward_rpm_per_unit)
            kd = float(self.cfg.k_d_forward_rpm_per_unit)
            ki = float(self.cfg.k_i_forward_rpm_per_unit)
        else:
            kp = float(self.cfg.k_p_backward_rpm_per_unit)
            kd = float(self.cfg.k_d_backward_rpm_per_unit)
            ki = float(self.cfg.k_i_backward_rpm_per_unit)
        dt_fixed = 0.005
        derivative = 0.0
        if self._last_error is not None:
            derivative = (e - self._last_error) / dt_fixed
        # Integral (simple anti-windup): tentatively integrate, then keep the
        # update only if command is not saturating.
        int_candidate = self._int_error
        if ki != 0.0  and e != 0.0:
            int_candidate = self._int_error + e * dt_fixed

        p_term = kp * e
        d_term = kd * derivative
        i_term = ki * int_candidate
        self._last_error = e

        Kff_forward = float(self.cfg.Kff_forward)
        Kff_backward = float(self.cfg.Kff_backward)
        # 决定是否启用 前馈
        if bool(self.cfg.ff_enabled):
            rad_rpm = (60 / (2 * math.pi))
            if v_proj >= 0.0 :
                ff_rpm = (
                    Kff_forward
                    *rad_rpm
                    * v_proj
                    / float(self.cfg.ff_radius_m)
                )
            else:
                ff_rpm = (
                    Kff_backward
                    *rad_rpm
                    * v_proj
                    / float(self.cfg.ff_radius_m)
                )                    
        else:
            ff_rpm = 0.0

        feedback_rpm = p_term + d_term + i_term
        speed_rpm_unsat = ff_rpm + feedback_rpm

        # Apply sign convention before deadband/saturation.
        speed_rpm = speed_rpm_unsat * float(self.cfg.speed_sign)

        # Deadband + saturation
        if abs(speed_rpm) < float(self.cfg.speed_deadband_rpm):
            speed_rpm = 0.0

        lim = abs(float(self.cfg.speed_limit_rpm))
        speed_rpm_sat = _clip(speed_rpm, -lim, lim)

        # If saturating, reject integral update (prevents windup).
        if abs(speed_rpm_sat - speed_rpm) <= 1e-9:
            self._int_error = int_candidate
        speed_rpm = speed_rpm_sat
        #return speed_rpm
        return speed_rpm,feedback_rpm, ff_rpm
