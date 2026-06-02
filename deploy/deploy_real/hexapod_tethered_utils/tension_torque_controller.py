from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Optional


def _clip(value: float, low: float, high: float) -> float:
    return low if value < low else high if value > high else value


@dataclass(frozen=True)
class TensionTorqueControllerConfig:
    """Configuration for reference-tension tracking via spool torque.

    All gains are expressed in *Nm per unit* so that the output can be written
    directly to `robot.spool_command_buffer.target_torque_nm[0]`.

    Conceptual law (paper-style P + feedforward):
        e = T_ref - T_meas
        torque_nm = speed_sign * (ff(v, yaw) + kp(v, yaw) * e)

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
    ff_radius_m: float = 0.07

    # Robustness / safety
    tension_deadband: float = 0.0

    torque_limit: float = 90.0  # Nm, saturation limit for output torque command
    torque_deadband: float = 0.0  # Nm, deadband for output torque command
    # Filtering and integrator clamping
    tension_lpf_alpha: float = 1.0
    Kff_forward: float = 0.0  # Feedforward scaling factor (paper-style), unitless multiplier on ff_rpm
    Kff_backward: float = 0.0  # Feedforward scaling factor (paper-style), unitless multiplier on ff_rpm

class TensionTorqueController:
    """Lightweight tension->torque controller.

    This class does *not* access any sensors; pass the needed scalars in.
    """

    def __init__(self, cfg: TensionTorqueControllerConfig):
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
        speed_input: float,
        yaw: float = 0.0,
        tension_ref: float,
        tension_meas: float,
    ) -> float:
        """Compute spool target torque in Nm.

        Args:
            speed_input: A velocity-like scalar used for feedforward (e.g., cmd vx).
            tension_ref: Reference/desired cable tension (e.g., from policy output).
            tension_meas: Measured/actual cable tension.

        Returns:
            target_torque_nm (float)
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

        v_mps = float(speed_input)
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
            if v_proj >= 0.0 :
                ff_torque_nm = (
                    Kff_forward
                    * tension_ref
                    * float(self.cfg.ff_radius_m)
                )
            else:
                ff_torque_nm = (
                    Kff_backward
                    * tension_ref
                    * float(self.cfg.ff_radius_m)
                )                    
        else:
            ff_torque_nm = 0.0

        feedback_torque_nm = p_term + d_term + i_term
        torque_nm_unsat = ff_torque_nm + feedback_torque_nm

        # Apply sign convention before deadband/saturation.
        torque_cmd = torque_nm_unsat * float(self.cfg.speed_sign)

        # Deadband + saturation
        if abs(torque_cmd) < float(self.cfg.torque_deadband):
            torque_cmd = 0.0

        lim = abs(float(self.cfg.torque_limit))
        torque_cmd_sat = _clip(torque_cmd, -lim, lim)

        # If saturating, reject integral update (prevents windup).
        if abs(torque_cmd_sat - torque_cmd) <= 1e-9:
            self._int_error = int_candidate
        torque_cmd = torque_cmd_sat
        #return speed_rpm
        return torque_cmd,feedback_torque_nm, ff_torque_nm
