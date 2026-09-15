"""Lightweight NumPy kinematics for the tethered hexapod.

The constants in this module are transcribed from
``hexapod_tethered_04_mjx_fullcollisions_slope.xml``.  The implementation
matches MuJoCo's MJCF hinge transform order, but deliberately has no MuJoCo,
Torch, or XML runtime dependency.  All calculations use float64.

Coordinates are expressed in the trunk/IMU FLU frame: x forward, y left,
z up.  The XML trunk ``pos`` and initial ``euler`` are not applied here: the
trunk owns a free joint in simulation, while the real robot attitude comes
from its IMU.  Every quantity needed by the literature controllers is
translation invariant, so the trunk origin can safely be fixed at zero.
"""

from dataclasses import dataclass

import numpy as np


_FLOAT = np.float64
_ZERO3 = np.zeros(3, dtype=_FLOAT)
_IDENTITY3 = np.eye(3, dtype=_FLOAT)
_IDENTITY_QUAT_WXYZ = np.array([1.0, 0.0, 0.0, 0.0], dtype=_FLOAT)


@dataclass(frozen=True)
class HexapodGeometry:
    """Dynamic geometry used by Nagatani and Polzin--Hughes controllers."""

    feet_body: np.ndarray
    com_body: np.ndarray
    tether_start_body: np.ndarray
    foot_radii_m: np.ndarray
    geometry_valid: bool


def _unit(vector: np.ndarray, *, name: str) -> np.ndarray:
    vector = np.asarray(vector, dtype=_FLOAT)
    norm = float(np.linalg.norm(vector))
    if not np.isfinite(norm) or norm <= 1.0e-12:
        raise ValueError(f"{name} must have a finite, nonzero norm")
    return vector / norm


def _quat_wxyz_to_matrix(quaternion: np.ndarray) -> np.ndarray:
    """Convert an MJCF scalar-first quaternion to a rotation matrix."""

    w, x, y, z = _unit(quaternion, name="body quaternion")
    return np.array(
        [
            [
                1.0 - 2.0 * (y * y + z * z),
                2.0 * (x * y - z * w),
                2.0 * (x * z + y * w),
            ],
            [
                2.0 * (x * y + z * w),
                1.0 - 2.0 * (x * x + z * z),
                2.0 * (y * z - x * w),
            ],
            [
                2.0 * (x * z - y * w),
                2.0 * (y * z + x * w),
                1.0 - 2.0 * (x * x + y * y),
            ],
        ],
        dtype=_FLOAT,
    )


def _axis_angle_matrix(axis: np.ndarray, angle: float) -> np.ndarray:
    """Return the active right-handed rotation about a local MJCF axis."""

    x, y, z = _unit(axis, name="joint axis")
    sine = float(np.sin(angle))
    cosine = float(np.cos(angle))
    one_minus_cosine = 1.0 - cosine
    return np.array(
        [
            [
                cosine + x * x * one_minus_cosine,
                x * y * one_minus_cosine - z * sine,
                x * z * one_minus_cosine + y * sine,
            ],
            [
                y * x * one_minus_cosine + z * sine,
                cosine + y * y * one_minus_cosine,
                y * z * one_minus_cosine - x * sine,
            ],
            [
                z * x * one_minus_cosine - y * sine,
                z * y * one_minus_cosine + x * sine,
                cosine + z * z * one_minus_cosine,
            ],
        ],
        dtype=_FLOAT,
    )


def _rotation_x(angle: float) -> np.ndarray:
    sine, cosine = float(np.sin(angle)), float(np.cos(angle))
    return np.array(
        [[1.0, 0.0, 0.0], [0.0, cosine, -sine], [0.0, sine, cosine]],
        dtype=_FLOAT,
    )


def _rotation_y(angle: float) -> np.ndarray:
    sine, cosine = float(np.sin(angle)), float(np.cos(angle))
    return np.array(
        [[cosine, 0.0, sine], [0.0, 1.0, 0.0], [-sine, 0.0, cosine]],
        dtype=_FLOAT,
    )


def _rotation_z(angle: float) -> np.ndarray:
    sine, cosine = float(np.sin(angle)), float(np.cos(angle))
    return np.array(
        [[cosine, -sine, 0.0], [sine, cosine, 0.0], [0.0, 0.0, 1.0]],
        dtype=_FLOAT,
    )


def slope_basis_body(
    gravity_body,
    slope_rad,
    body_yaw_rad=0.0,
    gravity_valid=True,
):
    """Return terrain ``(normal, uphill, cross_slope)`` in body axes.

    ``gravity_body`` is the unit downward direction returned by the real IMU
    in FLU axes.  Gravity determines roll and pitch; ``body_yaw_rad`` supplies
    the otherwise unobservable heading.  For the current straight-uphill
    experiment it is fixed to zero.

    If the gravity sample is invalid, the returned nominal basis assumes the
    body is aligned with the requested slope.  At zero yaw this is exactly
    ``normal=[0, 0, 1]``, ``uphill=[1, 0, 0]``, and
    ``cross_slope=[0, 1, 0]``.
    """

    slope = float(slope_rad)
    yaw = float(body_yaw_rad)
    if not np.isfinite(slope):
        raise ValueError("slope_rad must be finite")
    if not np.isfinite(yaw):
        raise ValueError("body_yaw_rad must be finite")

    sample_valid = bool(gravity_valid)
    if sample_valid:
        try:
            gravity = np.asarray(gravity_body, dtype=_FLOAT)
        except (TypeError, ValueError):
            gravity = np.zeros(3, dtype=_FLOAT)
            sample_valid = False
    else:
        gravity = np.zeros(3, dtype=_FLOAT)
    sample_valid = sample_valid and gravity.shape == (3,)
    sample_valid = sample_valid and bool(np.all(np.isfinite(gravity)))
    gravity_norm = float(np.linalg.norm(gravity)) if sample_valid else 0.0
    sample_valid = sample_valid and gravity_norm > 1.0e-8

    if sample_valid:
        gravity = gravity / gravity_norm
        # For R_wb = Rz(yaw) Ry(pitch) Rx(roll), the downward direction in
        # body coordinates is [sin(pitch), -sin(roll)cos(pitch),
        # -cos(roll)cos(pitch)].
        pitch = float(np.arcsin(np.clip(gravity[0], -1.0, 1.0)))
        roll = float(np.arctan2(-gravity[1], -gravity[2]))
        rotation_world_from_body = (
            _rotation_z(yaw) @ _rotation_y(pitch) @ _rotation_x(roll)
        )
    else:
        # Nominally the body x-axis points uphill and its z-axis is normal to
        # the slope.  R_y(-slope) reproduces that pose at yaw zero.
        rotation_world_from_body = _rotation_z(yaw) @ _rotation_y(-slope)

    sine, cosine = float(np.sin(slope)), float(np.cos(slope))
    uphill_world = np.array([cosine, 0.0, sine], dtype=_FLOAT)
    normal_world = np.array([-sine, 0.0, cosine], dtype=_FLOAT)

    rotation_body_from_world = rotation_world_from_body.T
    normal_body = _unit(
        rotation_body_from_world @ normal_world, name="terrain normal"
    )
    uphill_body = rotation_body_from_world @ uphill_world
    # Re-orthogonalize to suppress roundoff before the support projections.
    uphill_body -= float(np.dot(uphill_body, normal_body)) * normal_body
    uphill_body = _unit(uphill_body, name="uphill direction")
    cross_slope_body = _unit(
        np.cross(normal_body, uphill_body), name="cross-slope direction"
    )
    return normal_body, uphill_body, cross_slope_body


class HexapodMjcfKinematics:
    """MuJoCo-equivalent FK/COM for the 18 leg joints and fixed tether arm."""

    LEG_NAMES = ("RF", "RM", "RB", "LF", "LM", "LB")
    NUM_LEG_JOINTS = 18

    _TRUNK_INERTIAL_POS = np.array([0.0, 0.0, -0.146], dtype=_FLOAT)
    _TRUNK_MASS = 16.972

    _HIP_POS = np.array(
        [
            [0.45, -0.1, -0.1105],
            [0.0, -0.2, -0.1105],
            [-0.449999, -0.100003, -0.1108],
            [0.450001, 0.0999972, -0.1105],
            [0.0, 0.199997, -0.1105],
            [-0.449999, 0.0999972, -0.1105],
        ],
        dtype=_FLOAT,
    )
    _HIP_QUAT = np.array(
        [
            [0.0, 0.707107, 0.707107, 0.0],
            [0.0, 0.707107, 0.707107, 0.0],
            [0.0, 0.707107, 0.707107, 0.0],
            [0.707107, 0.0, 0.0, -0.707107],
            [0.707107, 0.0, 0.0, -0.707107],
            [0.707107, 0.0, 0.0, -0.707107],
        ],
        dtype=_FLOAT,
    )
    _HIP_AXIS = np.array(
        [
            [0.0, 0.0, 1.0],
            [0.0, 0.0, 1.0],
            [0.0, 0.0, 1.0],
            [0.0, 0.0, -1.0],
            [0.0, 0.0, -1.0],
            [0.0, 0.0, -1.0],
        ],
        dtype=_FLOAT,
    )
    _HIP_INERTIAL_POS = np.array(
        [
            [-0.078, 0.031, 0.149],
            [-0.078, -0.0042, 0.149],
            [-0.078, -0.031, 0.149],
            [-0.078, 0.031, -0.149],
            [-0.078, -0.042, -0.149],
            [-0.078, -0.031, -0.149],
        ],
        dtype=_FLOAT,
    )
    _HIP_MASS = np.full(6, 1.411, dtype=_FLOAT)

    _THIGH_POS = np.array(
        [
            [-0.09, 0.0795, 0.154],
            [-0.09, 0.0795, 0.154],
            [-0.09, 0.0945, 0.1537],
            [-0.09, 0.0795, -0.154],
            [-0.09, 0.0795, -0.154],
            [-0.09, 0.0945, -0.154],
        ],
        dtype=_FLOAT,
    )
    _THIGH_QUAT = np.array(
        [
            [0.707107, -0.707107, 0.0, 0.0],
            [0.707107, -0.707107, 0.0, 0.0],
            [0.707107, -0.707107, 0.0, 0.0],
            [0.707107, 0.707107, 0.0, 0.0],
            [0.707107, 0.707107, 0.0, 0.0],
            [0.707107, 0.707107, 0.0, 0.0],
        ],
        dtype=_FLOAT,
    )
    _THIGH_AXIS = np.array(
        [
            [0.0, 0.0, -1.0],
            [0.0, 0.0, -1.0],
            [0.0, 0.0, -1.0],
            [0.0, 0.0, 1.0],
            [0.0, 0.0, 1.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=_FLOAT,
    )
    _THIGH_INERTIAL_POS = np.array(
        [
            [-0.022, 0.01, -0.125],
            [-0.022, 0.01, -0.045],
            [-0.022, 0.01, -0.049],
            [-0.022, 0.01, 0.125],
            [-0.022, 0.01, 0.045],
            [-0.022, 0.01, 0.049],
        ],
        dtype=_FLOAT,
    )
    _THIGH_MASS = np.full(6, 1.286, dtype=_FLOAT)

    _SHANK_POS = np.array(
        [
            [-0.1782, 0.090798, -0.0685],
            [-0.1782, 0.090798, -0.0685],
            [-0.178201, 0.0907981, -0.0835],
            [-0.178201, 0.0907981, 0.0685],
            [-0.178201, 0.0907981, 0.0685],
            [-0.178201, 0.0907981, 0.0835],
        ],
        dtype=_FLOAT,
    )
    _SHANK_AXIS = np.array(
        [
            [0.0, 0.0, -1.0],
            [0.0, 0.0, -1.0],
            [0.0, 0.0, 1.0],
            [0.0, 0.0, 1.0],
            [0.0, 0.0, 1.0],
            [0.0, 0.0, -1.0],
        ],
        dtype=_FLOAT,
    )
    _SHANK_INERTIAL_POS = np.array(
        [
            [-0.0016092, -0.124894, -0.0108801],
            [-0.0016092, -0.124893, -0.0108802],
            [-0.0012739, -0.124897, -0.0111198],
            [-0.0012739, -0.124897, 0.0108802],
            [-0.0012739, -0.124897, 0.0108802],
            [-0.0016092, -0.124893, 0.0111198],
        ],
        dtype=_FLOAT,
    )
    _SHANK_MASS = np.array(
        [0.642909, 0.642908, 0.642908, 0.642909, 0.642909, 0.642909],
        dtype=_FLOAT,
    )

    _FOOT_SITE_POS = np.array(
        [
            [-0.0081826, -0.276, -0.011107],
            [-0.00818264, -0.276, -0.0111074],
            [0.00127539, -0.276, -0.0108926],
            [0.00127539, -0.276, 0.0111074],
            [0.00127539, -0.276, 0.0111074],
            [-0.00818264, -0.276, 0.0108926],
        ],
        dtype=_FLOAT,
    )
    _FOOT_RADII = np.full(6, 0.023, dtype=_FLOAT)

    _ARM_BODY_POS = np.array([0.0, 0.0, 0.018], dtype=_FLOAT)
    _ARM_JOINT_AXIS = np.array([0.0, 0.0, 1.0], dtype=_FLOAT)
    _ARM_INERTIAL_POS = np.array(
        [0.122389, 0.000233508, -0.0229787], dtype=_FLOAT
    )
    _ARM_MASS = 1.18679
    _TETHER_START_SITE_POS = np.array(
        [0.254313929, 0.0, -0.065196152], dtype=_FLOAT
    )

    XML_TOTAL_MASS_KG = 38.198242

    def __init__(self):
        self._last_valid_q = np.zeros(self.NUM_LEG_JOINTS, dtype=_FLOAT)
        self._last_valid_arm_yaw = 0.0

    @staticmethod
    def _hinge_body_pose(
        parent_position: np.ndarray,
        parent_rotation: np.ndarray,
        body_position: np.ndarray,
        body_quaternion: np.ndarray,
        joint_axis: np.ndarray,
        joint_angle: float,
    ) -> tuple[np.ndarray, np.ndarray]:
        """Apply MuJoCo's fixed body transform, then its local hinge."""

        position_zero = parent_position + parent_rotation @ body_position
        rotation_zero = parent_rotation @ _quat_wxyz_to_matrix(body_quaternion)

        # Every hinge in this XML has joint_pos=[0, 0, 0] and qpos0=0.  The
        # generic MuJoCo correction `anchor - R @ joint_pos` therefore leaves
        # position_zero unchanged.
        rotation = rotation_zero @ _axis_angle_matrix(joint_axis, joint_angle)
        return position_zero, rotation

    def forward(self, q_leg_rad, arm_yaw_rad=0.0) -> HexapodGeometry:
        """Compute feet, whole-body COM, and tether start in trunk axes.

        A non-finite joint sample is rejected as a complete frame and replaced
        by the most recent fully finite 18-vector.  The cache starts at the XML
        zero pose.  ``geometry_valid`` is false whenever either the requested
        leg vector or arm yaw was invalid, even though the returned geometry
        remains finite through this causal fallback.
        """

        try:
            q_requested = np.asarray(q_leg_rad, dtype=_FLOAT)
        except (TypeError, ValueError) as error:
            raise ValueError(
                "q_leg_rad must be an array-like of 18 numbers"
            ) from error
        if q_requested.shape != (self.NUM_LEG_JOINTS,):
            raise ValueError(
                f"q_leg_rad must have shape ({self.NUM_LEG_JOINTS},), "
                f"got {q_requested.shape}"
            )

        q_valid = bool(np.all(np.isfinite(q_requested)))
        if q_valid:
            q = q_requested.copy()
            self._last_valid_q = q.copy()
        else:
            q = self._last_valid_q.copy()

        try:
            arm_requested = float(arm_yaw_rad)
        except (TypeError, ValueError) as error:
            raise ValueError("arm_yaw_rad must be a scalar") from error
        arm_valid = bool(np.isfinite(arm_requested))
        if arm_valid:
            arm_yaw = arm_requested
            self._last_valid_arm_yaw = arm_yaw
        else:
            arm_yaw = self._last_valid_arm_yaw

        geometry_valid = q_valid and arm_valid
        feet = np.empty((6, 3), dtype=_FLOAT)

        weighted_position = self._TRUNK_MASS * self._TRUNK_INERTIAL_POS.copy()
        total_mass = float(self._TRUNK_MASS)

        for leg_index in range(6):
            joint_index = 3 * leg_index
            hip_position, hip_rotation = self._hinge_body_pose(
                _ZERO3,
                _IDENTITY3,
                self._HIP_POS[leg_index],
                self._HIP_QUAT[leg_index],
                self._HIP_AXIS[leg_index],
                float(q[joint_index]),
            )
            hip_com = (
                hip_position
                + hip_rotation @ self._HIP_INERTIAL_POS[leg_index]
            )
            weighted_position += self._HIP_MASS[leg_index] * hip_com
            total_mass += float(self._HIP_MASS[leg_index])

            thigh_position, thigh_rotation = self._hinge_body_pose(
                hip_position,
                hip_rotation,
                self._THIGH_POS[leg_index],
                self._THIGH_QUAT[leg_index],
                self._THIGH_AXIS[leg_index],
                float(q[joint_index + 1]),
            )
            thigh_com = (
                thigh_position
                + thigh_rotation @ self._THIGH_INERTIAL_POS[leg_index]
            )
            weighted_position += self._THIGH_MASS[leg_index] * thigh_com
            total_mass += float(self._THIGH_MASS[leg_index])

            shank_position, shank_rotation = self._hinge_body_pose(
                thigh_position,
                thigh_rotation,
                self._SHANK_POS[leg_index],
                _IDENTITY_QUAT_WXYZ,
                self._SHANK_AXIS[leg_index],
                float(q[joint_index + 2]),
            )
            shank_com = (
                shank_position
                + shank_rotation @ self._SHANK_INERTIAL_POS[leg_index]
            )
            weighted_position += self._SHANK_MASS[leg_index] * shank_com
            total_mass += float(self._SHANK_MASS[leg_index])
            feet[leg_index] = (
                shank_position
                + shank_rotation @ self._FOOT_SITE_POS[leg_index]
            )

        arm_position, arm_rotation = self._hinge_body_pose(
            _ZERO3,
            _IDENTITY3,
            self._ARM_BODY_POS,
            _IDENTITY_QUAT_WXYZ,
            self._ARM_JOINT_AXIS,
            arm_yaw,
        )
        arm_com = arm_position + arm_rotation @ self._ARM_INERTIAL_POS
        weighted_position += self._ARM_MASS * arm_com
        total_mass += float(self._ARM_MASS)

        com = weighted_position / total_mass
        tether_start = (
            arm_position + arm_rotation @ self._TETHER_START_SITE_POS
        )

        return HexapodGeometry(
            feet_body=feet,
            com_body=np.asarray(com, dtype=_FLOAT),
            tether_start_body=np.asarray(tether_start, dtype=_FLOAT),
            foot_radii_m=self._FOOT_RADII.copy(),
            geometry_valid=bool(geometry_valid),
        )
