"""Geometry and input validation for collision-checked Dobot direct moves.

All planning states use MoveIt radians/metres. Dobot MovJ/MovL service
arguments remain controller units: joint degrees or pose mm/degrees.
"""
import math


JOINT_NAMES = tuple(f"joint{i}" for i in range(1, 7))


def finite_values(values, expected_length, label):
    if not isinstance(values, (list, tuple)) or len(values) != expected_length:
        raise ValueError(f"{label} must contain exactly {expected_length} numbers")
    result = []
    for index, value in enumerate(values):
        if isinstance(value, bool):
            raise ValueError(f"{label}[{index}] must be a finite number")
        try:
            number = float(value)
        except (TypeError, ValueError, OverflowError) as exc:
            raise ValueError(f"{label}[{index}] must be a finite number") from exc
        if not math.isfinite(number):
            raise ValueError(f"{label}[{index}] must be a finite number")
        result.append(number)
    return result


def movj_target(values):
    """Validate a six-axis Dobot joint target in degrees."""
    return finite_values(values, 6, "target_joints_deg")


def movl_target(values):
    """Validate a Dobot pose target [x,y,z,rx,ry,rz] in mm/degrees."""
    return finite_values(values, 6, "target_pose_mm_deg")


def joint_path_samples(start_rad, target_deg, max_step_rad=math.radians(2), max_samples=200):
    start = finite_values(start_rad, 6, "current joint state")
    target = [math.radians(value) for value in movj_target(target_deg)]
    if not math.isfinite(max_step_rad) or max_step_rad <= 0:
        raise ValueError("joint sample step must be greater than zero")
    steps = max(1, math.ceil(max(abs(b - a) for a, b in zip(start, target)) / max_step_rad))
    if steps > max_samples:
        raise ValueError(
            f"Joint path requires {steps} samples; limit is {max_samples}. "
            "Use a closer target or increase the configured sample limit."
        )
    return [
        [a + (b - a) * index / steps for a, b in zip(start, target)]
        for index in range(1, steps + 1)
    ]


def quaternion_from_rpy_degrees(roll, pitch, yaw):
    r, p, y = map(math.radians, (roll, pitch, yaw))
    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def quaternion_slerp(start, target, fraction):
    q0 = finite_values(start, 4, "start quaternion")
    q1 = finite_values(target, 4, "target quaternion")
    dot = sum(a * b for a, b in zip(q0, q1))
    if dot < 0:
        q1 = [-value for value in q1]
        dot = -dot
    dot = max(-1.0, min(1.0, dot))
    if dot > 0.9995:
        value = [a + fraction * (b - a) for a, b in zip(q0, q1)]
        norm = math.sqrt(sum(item * item for item in value))
        if norm <= 1e-12:
            raise ValueError("Cannot interpolate a zero-length quaternion")
        return [item / norm for item in value]
    theta = math.acos(dot)
    sin_theta = math.sin(theta)
    first = math.sin((1 - fraction) * theta) / sin_theta
    second = math.sin(fraction * theta) / sin_theta
    return [first * a + second * b for a, b in zip(q0, q1)]


def quaternion_angular_distance(start, target):
    q0 = finite_values(start, 4, "start quaternion")
    q1 = finite_values(target, 4, "target quaternion")
    norm0 = math.sqrt(sum(item * item for item in q0))
    norm1 = math.sqrt(sum(item * item for item in q1))
    if norm0 <= 1e-12 or norm1 <= 1e-12:
        raise ValueError("Pose contains a zero-length quaternion")
    dot = abs(sum(a * b for a, b in zip(q0, q1)) / (norm0 * norm1))
    return 2 * math.acos(max(-1.0, min(1.0, dot)))


def movl_path_samples(start_position, start_orientation, target_pose_mm_deg,
                      max_linear_step_m=0.01,
                      max_angular_step_rad=math.radians(2),
                      max_samples=200):
    """Interpolate a straight Cartesian pose path in the MoveIt base frame."""
    start = finite_values(start_position, 3, "start position")
    target = movl_target(target_pose_mm_deg)
    end = [value / 1000.0 for value in target[:3]]
    end_orientation = quaternion_from_rpy_degrees(*target[3:])
    if max_linear_step_m <= 0 or max_angular_step_rad <= 0:
        raise ValueError("Cartesian sample steps must be greater than zero")
    linear_distance = math.sqrt(sum((b - a) ** 2 for a, b in zip(start, end)))
    angular_distance = quaternion_angular_distance(start_orientation, end_orientation)
    steps = max(
        1,
        math.ceil(linear_distance / max_linear_step_m),
        math.ceil(angular_distance / max_angular_step_rad),
    )
    if steps > max_samples:
        raise ValueError(
            f"Cartesian path requires {steps} samples; limit is {max_samples}. "
            "Use a closer target or increase the configured sample limit."
        )
    return [
        (
            [a + (b - a) * index / steps for a, b in zip(start, end)],
            quaternion_slerp(start_orientation, end_orientation, index / steps),
        )
        for index in range(1, steps + 1)
    ]
