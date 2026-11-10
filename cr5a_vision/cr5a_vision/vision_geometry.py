"""Geometry and calibration helpers for depth-backed object boxes."""
from __future__ import annotations

import math
from typing import Any

import numpy as np
import yaml


def normalize_quaternion(value: list[float] | tuple[float, ...]) -> list[float]:
    if len(value) != 4:
        raise ValueError("quaternion_xyzw must contain four values")
    q = [float(item) for item in value]
    if not all(math.isfinite(item) for item in q):
        raise ValueError("quaternion contains non-finite values")
    norm = math.sqrt(sum(item * item for item in q))
    if norm < 1e-9:
        raise ValueError("quaternion norm is zero")
    return [item / norm for item in q]


def quaternion_from_matrix(matrix: np.ndarray) -> list[float]:
    m = np.asarray(matrix, dtype=float)
    trace = float(np.trace(m))
    if trace > 0:
        scale = math.sqrt(trace + 1.0) * 2.0
        q = [(m[2, 1] - m[1, 2]) / scale,
             (m[0, 2] - m[2, 0]) / scale,
             (m[1, 0] - m[0, 1]) / scale, 0.25 * scale]
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        scale = math.sqrt(max(1e-12, 1.0 + m[0, 0] - m[1, 1] - m[2, 2])) * 2.0
        q = [0.25 * scale, (m[0, 1] + m[1, 0]) / scale,
             (m[0, 2] + m[2, 0]) / scale, (m[2, 1] - m[1, 2]) / scale]
    elif m[1, 1] > m[2, 2]:
        scale = math.sqrt(max(1e-12, 1.0 + m[1, 1] - m[0, 0] - m[2, 2])) * 2.0
        q = [(m[0, 1] + m[1, 0]) / scale, 0.25 * scale,
             (m[1, 2] + m[2, 1]) / scale, (m[0, 2] - m[2, 0]) / scale]
    else:
        scale = math.sqrt(max(1e-12, 1.0 + m[2, 2] - m[0, 0] - m[1, 1])) * 2.0
        q = [(m[0, 2] + m[2, 0]) / scale, (m[1, 2] + m[2, 1]) / scale,
             0.25 * scale, (m[1, 0] - m[0, 1]) / scale]
    return normalize_quaternion(q)


def rotation_from_quaternion(quaternion_xyzw: list[float]) -> np.ndarray:
    x, y, z, w = normalize_quaternion(quaternion_xyzw)
    return np.array([
        [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
        [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
        [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)],
    ], dtype=float)


def quaternion_multiply(a: list[float], b: list[float]) -> list[float]:
    ax, ay, az, aw = normalize_quaternion(a)
    bx, by, bz, bw = normalize_quaternion(b)
    return normalize_quaternion([
        aw*bx + ax*bw + ay*bz - az*by,
        aw*by - ax*bz + ay*bw + az*bx,
        aw*bz + ax*by - ay*bx + az*bw,
        aw*bw - ax*bx - ay*by - az*bz,
    ])


def parse_hand_eye_yaml(document: str) -> dict[str, Any]:
    """Read the repository calibration import format and normalize Link_T_Camera."""
    try:
        raw = yaml.safe_load(document) or {}
    except yaml.YAMLError as exc:
        raise ValueError(f"Invalid hand-eye YAML: {exc}") from exc
    if not isinstance(raw, dict) or raw.get("calibrated") is not True:
        raise ValueError("Calibration file must contain calibrated: true")
    frames = raw.get("frames") or {}
    parent = str(frames.get("parent") or "").strip()
    child = str(frames.get("child") or "").strip()
    if not parent or not child or parent == child:
        raise ValueError("Calibration file needs distinct frames.parent and frames.child")
    transform = raw.get("transform_parent_to_child") or {}
    if not isinstance(transform, dict):
        raise ValueError("transform_parent_to_child must be a mapping")
    matrix_value = transform.get("matrix")
    if matrix_value is not None:
        matrix = np.asarray(matrix_value, dtype=float)
        if matrix.shape != (4, 4) or not np.all(np.isfinite(matrix)):
            raise ValueError("transform_parent_to_child.matrix must be a finite 4x4 matrix")
        if not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-5):
            raise ValueError("hand-eye matrix bottom row must be [0, 0, 0, 1]")
        rotation = matrix[:3, :3]
        if not np.allclose(rotation.T @ rotation, np.eye(3), atol=2e-2):
            raise ValueError("hand-eye rotation is not orthonormal")
        if not math.isclose(float(np.linalg.det(rotation)), 1.0, abs_tol=2e-2):
            raise ValueError("hand-eye rotation determinant must be +1")
        translation = matrix[:3, 3].tolist()
        quaternion = quaternion_from_matrix(rotation)
    else:
        translation = transform.get("translation_m")
        quaternion = transform.get("quaternion_xyzw")
        if not isinstance(translation, list) or len(translation) != 3:
            raise ValueError("hand-eye transform requires a 4x4 matrix or translation_m")
        translation = [float(value) for value in translation]
        if not all(math.isfinite(value) for value in translation):
            raise ValueError("hand-eye translation contains non-finite values")
        quaternion = normalize_quaternion(quaternion or [])
    if max(abs(float(value)) for value in translation) > 2.0:
        raise ValueError("hand-eye translation exceeds 2 m; verify source units are metres")
    return {"parent": parent, "child": child,
            "translation_m": translation, "quaternion_xyzw": quaternion}


def depth_mask_to_camera_points(mask: np.ndarray, depth: np.ndarray, intrinsics: list[float],
                                encoding: str, min_depth_m: float = 0.12,
                                max_depth_m: float = 2.5, max_points: int = 5000) -> np.ndarray:
    """Deproject a segmentation mask using aligned depth and CameraInfo K."""
    if len(intrinsics) != 9:
        raise ValueError("CameraInfo K must contain 9 values")
    if mask.shape[:2] != depth.shape[:2]:
        raise ValueError("Segmentation mask and aligned depth image must have equal dimensions")
    k = np.asarray(intrinsics, dtype=float).reshape(3, 3)
    fx, fy, cx, cy = float(k[0, 0]), float(k[1, 1]), float(k[0, 2]), float(k[1, 2])
    if min(fx, fy) <= 0:
        raise ValueError("CameraInfo focal lengths must be positive")
    z = depth.astype(np.float32, copy=False)
    if encoding.upper() in ("16UC1", "MONO16") or depth.dtype == np.uint16:
        z = z * 0.001
    valid = np.asarray(mask, dtype=bool) & np.isfinite(z) & (z >= min_depth_m) & (z <= max_depth_m)
    rows, cols = np.nonzero(valid)
    if rows.size < 20:
        return np.empty((0, 3), dtype=np.float64)
    if rows.size > max_points:
        indices = np.linspace(0, rows.size - 1, max_points, dtype=np.int64)
        rows, cols = rows[indices], cols[indices]
    distances = z[rows, cols].astype(np.float64)
    x = (cols.astype(np.float64) - cx) * distances / fx
    y = (rows.astype(np.float64) - cy) * distances / fy
    return np.column_stack((x, y, distances))


def transform_points(points: np.ndarray, translation: list[float], quaternion: list[float]) -> np.ndarray:
    return np.asarray(points, dtype=float) @ rotation_from_quaternion(quaternion).T + np.asarray(translation, dtype=float)


def oriented_box_from_base_points(points: np.ndarray) -> dict[str, Any]:
    """Fit a gravity-aligned OBB in base frame; base Z is treated as vertical."""
    points = np.asarray(points, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3 or points.shape[0] < 20:
        raise ValueError("At least 20 valid 3D depth points are required for a box")
    points = points[np.all(np.isfinite(points), axis=1)]
    if points.shape[0] < 20:
        raise ValueError("Too few finite 3D points remain after filtering")
    xy = points[:, :2]
    centered = xy - np.mean(xy, axis=0)
    covariance = centered.T @ centered / max(1, centered.shape[0] - 1)
    eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    axis = eigenvectors[:, int(np.argmax(eigenvalues))]
    yaw = math.atan2(float(axis[1]), float(axis[0]))
    c, s = math.cos(yaw), math.sin(yaw)
    local = np.column_stack((c * centered[:, 0] + s * centered[:, 1],
                             -s * centered[:, 0] + c * centered[:, 1]))
    min_xy, max_xy = np.percentile(local, [1, 99], axis=0)
    min_z, max_z = np.percentile(points[:, 2], [1, 99])
    extents = np.maximum([0.008, 0.008, 0.008], [*(max_xy - min_xy), max_z - min_z])
    center_local = (min_xy + max_xy) / 2
    center_xy = np.array([c * center_local[0] - s * center_local[1],
                          s * center_local[0] + c * center_local[1]]) + np.mean(xy, axis=0)
    center = [float(center_xy[0]), float(center_xy[1]), float((min_z + max_z) / 2)]
    return {"position_m": center, "dimensions_m": [float(value) for value in extents],
            "yaw_rad": yaw, "yaw_deg": math.degrees(yaw),
            "orientation_xyzw": [0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)]}
