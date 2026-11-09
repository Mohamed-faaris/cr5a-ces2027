#!/usr/bin/env python3
"""Convert a legacy CR5A wrist-eye dataset into the Calibration page import bundle.

This is an offline, one-time adapter. It does not launch ROS, plan motion, or
modify the source dataset. Camera poses are computed as base_T_tcp * tcp_T_camera
when a validated-format hand_eye_transform.yaml is present; otherwise TCP poses
are used and explicitly labelled as such.
"""
from __future__ import annotations

import argparse
import base64
import json
import math
from pathlib import Path
from typing import Any


def read_json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{path} must contain a JSON object")
    return value


def pose_matrix(pose: dict[str, Any], label: str) -> list[list[float]]:
    matrix = pose.get("matrix")
    if matrix is not None:
        if not isinstance(matrix, list) or len(matrix) != 4 or any(
            not isinstance(row, list) or len(row) != 4 for row in matrix
        ):
            raise ValueError(f"{label}.matrix must be 4x4")
        result = [[float(v) for v in row] for row in matrix]
    else:
        t = pose.get("translation_m")
        q = pose.get("quaternion_xyzw")
        if not isinstance(t, list) or len(t) != 3 or not isinstance(q, list) or len(q) != 4:
            raise ValueError(f"{label} needs matrix or translation_m + quaternion_xyzw")
        x, y, z, w = (float(v) for v in q)
        norm = math.sqrt(x*x + y*y + z*z + w*w)
        if norm < 1e-12:
            raise ValueError(f"{label} quaternion has zero length")
        x, y, z, w = x/norm, y/norm, z/norm, w/norm
        result = [
            [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w), float(t[0])],
            [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w), float(t[1])],
            [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y), float(t[2])],
            [0.0, 0.0, 0.0, 1.0],
        ]
    if not all(math.isfinite(v) for row in result for v in row):
        raise ValueError(f"{label} contains non-finite values")
    return result


def matmul(a: list[list[float]], b: list[list[float]]) -> list[list[float]]:
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def quaternion_from_matrix(m: list[list[float]]) -> list[float]:
    trace = m[0][0] + m[1][1] + m[2][2]
    if trace > 0:
        s = math.sqrt(trace + 1.0) * 2
        q = [(m[2][1]-m[1][2])/s, (m[0][2]-m[2][0])/s,
             (m[1][0]-m[0][1])/s, 0.25*s]
    elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2
        q = [0.25*s, (m[0][1]+m[1][0])/s, (m[0][2]+m[2][0])/s, (m[2][1]-m[1][2])/s]
    elif m[1][1] > m[2][2]:
        s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2
        q = [(m[0][1]+m[1][0])/s, 0.25*s, (m[1][2]+m[2][1])/s, (m[0][2]-m[2][0])/s]
    else:
        s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2
        q = [(m[0][2]+m[2][0])/s, (m[1][2]+m[2][1])/s, 0.25*s, (m[1][0]-m[0][1])/s]
    norm = math.sqrt(sum(v*v for v in q))
    return [v/norm for v in q]


def load_tcp_to_camera(root: Path) -> tuple[list[list[float]] | None, dict[str, str]]:
    path = root / "results" / "hand_eye_transform.yaml"
    if not path.is_file():
        return None, {}
    try:
        import yaml
        result = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
        frames = result.get("frames") or {}
        transform = result.get("transform_parent_to_child") or {}
        matrix = transform.get("matrix")
        if not result.get("calibrated") or not matrix:
            return None, {}
        return pose_matrix(transform, "transform_parent_to_child"), {
            "parent": str(frames.get("parent", "Link6")),
            "child": str(frames.get("child", "d415_color_optical_frame")),
        }
    except Exception as exc:
        raise ValueError(f"Cannot read {path}: {exc}") from exc


def load_joint_plan(root: Path) -> dict[int, list[float]]:
    path = root / "poses" / "calibration_poses.jsonl"
    result: dict[int, list[float]] = {}
    if not path.is_file():
        return result
    for line_number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        if not line.strip():
            continue
        row = json.loads(line)
        if row.get("type") == "joint" and isinstance(row.get("positions"), list):
            result[int(row.get("id", line_number - 1))] = [float(v) for v in row["positions"]]
    return result


def load_planned_poses(path: Path | None, warnings: list[str]) -> list[dict[str, Any]]:
    if path is None or not path.is_file():
        return []
    planned = []
    skipped_joint_only = 0
    joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
    for line_number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        if not line.strip():
            continue
        row = json.loads(line)
        pose = row.get("pose")
        if not isinstance(pose, dict):
            skipped_joint_only += 1
            continue
        matrix = pose_matrix(pose, f"{path.name}:{line_number}.pose")
        positions = row.get("positions")
        joints = dict(zip(joint_names, [float(v) for v in positions])) if isinstance(positions, list) and len(positions) == 6 else None
        pose_id = int(row.get("id", line_number - 1))
        planned.append({
            "id": f"plan-{pose_id:03d}",
            "label": f"Planned pose {pose_id:03d}",
            "kind": "planned",
            "position_m": [float(matrix[i][3]) for i in range(3)],
            "orientation_xyzw": quaternion_from_matrix(matrix),
            "pose_frame": "tcp",
            "joints": joints,
            "tilt_deg": row.get("tilt_deg"),
            "azimuth_deg": row.get("azimuth_deg"),
            "roll_deg": row.get("roll_deg"),
            "distance_m": row.get("distance_m"),
        })
    if skipped_joint_only:
        warnings.append(
            f"{skipped_joint_only} joint-only plan entries were not plotted; they have no base-frame TCP pose and need FK/MoveIt to visualize."
        )
    return planned


def average_board_pose(transforms: list[list[list[float]]]) -> dict[str, Any] | None:
    """Aggregate fixed-board poses estimated in the same base frame."""
    if not transforms:
        return None
    positions = [[float(matrix[row][3]) for row in range(3)] for matrix in transforms]
    position = [
        sorted(axis)[len(axis) // 2] if len(axis) % 2 else
        (sorted(axis)[len(axis) // 2 - 1] + sorted(axis)[len(axis) // 2]) / 2
        for axis in zip(*positions)
    ]
    quaternions = [quaternion_from_matrix(matrix) for matrix in transforms]
    reference = quaternions[0]
    aligned = [
        [-v for v in quaternion] if sum(a * b for a, b in zip(reference, quaternion)) < 0 else quaternion
        for quaternion in quaternions
    ]
    averaged = [sum(q[i] for q in aligned) for i in range(4)]
    norm = math.sqrt(sum(v * v for v in averaged))
    if norm < 1e-12:
        raise ValueError("Could not average board orientation")
    orientation = [value / norm for value in averaged]
    deviations_mm = sorted(
        math.sqrt(sum((p[i] - position[i]) ** 2 for i in range(3))) * 1000
        for p in positions
    )
    median_deviation = (deviations_mm[len(deviations_mm) // 2 - 1] + deviations_mm[len(deviations_mm) // 2]) / 2 if len(deviations_mm) % 2 == 0 else deviations_mm[len(deviations_mm) // 2]
    return {
        "position_m": position,
        "orientation_xyzw": orientation,
        "pose_sample_count": len(transforms),
        "median_position_deviation_mm": median_deviation,
        "max_position_deviation_mm": max(deviations_mm),
        "aggregation": "coordinate-wise median position and sign-aligned quaternion mean",
    }


def image_thumbnail(path: Path, max_width: int, quality: int) -> str:
    import cv2
    import numpy as np
    image = cv2.imread(str(path), cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError(f"Could not decode image: {path}")
    height, width = image.shape[:2]
    if width > max_width:
        scale = max_width / width
        image = cv2.resize(image, (max_width, max(1, round(height * scale))), interpolation=cv2.INTER_AREA)
    ok, encoded = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, quality])
    if not ok:
        raise ValueError(f"Could not encode thumbnail: {path}")
    payload = base64.b64encode(encoded.tobytes()).decode("ascii")
    return "data:image/jpeg;base64," + payload


def convert_dataset(root: Path, include_thumbnails: bool = True,
                    max_width: int = 480, quality: int = 72,
                    pose_plan_path: Path | None = None) -> dict[str, Any]:
    root = root.expanduser().resolve()
    samples_dir = root / "samples"
    if not samples_dir.is_dir():
        raise ValueError(f"Missing samples directory: {samples_dir}")
    manifest_path = root / "manifest.json"
    manifest = read_json(manifest_path) if manifest_path.is_file() else {}
    tcp_to_camera, camera_frames = load_tcp_to_camera(root)
    joint_plan = load_joint_plan(root)
    sample_files = sorted(samples_dir.glob("sample_*.json"))
    if not sample_files:
        raise ValueError(f"No sample_*.json files found in {samples_dir}")
    first_sample = read_json(sample_files[0])
    board_detection = first_sample.get("board_detection") or {}

    poses = []
    board_transforms_base = []
    warnings = []
    joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
    for sample_path in sample_files:
        sample = read_json(sample_path)
        pose_id = int(sample.get("pose_id", int(sample_path.stem.split("_")[-1])))
        base_tcp = pose_matrix(sample.get("base_to_tcp") or {}, f"{sample_path.name}.base_to_tcp")
        pose_frame = "tcp"
        display_matrix = base_tcp
        if tcp_to_camera is not None:
            frames = sample.get("frames") or {}
            sample_tcp_frame = str(frames.get("tcp", camera_frames.get("parent", "Link6")))
            if sample_tcp_frame == camera_frames.get("parent"):
                display_matrix = matmul(base_tcp, tcp_to_camera)
                pose_frame = "camera"
            else:
                warnings.append(f"{sample_path.name}: TCP frame {sample_tcp_frame!r} does not match hand-eye parent {camera_frames.get('parent')!r}; showing TCP pose")
        positions = joint_plan.get(pose_id)
        joints = dict(zip(joint_names, positions)) if positions and len(positions) == 6 else None
        detection = sample.get("board_detection") or {}
        camera_board = detection.get("camera_T_board")
        if pose_frame == "camera" and isinstance(camera_board, dict):
            rms = detection.get("reprojection_rms_px")
            if isinstance(rms, (int, float)) and float(rms) > 5.0:
                warnings.append(f"{sample_path.name}: board pose excluded from scene estimate (reprojection RMS {float(rms):.2f} px > 5 px)")
            else:
                try:
                    board_transforms_base.append(matmul(display_matrix, pose_matrix(camera_board, f"{sample_path.name}.camera_T_board")))
                except (TypeError, ValueError) as exc:
                    warnings.append(f"{sample_path.name}: board pose excluded from scene estimate ({exc})")
        image_rel = str(sample.get("image") or "")
        image_path = (root / image_rel).resolve() if image_rel else None
        try:
            if image_path and root not in image_path.parents:
                image_path = None
        except Exception:
            image_path = None
        image_url = None
        if include_thumbnails and image_path and image_path.is_file():
            try:
                image_url = image_thumbnail(image_path, max_width, quality)
            except Exception as exc:
                warnings.append(f"{sample_path.name}: thumbnail skipped ({exc})")
        if not image_path or not image_path.is_file():
            warnings.append(f"{sample_path.name}: image file is missing ({image_rel or 'no image path'})")

        timestamp = sample.get("timestamp") or {}
        poses.append({
            "id": f"sample-{pose_id:03d}",
            "label": f"Sample {pose_id:03d}",
            "position_m": [float(display_matrix[i][3]) for i in range(3)],
            "orientation_xyzw": quaternion_from_matrix(display_matrix),
            "pose_frame": pose_frame,
            "tcp_position_m": [float(base_tcp[i][3]) for i in range(3)],
            "tcp_orientation_xyzw": quaternion_from_matrix(base_tcp),
            "joints": joints,
            "image_path": image_rel or None,
            "image_data_url": image_url,
            "reprojection_rms_px": detection.get("reprojection_rms_px"),
            "tag_ids": detection.get("tag_ids") or [],
            "timestamp": timestamp,
        })

    plan_path = pose_plan_path.expanduser().resolve() if pose_plan_path else root / "poses" / "calibration_poses.jsonl"
    planned_poses = load_planned_poses(plan_path, warnings)
    board_pose = average_board_pose(board_transforms_base)
    board = {
        "family": board_detection.get("tag_family"),
        "type": board_detection.get("board_type"),
        "dictionary": "DICT_5X5_1000" if board_detection.get("tag_family") == "aruco_dict_5x5" else None,
        "frame_id": str((first_sample.get("frames") or {}).get("base", manifest.get("base_frame", "base_link"))),
        "position_m": board_pose.get("position_m") if board_pose else None,
        "orientation_xyzw": board_pose.get("orientation_xyzw") if board_pose else None,
        "pose_sample_count": board_pose.get("pose_sample_count", 0) if board_pose else 0,
        "pose_aggregation": board_pose.get("aggregation") if board_pose else None,
        "median_position_deviation_mm": board_pose.get("median_position_deviation_mm") if board_pose else None,
        "max_position_deviation_mm": board_pose.get("max_position_deviation_mm") if board_pose else None,
        "geometry": {
            "columns": 24,
            "rows": 17,
            "square_size_m": 0.030,
            "marker_size_m": 0.022,
            "margin_m": 0.010,
            "pattern_width_m": 0.720,
            "pattern_height_m": 0.510,
            "width_m": 0.740,
            "height_m": 0.530,
            "center_offset_m": [0.360, 0.255],
            "texture_url": "/calibration/charuco-board-24x17.png",
        },
    }
    if board_pose:
        warnings.append(
            "Calibration board is shown at the median pose reconstructed from captured board detections and the stored hand-eye transform; this is a visual reference, not a ground-truth survey or collision object."
        )
    else:
        warnings.append(
            "Calibration board pose is unavailable in the base frame. A validated hand-eye transform and per-sample camera_T_board detections are required before the board can be placed in the 3D scene."
        )
    if planned_poses:
        warnings.append("Generated target poses are preview-only; this import does not run MoveIt IK or collision checks.")
    report_path = root / "results" / "report.json"
    comparison_path = root / "results" / "solver_comparison.json"
    report = read_json(report_path) if report_path.is_file() else {}
    comparison = read_json(comparison_path) if comparison_path.is_file() else {}
    algorithms = comparison.get("algorithms", [])
    valid_algorithms = [row for row in algorithms if isinstance(row, dict)]
    best_translation = min(
        (row for row in valid_algorithms if isinstance(row.get("translation_rms_mm"), (int, float))),
        key=lambda row: float(row["translation_rms_mm"]), default={}
    )
    best_rotation = min(
        (row for row in valid_algorithms if isinstance(row.get("rotation_rms_deg"), (int, float))),
        key=lambda row: float(row["rotation_rms_deg"]), default={}
    )
    return {
        "schema_version": 1,
        "source_format": "cr5a_wrist_eye_calibration",
        "source_dataset": root.name,
        "frames": {
            "base": str((first_sample.get("frames") or {}).get("base", manifest.get("base_frame", "base_link"))),
            "tcp": str((first_sample.get("frames") or {}).get("tcp", manifest.get("tcp_frame", "Link6"))),
            "camera": str((first_sample.get("frames") or {}).get("camera", camera_frames.get("child", "d415_color_optical_frame"))),
        },
        "board": board,
        "summary": {
            "sample_count": len(poses),
            "planned_pose_count": len(planned_poses),
            "calibrated": bool(report.get("calibrated", False)),
            "algorithm": report.get("algorithm"),
            "mean_board_reprojection_rms_px": report.get("mean_board_reprojection_rms_px"),
            "translation_rms_mm": report.get("translation_rms_mm") if report.get("translation_rms_mm") is not None else best_translation.get("translation_rms_mm"),
            "rotation_rms_deg": report.get("rotation_rms_deg") if report.get("rotation_rms_deg") is not None else best_rotation.get("rotation_rms_deg"),
            "best_translation_rms_algorithm": comparison.get("best_translation_rms_algorithm") or best_translation.get("algorithm"),
            "best_rotation_rms_algorithm": comparison.get("best_rotation_rms_algorithm") or best_rotation.get("algorithm"),
            "algorithms": algorithms,
        },
        "pose_reference": "camera optical frame origin in base frame" if tcp_to_camera is not None else "TCP origin in base frame (no calibrated camera extrinsic found)",
        "poses": poses,
        "planned_poses": planned_poses,
        "warnings": warnings,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset-dir", required=True, type=Path, help="Legacy cr5a_wrist_eye_calibration/data/<run> directory")
    parser.add_argument("--output", required=True, type=Path, help="Output JSON bundle to import in the Calibration page")
    parser.add_argument("--pose-plan", type=Path, help="Optional generated JSONL pose plan to overlay with the captured samples")
    parser.add_argument("--no-thumbnails", action="store_true", help="Do not embed compressed image previews")
    parser.add_argument("--thumbnail-width", type=int, default=480)
    parser.add_argument("--jpeg-quality", type=int, default=72)
    args = parser.parse_args()
    bundle = convert_dataset(args.dataset_dir, not args.no_thumbnails,
                             args.thumbnail_width, args.jpeg_quality, args.pose_plan)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(bundle, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({
        "output": str(args.output.resolve()),
        "source_dataset": bundle["source_dataset"],
        "samples": len(bundle["poses"]),
        "planned_poses": len(bundle["planned_poses"]),
        "samples_with_thumbnails": sum(bool(p["image_data_url"]) for p in bundle["poses"]),
        "pose_reference": bundle["pose_reference"],
        "summary": bundle["summary"],
        "warnings": bundle["warnings"],
    }, indent=2))


if __name__ == "__main__":
    main()
