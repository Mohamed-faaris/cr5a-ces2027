import math

import numpy as np
import pytest

from cr5a_vision.model_registry import resolve_model_checkpoint
from cr5a_vision.vision_geometry import (
    depth_mask_to_camera_points,
    oriented_box_from_base_points,
    parse_hand_eye_yaml,
    transform_points,
)


def test_hand_eye_matrix_is_loaded_with_frame_names_and_translation():
    document = """
calibrated: true
frames:
  parent: Link6
  child: d415_color_optical_frame
transform_parent_to_child:
  matrix:
    - [1, 0, 0, 0.02]
    - [0, 1, 0, 0.01]
    - [0, 0, 1, 0.10]
    - [0, 0, 0, 1]
"""
    value = parse_hand_eye_yaml(document)
    assert value["parent"] == "Link6"
    assert value["child"] == "d415_color_optical_frame"
    assert value["translation_m"] == pytest.approx([0.02, 0.01, 0.1])
    assert value["quaternion_xyzw"] == pytest.approx([0, 0, 0, 1])


def test_hand_eye_rejects_unvalidated_or_bad_rotation():
    with pytest.raises(ValueError, match="calibrated: true"):
        parse_hand_eye_yaml("frames: {parent: Link6, child: camera}\ntransform_parent_to_child: {}")
    bad = """
calibrated: true
frames: {parent: Link6, child: camera}
transform_parent_to_child:
  matrix: [[2,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]
"""
    with pytest.raises(ValueError, match="orthonormal"):
        parse_hand_eye_yaml(bad)


def test_depth_mask_deprojects_16uc1_millimetres_to_camera_metres():
    mask = np.ones((5, 5), dtype=bool)
    depth = np.full((5, 5), 1000, dtype=np.uint16)
    points = depth_mask_to_camera_points(mask, depth, [100,0,2,0,100,2,0,0,1], "16UC1")
    assert points.shape == (25, 3)
    assert np.allclose(points[:, 2], 1.0)
    assert np.mean(points[:, 0]) == pytest.approx(0)
    assert np.mean(points[:, 1]) == pytest.approx(0)


def test_transform_and_gravity_aligned_box_estimation():
    points = np.array([[x, y, z] for x in np.linspace(-.05, .05, 10)
                       for y in np.linspace(-.02, .02, 6)
                       for z in np.linspace(.1, .2, 5)])
    transformed = transform_points(points, [0.3, -0.2, 0.4], [0, 0, math.sin(math.pi/4), math.cos(math.pi/4)])
    box = oriented_box_from_base_points(transformed)
    assert box["position_m"] == pytest.approx([0.3, -0.2, 0.55], abs=.002)
    assert box["dimensions_m"][0] > box["dimensions_m"][1]
    assert box["yaw_deg"] == pytest.approx(90, abs=1)


@pytest.mark.parametrize("size", ["n", "s", "m", "l", "x"])
@pytest.mark.parametrize(("family", "prefix"), [
    ("yolo26-seg", "yolo26"),
    ("yoloe-26-seg", "yoloe-26"),
    ("yolo11-seg", "yolo11"),
    ("yolov8-seg", "yolov8"),
])
def test_official_model_registry_maps_family_and_size(family, prefix, size):
    assert resolve_model_checkpoint(family, size) == f"{prefix}{size}-seg.pt"


def test_model_registry_requires_existing_custom_checkpoint(tmp_path):
    checkpoint = tmp_path / "best.pt"
    checkpoint.write_bytes(b"test")
    assert resolve_model_checkpoint("custom", "n", str(checkpoint)) == str(checkpoint)
    with pytest.raises(ValueError, match="does not exist"):
        resolve_model_checkpoint("custom", "n", str(tmp_path / "missing.pt"))
    with pytest.raises(ValueError, match="model size"):
        resolve_model_checkpoint("yolo26-seg", "xx")
