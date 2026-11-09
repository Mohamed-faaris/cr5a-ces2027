import math
import pytest

from cr5a_ros_bridge.pick_place import (
    SHELF_PRESETS, build_task_waypoints, estimate_payload_mass,
    generate_grasp_candidates, generate_shelf_scene, normalize_object,
)


@pytest.mark.parametrize("profile", list(SHELF_PRESETS))
def test_all_shelf_profiles_generate_collision_panels(profile):
    scene = generate_shelf_scene(profile, level=0, object_dimensions_m=[0.066, 0.066, 0.122])
    assert len(scene["collision_objects"]) == SHELF_PRESETS[profile]["levels"] + 4
    assert scene["fit"]["valid"] is True
    assert scene["shelf"]["bay_dimensions_m"][0] > 0


def test_coke_and_pringles_payload_estimates_are_traceable():
    coke = normalize_object({"kind": "coca_cola_330ml"})
    pringles = normalize_object({"kind": "pringles_165g"})
    assert coke["mass_kg"] == pytest.approx(0.355)
    assert pringles["mass_kg"] == pytest.approx(0.205)
    assert "estimate" in coke["source"].lower()


def test_wood_box_mass_uses_volume_times_density():
    result = estimate_payload_mass("wood_box", [0.12, 0.09, 0.08], 650)
    assert result["mass_kg"] == pytest.approx(0.5616)
    assert "650" in result["source"]


def test_vision_object_schema_normalizes_centroid_and_mass():
    obj = normalize_object({"id": "detected_1", "class": "box", "shape": "box",
        "centroid_m": [0.2, 0.1, 0.3], "dimensions_m": [0.1, 0.08, 0.06], "mass_kg": 0.7})
    assert obj["centroid_m"] == [0.2, 0.1, 0.3]
    assert obj["mass_kg"] == 0.7
    assert obj["estimated"] is False


def test_oversized_object_fails_shelf_fit_with_clearance():
    scene = generate_shelf_scene("compact", object_dimensions_m=[0.8, 0.2, 0.2])
    assert scene["fit"]["valid"] is False
    assert scene["fit"]["reasons"]


def test_grasp_centroid_yaw_approach_and_pregrasp_are_distinct():
    obj = normalize_object({"kind": "coca_cola_330ml", "centroid_m": [0.3, 0.1, 0.2]})
    grasp = generate_grasp_candidates(obj, gripper_yaw_deg=90, approach_angle_deg=15)[0]
    assert grasp["tcp_target_m"] == [0.3, 0.1, 0.2]
    assert grasp["gripper_yaw_deg"] == 90
    assert grasp["approach_angle_deg"] == 15
    assert grasp["pregrasp_target_m"] != grasp["tcp_target_m"]


def test_task_waypoints_include_pick_lift_place_and_retreat():
    obj = normalize_object({"kind": "coca_cola_330ml", "centroid_m": [0.3, 0.0, 0.1]})
    grasp = generate_grasp_candidates(obj, gripper_yaw_deg=0)[0]
    waypoints = build_task_waypoints(obj, grasp, [0.45, 0.0, 0.02])
    assert [item["name"] for item in waypoints] == ["pregrasp", "grasp", "lift", "preplace", "place", "retreat"]
    assert waypoints[2]["object_attached"] is True


def test_rejects_non_finite_dimensions():
    with pytest.raises(ValueError):
        normalize_object({"dimensions_m": [0.1, math.nan, 0.1]})
