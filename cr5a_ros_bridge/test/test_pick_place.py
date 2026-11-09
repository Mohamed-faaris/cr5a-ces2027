import math
import pytest

from cr5a_ros_bridge.pick_place import (
    SHELF_PRESETS, build_task_waypoints, calculate_payload_model, estimate_payload_mass,
    evaluate_gripper_fit, generate_grasp_candidates, generate_shelf_scene, normalize_object,
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
    assert pringles["mass_kg"] == pytest.approx(0.215)
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
    assert [item["name"] for item in waypoints] == ["pregrasp", "grasp", "withdraw", "lift", "preplace", "place", "retreat"]
    assert waypoints[2]["object_attached"] is True
    assert waypoints[6]["object_attached"] is False


def test_rejects_non_finite_dimensions():
    with pytest.raises(ValueError):
        normalize_object({"dimensions_m": [0.1, math.nan, 0.1]})


def test_payload_model_combines_tool_and_object_mass_and_scales_motion():
    light = calculate_payload_model(0.20, tool_mass_kg=0.50, object_com_m=[0.02, 0.0, 0.0])
    heavy = calculate_payload_model(1.00, tool_mass_kg=0.50, object_com_m=[0.02, 0.0, 0.0])
    assert light["total_mass_kg"] == pytest.approx(0.70)
    assert light["center_of_mass_m"][0] == pytest.approx(0.0057142857)
    assert light["static_weight_n"] == pytest.approx(0.70 * 9.80665)
    assert heavy["max_velocity_scaling"] < light["max_velocity_scaling"]
    assert heavy["max_acceleration_scaling"] < light["max_acceleration_scaling"]


def test_payload_model_rejects_non_finite_or_out_of_range_scaling():
    with pytest.raises(ValueError):
        calculate_payload_model(0.2, base_velocity_scaling=1.2)


def test_grasp_tcp_offset_is_rotated_in_gripper_frame():
    obj = normalize_object({"kind": "coca_cola_330ml", "centroid_m": [0.3, 0.1, 0.2]})
    grasp = generate_grasp_candidates(obj, gripper_yaw_deg=90, approach_angle_deg=30,
                                      tcp_offset_m=[0.0, 0.0, 0.02])[0]
    assert grasp["tcp_target_m"][0] == pytest.approx(0.3)
    assert grasp["tcp_target_m"][1] == pytest.approx(0.11)
    assert grasp["tcp_target_m"][2] == pytest.approx(0.217320508)


def test_relative_pose_and_composition_round_trip():
    from cr5a_ros_bridge.pick_place import pose_compose, pose_relative_to
    parent_pos = [0.2, -0.1, 0.4]
    parent_q = [0.0, 0.0, math.sin(math.pi/4), math.cos(math.pi/4)]
    child_pos = [0.3, 0.0, 0.5]
    child_q = [0.0, 0.0, 0.0, 1.0]
    relative = pose_relative_to(parent_pos, parent_q, child_pos, child_q)
    result = pose_compose(parent_pos, parent_q, relative["position_m"], relative["orientation_xyzw"])
    assert result["position_m"] == pytest.approx(child_pos)
    assert result["orientation_xyzw"] == pytest.approx(child_q)


def test_all_shelf_profiles_fit_coke_pringles_and_wood_box():
    objects = [[0.066, 0.066, 0.122], [0.075, 0.075, 0.240], [0.12, 0.09, 0.08]]
    for profile in SHELF_PRESETS:
        for dimensions in objects:
            for level in range(SHELF_PRESETS[profile]["levels"]):
                scene = generate_shelf_scene(profile, level=level, object_dimensions_m=dimensions)
                assert scene["fit"]["valid"] is True, (profile, level, dimensions, scene["fit"])


def test_custom_pgea_finger_wrap_envelope_accepts_coke_and_pringles():
    coke = evaluate_gripper_fit([0.066, 0.066, 0.122], strategy="pgea_finger_wrap")
    pringles = evaluate_gripper_fit([0.075, 0.075, 0.240], strategy="pgea_finger_wrap")
    assert coke["valid"] is True
    assert pringles["valid"] is True
    assert coke["nominal_total_stroke_m"] == pytest.approx(0.040)
    assert pringles["estimated_wrap_envelope_m"] == pytest.approx(0.085)


def test_parallel_jaw_does_not_inherit_extended_finger_envelope():
    result = evaluate_gripper_fit([0.066, 0.066, 0.122], strategy="parallel_jaw")
    assert result["valid"] is False


def test_finger_wrap_envelope_still_checks_clearance_and_orientation():
    too_wide = evaluate_gripper_fit([0.080, 0.080, 0.120], strategy="pgea_finger_wrap")
    rotated_box = evaluate_gripper_fit([0.070, 0.020, 0.120], strategy="parallel_jaw", gripper_yaw_deg=90)
    assert too_wide["valid"] is False
    assert rotated_box["projected_object_width_m"] == pytest.approx(0.020)


def test_grasp_yaw_is_relative_to_detected_object_orientation():
    obj = normalize_object({"kind": "coca_cola_330ml", "yaw_deg": 35})
    grasp = generate_grasp_candidates(obj, gripper_yaw_deg=20)[0]
    assert grasp["gripper_yaw_relative_deg"] == pytest.approx(20)
    assert grasp["gripper_yaw_deg"] == pytest.approx(55)


def test_shelf_collision_geometry_and_pick_target_use_world_pose():
    scene = generate_shelf_scene("standard", level=1, object_dimensions_m=[0.066, 0.066, 0.122],
                                 origin_m=[0.55, 0.0, 0.0], yaw_rad=math.pi/2)
    assert scene["pick_slot_center_m"] == pytest.approx([0.55, 0.0, 0.018 + 0.294 + 0.012 + 0.061])
    left = next(item for item in scene["collision_objects"] if item["id"] == "shelf_left")
    assert left["position_m"] == pytest.approx([0.55, -0.291, 0.45])
    assert left["orientation_xyzw"][2] == pytest.approx(math.sin(math.pi/4))


def test_shelf_place_surface_is_the_selected_bay_floor_in_world_frame():
    scene = generate_shelf_scene("standard", level=1, object_dimensions_m=[0.066, 0.066, 0.122],
                                 origin_m=[0.55, 0.0, 0.0], yaw_rad=math.pi / 2,
                                 clearance_m=0.012)
    expected_floor = 0.018 + (0.900 - 0.018 * 4) / 3 + 0.018 + 0.012
    assert scene["shelf_place_surface_center_m"] == pytest.approx([0.55, 0.0, expected_floor])


def test_place_and_retreat_waypoints_follow_the_approach_axis():
    obj = normalize_object({"kind": "pringles_165g", "centroid_m": [0.5, 0.0, 0.2]})
    grasp = generate_grasp_candidates(obj, gripper_yaw_deg=90, approach_distance_m=0.08)[0]
    waypoints = build_task_waypoints(obj, grasp, [0.3, -0.2, 0.05], place_approach_distance_m=0.28)
    preplace = waypoints[4]["position_m"]
    place = waypoints[5]["position_m"]
    retreat = waypoints[6]["position_m"]
    assert preplace == pytest.approx([place[0] - grasp["approach_vector"][0] * 0.28,
                                      place[1] - grasp["approach_vector"][1] * 0.28,
                                      place[2] - grasp["approach_vector"][2] * 0.28])
    assert retreat == pytest.approx(preplace)


def test_shelf_pick_withdraws_before_lifting_to_avoid_upper_shelf_board():
    obj = normalize_object({"kind": "pringles_165g", "centroid_m": [0.55, 0.0, 0.46], "yaw_deg": 90})
    grasp = generate_grasp_candidates(obj, gripper_yaw_deg=0, approach_distance_m=0.27)[0]
    waypoints = build_task_waypoints(obj, grasp, [0.35, -0.35, 0.02],
                                     lift_distance_m=0.10, withdrawal_distance_m=0.27)
    withdraw, lift = waypoints[2]["position_m"], waypoints[3]["position_m"]
    assert withdraw[0] < obj["centroid_m"][0] - 0.25
    assert lift[0] == pytest.approx(withdraw[0])
    assert lift[2] == pytest.approx(withdraw[2] + 0.10)
