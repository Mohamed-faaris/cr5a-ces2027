import math
import pytest
from cr5a_ros_bridge.platform_geometry import build_platform_box


def marker(marker_id, corner, xyz, yaw=0.0):
    return {"id": str(marker_id), "corner": corner, "position_m": list(xyz),
            "orientation_xyzw": [0.0, 0.0, math.sin(yaw/2), math.cos(yaw/2)]}


def test_four_corner_rectangle_builds_solid_platform_to_ground():
    markers = [marker(0,"front_left",[.4,.2,.8]), marker(1,"front_right",[.9,.2,.801]),
               marker(2,"back_right",[.9,.6,.799]), marker(3,"back_left",[.4,.6,.8])]
    result = build_platform_box("four_corners",markers,-.74,margin_m=.01)
    assert result["dimensions_m"] == pytest.approx([.52,.42,1.54])
    assert result["position_m"] == pytest.approx([.65,.4,.03])
    assert result["top_z_m"] == pytest.approx(.8)
    assert result["quality"]["plane_rms_m"] < .001


def test_two_diagonal_corners_use_tag_orientation_for_axes():
    markers=[marker(10,"front_left",[.4,.2,.75]),marker(11,"back_right",[.9,.6,.75])]
    result=build_platform_box("two_corners",markers,-.74)
    assert result["dimensions_m"] == pytest.approx([.5,.4,1.49])
    assert result["position_m"] == pytest.approx([.65,.4,.005])


def test_center_tag_mode_uses_manual_dimensions_and_tag_yaw():
    result=build_platform_box("center_dimensions",[marker(3,"center",[.6,-.1,.8],math.pi/2)],-.74,width_m=.6,depth_m=.3)
    assert result["dimensions_m"] == pytest.approx([.6,.3,1.54])
    assert result["orientation_xyzw"][2] == pytest.approx(math.sqrt(.5))
    assert result["position_m"] == pytest.approx([.6,-.1,.03])


def test_rejects_tags_not_facing_up():
    bad=marker(1,"center",[0,0,.5]);bad["orientation_xyzw"]=[math.sin(math.pi/4),0,0,math.cos(math.pi/4)]
    with pytest.raises(ValueError,match="must face up"):
        build_platform_box("center_dimensions",[bad],-.7,width_m=.4,depth_m=.4)


def test_rejects_height_spread_and_bad_corner_assignments():
    bad=[marker(0,"front_left",[0,0,.7]),marker(1,"front_right",[.5,0,.7]),
         marker(2,"back_right",[.5,.4,.8]),marker(3,"back_left",[0,.4,.7])]
    with pytest.raises(ValueError,match="Marker heights differ"):
        build_platform_box("four_corners",bad,-.7)
    labels=[marker(i,"unknown",[i*.1,0,.7]) for i in range(4)]
    with pytest.raises(ValueError,match="corner assignments"):
        build_platform_box("four_corners",labels,-.7)


def test_rejects_inconsistent_two_corner_tag_heading():
    markers=[marker(1,"front_left",[0,0,.7]),marker(2,"back_right",[.5,.4,.7],math.pi/2)]
    with pytest.raises(ValueError,match="share the same horizontal orientation"):
        build_platform_box("two_corners",markers,-.7)


def test_compose_marker_pose_applies_camera_extrinsic():
    from cr5a_ros_bridge.platform_geometry import compose_marker_pose
    # Camera is translated +1 m on base X. Rotate the camera +90° around base Z.
    half = math.sqrt(0.5)
    result = compose_marker_pose(
        [1, 0, 0], [0, 0, half, half], [0.2, 0, 0.5], [0, 0, 0, 1]
    )
    assert result["position_m"] == pytest.approx([1.0, 0.2, 0.5])
    assert result["orientation_xyzw"] == pytest.approx([0, 0, half, half])
