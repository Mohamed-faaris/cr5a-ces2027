from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument("camera_topic", default_value="/cr5/wrist_camera/color/image_raw"),
        DeclareLaunchArgument("depth_topic", default_value="/cr5/wrist_camera/aligned_depth_to_color/image_raw"),
        DeclareLaunchArgument("camera_info_topic", default_value="/cr5/wrist_camera/color/camera_info"),
        DeclareLaunchArgument("base_frame", default_value="dummy_link"),
        DeclareLaunchArgument("detector_backend", default_value="yolo_seg"),
        DeclareLaunchArgument("model_family", default_value="yoloe-26-seg"),
        DeclareLaunchArgument("model_size", default_value="nano", description="Use nano/small/medium/large/xlarge here; the Vision UI uses n/s/m/l/x."),
        DeclareLaunchArgument("image_size", default_value="640"),
        DeclareLaunchArgument("yolo_weights_path", default_value=""),
        DeclareLaunchArgument("boxernet_topic", default_value="/boxernet/detections_3d"),
    ]
    node = Node(
        package="cr5a_vision", executable="cr5a_vision_node", name="cr5a_vision",
        output="screen", parameters=[{
            "camera_topic": LaunchConfiguration("camera_topic"),
            "depth_topic": LaunchConfiguration("depth_topic"),
            "camera_info_topic": LaunchConfiguration("camera_info_topic"),
            "base_frame": LaunchConfiguration("base_frame"),
            "detector_backend": LaunchConfiguration("detector_backend"),
            "model_family": LaunchConfiguration("model_family"),
            "model_size": ParameterValue(LaunchConfiguration("model_size"), value_type=str),
            "image_size": LaunchConfiguration("image_size"),
            "yolo_weights_path": LaunchConfiguration("yolo_weights_path"),
            "boxernet_topic": LaunchConfiguration("boxernet_topic"),
        }],
    )
    apriltag_node = Node(
        package="cr5a_vision", executable="cr5a_apriltag_detector", name="cr5a_apriltag_detector",
        output="screen", parameters=[{
            "camera_topic": LaunchConfiguration("camera_topic"),
            "camera_info_topic": LaunchConfiguration("camera_info_topic"),
            "tag_family": "tag36h11", "tag_size_m": 0.04,
            "tag_ids": "", "frame_prefix": "tag_",
        }],
    )
    return LaunchDescription(arguments + [node, apriltag_node])
