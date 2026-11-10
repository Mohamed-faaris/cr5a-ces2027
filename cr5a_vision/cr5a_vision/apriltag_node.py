"""Optional OpenCV AprilTag pose detector publishing tag_<id> TF frames."""
from __future__ import annotations

import json
import math
import time
from typing import Any

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster


FAMILIES = {
    "tag36h11": cv2.aruco.DICT_APRILTAG_36h11,
    "tag25h9": cv2.aruco.DICT_APRILTAG_25h9,
    "tag16h5": cv2.aruco.DICT_APRILTAG_16h5,
}


def quaternion_from_rotation(matrix: np.ndarray) -> list[float]:
    m = np.asarray(matrix, dtype=float)
    trace = float(np.trace(m))
    if trace > 0:
        scale = math.sqrt(trace + 1.0) * 2.0
        q = [(m[2, 1]-m[1, 2])/scale, (m[0, 2]-m[2, 0])/scale,
             (m[1, 0]-m[0, 1])/scale, 0.25*scale]
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        scale = math.sqrt(1.0+m[0, 0]-m[1, 1]-m[2, 2])*2
        q = [0.25*scale, (m[0, 1]+m[1, 0])/scale,
             (m[0, 2]+m[2, 0])/scale, (m[2, 1]-m[1, 2])/scale]
    elif m[1, 1] > m[2, 2]:
        scale = math.sqrt(1.0+m[1, 1]-m[0, 0]-m[2, 2])*2
        q = [(m[0, 1]+m[1, 0])/scale, 0.25*scale,
             (m[1, 2]+m[2, 1])/scale, (m[0, 2]-m[2, 0])/scale]
    else:
        scale = math.sqrt(1.0+m[2, 2]-m[0, 0]-m[1, 1])*2
        q = [(m[0, 2]+m[2, 0])/scale, (m[1, 2]+m[2, 1])/scale,
             0.25*scale, (m[1, 0]-m[0, 1])/scale]
    norm = math.sqrt(sum(value*value for value in q))
    if norm < 1e-9 or not math.isfinite(norm):
        raise ValueError("AprilTag pose produced an invalid rotation")
    return [value/norm for value in q]


class AprilTagDetectorNode(Node):
    def __init__(self):
        super().__init__("cr5a_apriltag_detector")
        self.declare_parameter("camera_topic", "/cr5/wrist_camera/color/image_raw")
        self.declare_parameter("camera_info_topic", "/cr5/wrist_camera/color/camera_info")
        self.declare_parameter("tag_family", "tag36h11")
        self.declare_parameter("tag_size_m", 0.04)
        self.declare_parameter("tag_ids", "")
        self.declare_parameter("frame_prefix", "tag_")
        self.declare_parameter("max_fps", 12.0)
        self.declare_parameter("jpeg_quality", 75)
        self._bridge = CvBridge()
        self._active = False
        self._camera_info: CameraInfo | None = None
        self._last_time = 0.0
        self._frame_count = 0
        self._detection_count = 0
        self._last_ids: list[int] = []
        self._error = ""
        self._last_message = "Detector stopped"
        self._status_pub = self.create_publisher(String, "/cr5/apriltag/status", 10)
        self._detections_pub = self.create_publisher(String, "/cr5/apriltag/detections", 10)
        self._preview_pub = self.create_publisher(CompressedImage, "/cr5/apriltag/annotated/compressed", 2)
        self._tf = TransformBroadcaster(self)
        sensor_qos = QoSProfile(depth=4, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._camera_info_sub = self.create_subscription(
            CameraInfo, self._parameter("camera_info_topic"), self._camera_info_callback, sensor_qos
        )
        self._image_sub = self.create_subscription(
            Image, self._parameter("camera_topic"), self._image_callback, sensor_qos
        )
        self.create_service(Trigger, "/cr5a_apriltag/start", self._start)
        self.create_service(Trigger, "/cr5a_apriltag/stop", self._stop)
        self.add_on_set_parameters_callback(self._validate_parameters)
        self.create_timer(1.0, self._publish_status)
        self._publish_status()

    def _parameter(self, name: str) -> Any:
        return self.get_parameter(name).value

    def _validate_parameters(self, parameters) -> SetParametersResult:
        for item in parameters:
            if item.name == "tag_family" and item.value not in FAMILIES:
                return SetParametersResult(successful=False, reason=f"tag_family must be one of {sorted(FAMILIES)}")
            if item.name == "tag_size_m" and (not math.isfinite(float(item.value)) or not 0.005 <= float(item.value) <= 0.5):
                return SetParametersResult(successful=False, reason="tag_size_m must be within 0.005–0.5 metres")
            if item.name == "max_fps" and (not math.isfinite(float(item.value)) or not 0.1 <= float(item.value) <= 60):
                return SetParametersResult(successful=False, reason="max_fps must be within 0.1–60")
            if item.name in ("camera_topic", "camera_info_topic") and (not isinstance(item.value, str) or not item.value.startswith("/")):
                return SetParametersResult(successful=False, reason=f"{item.name} must be an absolute ROS topic")
        return SetParametersResult(successful=True)

    def _camera_info_callback(self, message: CameraInfo):
        self._camera_info = message

    def _start(self, _request, response):
        try:
            family = str(self._parameter("tag_family"))
            if family not in FAMILIES:
                raise ValueError(f"Unsupported AprilTag family: {family}")
            dictionary = cv2.aruco.getPredefinedDictionary(FAMILIES[family])
            parameters = cv2.aruco.DetectorParameters_create()
            parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
            self._dictionary = dictionary
            self._detector_parameters = parameters
            self._active = True
            self._error = ""
            self._last_message = "Waiting for camera intrinsics and images"
            response.success = True
            response.message = f"AprilTag detector started · {family} · {self._parameter('tag_size_m'):.3f} m tags"
        except Exception as exc:
            self._active = False
            self._error = str(exc)
            response.success = False
            response.message = self._error
        self._publish_status()
        return response

    def _stop(self, _request, response):
        self._active = False
        self._last_message = "Detector stopped"
        response.success = True
        response.message = "AprilTag detection stopped; TF frames remain available until their last TF cache entry expires"
        self._publish_status()
        return response

    def _publish_status(self):
        payload = {
            "active": self._active, "state": "error" if self._error else ("running" if self._active else "stopped"),
            "message": self._last_message, "error": self._error,
            "camera_info_ready": self._camera_info is not None,
            "camera_topic": self._parameter("camera_topic"),
            "camera_info_topic": self._parameter("camera_info_topic"),
            "camera_frame": self._camera_info.header.frame_id if self._camera_info else "",
            "tag_family": self._parameter("tag_family"), "tag_size_m": self._parameter("tag_size_m"),
            "detections": self._detection_count, "last_ids": self._last_ids,
            "frame_count": self._frame_count,
        }
        msg = String(); msg.data = json.dumps(payload, allow_nan=False); self._status_pub.publish(msg)

    def _image_callback(self, message: Image):
        if not self._active:
            return
        now = time.monotonic()
        max_fps = max(0.1, float(self._parameter("max_fps")))
        if now-self._last_time < 1.0/max_fps:
            return
        self._last_time = now
        if self._camera_info is None:
            self._error = "Waiting for CameraInfo; pose estimation requires real camera intrinsics"
            self._last_message = self._error
            return
        frame_id = str(message.header.frame_id or self._camera_info.header.frame_id)
        if not frame_id:
            self._error = "Image frame_id is empty; cannot publish tag TF"
            return
        try:
            image = self._bridge.imgmsg_to_cv2(message, desired_encoding="mono8")
            corners, ids, _rejected = cv2.aruco.detectMarkers(image, self._dictionary, parameters=self._detector_parameters)
            self._frame_count += 1
            if ids is None or len(ids) == 0:
                self._detection_count = 0
                self._last_ids = []
                self._last_message = "No AprilTags detected in latest frame"
                self._detections_pub.publish(String(data=json.dumps({"frame_id": frame_id, "markers": [], "stamp": {"sec": message.header.stamp.sec, "nanosec": message.header.stamp.nanosec}})))
                self._publish_preview(image, corners, None, message.header)
                self._publish_status()
                return
            camera = self._camera_info
            matrix = np.asarray(camera.k, dtype=np.float64).reshape(3, 3)
            distortion = np.asarray(camera.d, dtype=np.float64)
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, float(self._parameter("tag_size_m")), matrix, distortion)
            requested = {int(value.strip()) for value in str(self._parameter("tag_ids")).split(",") if value.strip()}
            prefix = str(self._parameter("frame_prefix"))
            transforms = []
            markers = []
            selected_corners = []
            selected_rvecs, selected_tvecs = [], []
            detected_ids = []
            for index, raw_id in enumerate(ids.flatten().tolist()):
                marker_id = int(raw_id)
                if requested and marker_id not in requested:
                    continue
                rvec = rvecs[index].reshape(3)
                tvec = tvecs[index].reshape(3)
                rotation, _ = cv2.Rodrigues(rvec)
                quaternion = quaternion_from_rotation(rotation)
                tf = TransformStamped()
                tf.header = message.header
                tf.header.frame_id = frame_id
                tf.child_frame_id = f"{prefix}{marker_id}"
                tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = map(float, tvec)
                tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w = quaternion
                transforms.append(tf)
                detected_ids.append(marker_id)
                markers.append({"id": marker_id, "frame_id": tf.child_frame_id,
                                "position_m": [float(x) for x in tvec], "orientation_xyzw": quaternion,
                                "decision_margin": None})
                selected_corners.append(corners[index]); selected_rvecs.append(rvecs[index]); selected_tvecs.append(tvecs[index])
            if transforms:
                self._tf.sendTransform(transforms)
            self._last_ids = detected_ids
            self._detection_count = len(detected_ids)
            self._error = ""
            self._last_message = f"Detected {len(detected_ids)} AprilTag(s) in {frame_id}"
            self._detections_pub.publish(String(data=json.dumps({
                "frame_id": frame_id, "markers": markers,
                "stamp": {"sec": message.header.stamp.sec, "nanosec": message.header.stamp.nanosec},
            }, allow_nan=False)))
            self._publish_preview(image, selected_corners, (selected_rvecs, selected_tvecs), message.header)
            self._publish_status()
        except Exception as exc:
            self._error = str(exc)
            self._last_message = f"AprilTag detection error: {exc}"
            self.get_logger().warning(self._last_message)
            self._publish_status()

    def _publish_preview(self, image, corners, poses, header):
        annotated = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        if corners:
            cv2.aruco.drawDetectedMarkers(annotated, corners)
        if poses:
            matrix = np.asarray(self._camera_info.k, dtype=np.float64).reshape(3, 3)
            distortion = np.asarray(self._camera_info.d, dtype=np.float64)
            for rvec, tvec in zip(*poses):
                cv2.drawFrameAxes(annotated, matrix, distortion, rvec, tvec, float(self._parameter("tag_size_m"))*0.5)
        quality = int(self._parameter("jpeg_quality"))
        success, encoded = cv2.imencode(".jpg", annotated, [int(cv2.IMWRITE_JPEG_QUALITY), max(35, min(95, quality))])
        if not success:
            return
        output = CompressedImage()
        output.header = header
        output.format = "jpeg"
        output.data = encoded.tobytes()
        self._preview_pub.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
