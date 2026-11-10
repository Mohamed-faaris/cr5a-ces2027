"""Optional YOLO-seg RGB-D detector and BoxerNet 3D-box adapter for CR5A."""
from __future__ import annotations

import json
import math
from pathlib import Path
import time
from typing import Any

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformException, TransformListener

from .model_registry import resolve_model_checkpoint
from .vision_geometry import (
    depth_mask_to_camera_points,
    oriented_box_from_base_points,
    parse_hand_eye_yaml,
    quaternion_multiply,
    rotation_from_quaternion,
    transform_points,
)


class Cr5aVisionNode(Node):
    """A camera/TF-aware vision node; it never sends robot motion commands."""

    def __init__(self) -> None:
        super().__init__("cr5a_vision")
        self.declare_parameter("camera_topic", "/cr5/wrist_camera/color/image_raw")
        self.declare_parameter("depth_topic", "/cr5/wrist_camera/aligned_depth_to_color/image_raw")
        self.declare_parameter("camera_info_topic", "/cr5/wrist_camera/color/camera_info")
        self.declare_parameter("base_frame", "dummy_link")
        self.declare_parameter("detector_backend", "yolo_seg")
        self.declare_parameter("model_family", "yoloe-26-seg")
        self.declare_parameter("model_size", "n")
        self.declare_parameter("image_size", 640)
        self.declare_parameter("class_prompts", "Coca-Cola can, Pringles can, cardboard box, wooden box")
        self.declare_parameter("yolo_weights_path", "")
        self.declare_parameter("confidence_threshold", 0.35)
        self.declare_parameter("iou_threshold", 0.45)
        self.declare_parameter("min_depth_m", 0.12)
        self.declare_parameter("max_depth_m", 2.5)
        self.declare_parameter("max_fps", 4.0)
        self.declare_parameter("max_detections", 30)
        self.declare_parameter("boxernet_topic", "/boxernet/detections_3d")
        self.declare_parameter("hand_eye_transform_yaml", "")
        self.declare_parameter("hand_eye_parent_frame", "Link6")
        self.declare_parameter("preview_width", 800)
        self.declare_parameter("jpeg_quality", 75)
        self.declare_parameter("tf_lookup_timeout_sec", 0.25)

        self._bridge = CvBridge()
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._static_tf = StaticTransformBroadcaster(self)
        self._active = False
        self._model = None
        self._sync = None
        self._image_sub = None
        self._depth_sub = None
        self._camera_info_sub = None
        self._boxernet_sub = None
        self._latest_camera_info: CameraInfo | None = None
        self._last_frame_time = 0.0
        self._frame_count = 0
        self._detections: list[dict[str, Any]] = []
        self._camera_frame = ""
        self._error = ""
        self._hand_eye_frames: tuple[str, str] | None = None
        self._single_shot = False

        self._status_pub = self.create_publisher(String, "/cr5/vision/status", 10)
        self._detections_pub = self.create_publisher(String, "/cr5/vision/detections", 10)
        self._preview_pub = self.create_publisher(CompressedImage, "/cr5/vision/annotated/compressed", 2)
        self._camera_info_qos = QoSProfile(
            depth=5, reliability=ReliabilityPolicy.BEST_EFFORT
        )
        self.create_service(Trigger, "/cr5a_vision/start", self._start)
        self.create_service(Trigger, "/cr5a_vision/stop", self._stop)
        self.create_service(Trigger, "/cr5a_vision/detect_once", self._detect_once)
        self.create_timer(1.0, self._publish_status)
        self._publish_status()

    def _parameter(self, name: str) -> Any:
        return self.get_parameter(name).value

    def _status(self, state: str, message: str = "") -> None:
        self._error = message if state == "error" else ""
        payload = {
            "state": state,
            "active": self._active,
            "backend": self._parameter("detector_backend"),
            "model_family": self._parameter("model_family"),
            "model_size": getattr(self, "_model_size", self._parameter("model_size")),
            "image_size": self._parameter("image_size"),
            "model_name": getattr(self, "_model_name", ""),
            "message": message,
            "error": self._error,
            "base_frame": self._parameter("base_frame"),
            "camera_frame": self._camera_frame,
            "camera_info_ready": self._latest_camera_info is not None,
            "calibration_loaded": self._hand_eye_frames is not None,
            "hand_eye_frames": list(self._hand_eye_frames) if self._hand_eye_frames else None,
            "detections": len(self._detections),
            "frame_count": self._frame_count,
            "camera_topic": self._parameter("camera_topic"),
            "depth_topic": self._parameter("depth_topic"),
        }
        msg = String()
        msg.data = json.dumps(payload, allow_nan=False)
        self._status_pub.publish(msg)

    def _publish_status(self) -> None:
        state = "error" if self._error else ("active" if self._active else "stopped")
        message = self._error or ("Streaming RGB-D detections" if self._active else "Vision stopped")
        self._status(state, message)

    def _apply_hand_eye_calibration(self) -> None:
        document = str(self._parameter("hand_eye_transform_yaml") or "").strip()
        if not document:
            self._hand_eye_frames = None
            return
        transform = parse_hand_eye_yaml(document)
        parent = str(transform["parent"])
        child = str(transform["child"])
        expected_parent = str(self._parameter("hand_eye_parent_frame") or parent)
        if parent != expected_parent:
            raise ValueError(
                f"Hand-eye parent frame is {parent!r}, but configured mount frame is {expected_parent!r}"
            )
        base_frame = str(self._parameter("base_frame"))
        try:
            self._tf_buffer.lookup_transform(base_frame, child, Time(), timeout=Duration(seconds=0.05))
            camera_path_exists = True
        except TransformException:
            camera_path_exists = False
        if camera_path_exists:
            try:
                live = self._tf_buffer.lookup_transform(parent, child, Time(), timeout=Duration(seconds=0.05))
            except TransformException as exc:
                raise ValueError(
                    f"Camera frame {child} is already connected to {base_frame} through a different TF parent. "
                    f"Disable the competing static camera transform before applying {parent} -> {child}."
                ) from exc
            live_t = live.transform.translation
            live_q = live.transform.rotation
            expected_t = transform["translation_m"]
            expected_q = transform["quaternion_xyzw"]
            translation_error = math.sqrt(
                (live_t.x - expected_t[0]) ** 2
                + (live_t.y - expected_t[1]) ** 2
                + (live_t.z - expected_t[2]) ** 2
            )
            live_q_values = [live_q.x, live_q.y, live_q.z, live_q.w]
            quaternion_dot = abs(sum(a * b for a, b in zip(live_q_values, expected_q)))
            quaternion_dot = min(1.0, max(-1.0, quaternion_dot))
            rotation_error_deg = math.degrees(2.0 * math.acos(quaternion_dot))
            if translation_error > 0.005 or rotation_error_deg > 2.0:
                raise ValueError(
                    f"A TF path already exists for {child}, but differs from the imported hand-eye calibration "
                    f"(translation Δ={translation_error * 1000:.1f} mm, rotation Δ={rotation_error_deg:.1f}°). "
                    "Remove or disable the competing URDF/static camera transform before using this calibration."
                )
            self._hand_eye_frames = (parent, child)
            self.get_logger().info(f"Existing TF {parent} -> {child} matches imported hand-eye calibration")
            return
        message = TransformStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = parent
        message.child_frame_id = child
        message.transform.translation.x = float(transform["translation_m"][0])
        message.transform.translation.y = float(transform["translation_m"][1])
        message.transform.translation.z = float(transform["translation_m"][2])
        (message.transform.rotation.x, message.transform.rotation.y,
         message.transform.rotation.z, message.transform.rotation.w) = transform["quaternion_xyzw"]
        self._static_tf.sendTransform(message)
        self._hand_eye_frames = (parent, child)
        self.get_logger().info(f"Published calibrated hand-eye TF: {parent} -> {child}")

    def _load_yolo_model(self) -> None:
        family = str(self._parameter("model_family")).strip().lower()
        size = str(self._parameter("model_size")).strip().lower()
        size = {"nano": "n", "small": "s", "medium": "m", "large": "l", "xlarge": "x"}.get(size, size)
        custom_path = str(self._parameter("yolo_weights_path") or "").strip()
        try:
            model_name = resolve_model_checkpoint(family, size, custom_path)
        except ValueError as exc:
            raise RuntimeError(str(exc)) from exc

        try:
            if family == "yoloe-26-seg":
                from ultralytics import YOLOE
                model = YOLOE(model_name)
            else:
                from ultralytics import YOLO
                model = YOLO(model_name, task="segment")
        except ImportError as exc:
            raise RuntimeError(
                "Ultralytics is not installed in the vision Python environment. "
                "Install cr5a_vision/requirements-yolo.txt and restart the node."
            ) from exc
        task = getattr(model, "task", "segment")
        if task != "segment":
            raise RuntimeError(f"Checkpoint task is {task!r}; choose an instance-segmentation checkpoint")

        prompts = [value.strip() for value in str(self._parameter("class_prompts") or "").split(",") if value.strip()]
        if family == "yoloe-26-seg":
            if not prompts:
                raise RuntimeError("YOLOE-26 requires at least one comma-separated class prompt")
            model.set_classes(prompts)
        self._model = model
        self._model_name = model_name
        self._model_family = family
        self._model_size = size
        self._class_prompts = prompts if family == "yoloe-26-seg" else []
        self.get_logger().info(
            f"Loaded {family} size={size} checkpoint={model_name} imgsz={int(self._parameter('image_size'))}"
        )

    def _start(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        if self._active:
            response.success = True
            response.message = "Vision is already running"
            return response
        backend = str(self._parameter("detector_backend")).lower()
        if backend not in ("yolo_seg", "boxernet"):
            response.success = False
            response.message = "detector_backend must be yolo_seg or boxernet"
            self._status("error", response.message)
            return response
        try:
            self._apply_hand_eye_calibration()
            if backend == "yolo_seg":
                self._load_yolo_model()
                self._subscribe_rgbd()
                message = "YOLO segmentation running; waiting for synchronized RGB, depth and CameraInfo"
            else:
                topic = str(self._parameter("boxernet_topic"))
                self._boxernet_sub = self.create_subscription(String, topic, self._boxernet_callback, 10)
                # The preview stream is independent of the external BoxerNet output adapter.
                self._image_sub = self.create_subscription(
                    Image, str(self._parameter("camera_topic")), self._preview_callback, self._camera_info_qos
                )
                message = f"BoxerNet adapter connected to {topic}; waiting for 3D box messages"
            self._active = True
            self._status("active", message)
            response.success = True
            response.message = message
        except Exception as exc:  # Keep the ROS node alive and surface the reason to the UI.
            self._active = False
            self._status("error", str(exc))
            response.success = False
            response.message = str(exc)
        return response

    def _stop(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        self._active = False
        self._sync = None
        self._image_sub = None
        self._depth_sub = None
        self._camera_info_sub = None
        self._boxernet_sub = None
        self._model = None
        self._status("stopped", "Vision stopped")
        response.success = True
        response.message = "Vision stopped"
        return response

    def _detect_once(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        if not self._active:
            response.success = False
            response.message = "Start vision before requesting a detection"
            return response
        self._single_shot = True
        response.success = True
        response.message = "One-shot detection armed; waiting for the next synchronized camera frame"
        return response

    def _subscribe_rgbd(self) -> None:
        self._latest_camera_info = None
        self._camera_info_sub = self.create_subscription(
            CameraInfo, str(self._parameter("camera_info_topic")),
            self._camera_info_callback, self._camera_info_qos,
        )
        self._image_sub = Subscriber(self, Image, str(self._parameter("camera_topic")), qos_profile=self._camera_info_qos)
        self._depth_sub = Subscriber(self, Image, str(self._parameter("depth_topic")), qos_profile=self._camera_info_qos)
        self._sync = ApproximateTimeSynchronizer(
            [self._image_sub, self._depth_sub], queue_size=5, slop=0.12
        )
        self._sync.registerCallback(self._rgbd_callback)

    def _camera_info_callback(self, message: CameraInfo) -> None:
        self._latest_camera_info = message
        self._camera_frame = str(message.header.frame_id or self._camera_frame)

    def _preview_callback(self, image_message: Image) -> None:
        try:
            image = self._bridge.imgmsg_to_cv2(image_message, desired_encoding="bgr8")
            self._publish_preview(image, image_message.header)
        except Exception as exc:
            self.get_logger().warning(f"Could not create vision preview: {exc}")

    def _rgbd_callback(self, image_message: Image, depth_message: Image) -> None:
        if not self._active or str(self._parameter("detector_backend")).lower() != "yolo_seg":
            return
        now = time.monotonic()
        max_fps = max(0.2, float(self._parameter("max_fps")))
        if now - self._last_frame_time < 1.0 / max_fps:
            return
        self._last_frame_time = now
        try:
            if self._latest_camera_info is None:
                raise RuntimeError("Waiting for CameraInfo before 3D deprojection")
            image = self._bridge.imgmsg_to_cv2(image_message, desired_encoding="bgr8")
            depth = self._bridge.imgmsg_to_cv2(depth_message, desired_encoding="passthrough")
            camera_info = self._latest_camera_info
            annotated, detections = self._run_yolo_seg(image, depth, depth_message.encoding, camera_info, image_message.header)
            self._detections = detections
            self._frame_count += 1
            self._publish_detections(detections, image_message.header)
            self._publish_preview(annotated, image_message.header)
            self._status("active", f"YOLO-seg: {len(detections)} 3D boxes from frame {self._frame_count}")
            if self._single_shot:
                self._single_shot = False
                self._active = False
                self._sync = None
                self._image_sub = None
                self._depth_sub = None
                self._camera_info_sub = None
                self._status("stopped", "One-shot detection completed")
        except (TransformException, ValueError, RuntimeError) as exc:
            self._status("error", str(exc))
        except Exception as exc:
            self.get_logger().exception("Unexpected error in YOLO-seg callback")
            self._status("error", str(exc))

    def _run_yolo_seg(self, image: np.ndarray, depth: np.ndarray, depth_encoding: str,
                      camera_info: CameraInfo, header: Any) -> tuple[np.ndarray, list[dict[str, Any]]]:
        if self._model is None:
            raise RuntimeError("YOLO segmentation model is not loaded")
        predictions = self._model.predict(
            source=image,
            conf=float(self._parameter("confidence_threshold")),
            iou=float(self._parameter("iou_threshold")),
            imgsz=int(self._parameter("image_size")),
            max_det=int(self._parameter("max_detections")),
            task="segment", verbose=False,
        )
        annotated = image.copy()
        result = predictions[0] if predictions else None
        if result is None or result.boxes is None or result.masks is None:
            return annotated, []
        boxes = result.boxes
        masks = result.masks.data.detach().cpu().numpy()
        names = getattr(result, "names", {}) or {}
        frame_id = str(header.frame_id or camera_info.header.frame_id)
        self._camera_frame = frame_id
        transform = self._lookup_camera_to_base(frame_id, header.stamp)
        tf_translation, tf_quaternion = self._transform_values(transform)
        detections: list[dict[str, Any]] = []
        limit = int(self._parameter("max_detections"))
        height, width = image.shape[:2]
        for index, mask_value in enumerate(masks[:limit]):
            mask = cv2.resize(mask_value.astype(np.float32), (width, height), interpolation=cv2.INTER_NEAREST) > 0.5
            score = float(boxes.conf[index].detach().cpu().item()) if boxes.conf is not None else 0.0
            class_id = int(boxes.cls[index].detach().cpu().item()) if boxes.cls is not None else -1
            class_name = str(names.get(class_id, class_id))
            points_camera = depth_mask_to_camera_points(
                mask, depth, list(camera_info.k), depth_encoding,
                float(self._parameter("min_depth_m")), float(self._parameter("max_depth_m")),
            )
            if points_camera.shape[0] < 20:
                continue
            points_base = transform_points(points_camera, tf_translation, tf_quaternion)
            box = oriented_box_from_base_points(points_base)
            x1, y1, x2, y2 = [int(round(value)) for value in boxes.xyxy[index].detach().cpu().tolist()]
            cv2.rectangle(annotated, (x1, y1), (x2, y2), (30, 190, 90), 2)
            label = f"{class_name} {score:.2f}"
            cv2.putText(annotated, label, (x1, max(18, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (30, 190, 90), 2)
            detections.append({
                "id": f"yolo_{self._frame_count + 1}_{index}",
                "class_name": class_name, "confidence": score,
                "shape": "box", "frame_id": str(self._parameter("base_frame")),
                "position_m": box["position_m"], "centroid_m": box["position_m"],
                "dimensions_m": box["dimensions_m"], "orientation_xyzw": box["orientation_xyzw"],
                "yaw_deg": box["yaw_deg"], "valid_3d": True,
                "source_backend": "yolo_seg", "depth_points": int(points_camera.shape[0]),
            })
        return annotated, detections

    def _lookup_camera_to_base(self, camera_frame: str, stamp: Any) -> TransformStamped:
        base_frame = str(self._parameter("base_frame"))
        if not camera_frame:
            raise TransformException("Image header has no optical frame_id")
        time_value = Time.from_msg(stamp) if stamp and (stamp.sec or stamp.nanosec) else Time()
        return self._tf_buffer.lookup_transform(
            base_frame, camera_frame, time_value,
            timeout=Duration(seconds=float(self._parameter("tf_lookup_timeout_sec"))),
        )

    @staticmethod
    def _transform_values(transform: TransformStamped) -> tuple[list[float], list[float]]:
        t = transform.transform.translation
        q = transform.transform.rotation
        return [float(t.x), float(t.y), float(t.z)], [float(q.x), float(q.y), float(q.z), float(q.w)]

    def _boxernet_callback(self, message: String) -> None:
        if not self._active:
            return
        try:
            payload = json.loads(message.data)
            frame_id = str(payload.get("frame_id") or "")
            if not frame_id:
                raise ValueError("BoxerNet payload must include frame_id")
            stamp_payload = payload.get("stamp") or {}
            stamp = self.get_clock().now().to_msg()
            if isinstance(stamp_payload, dict) and (stamp_payload.get("sec") or stamp_payload.get("nanosec")):
                stamp.sec = int(stamp_payload.get("sec", 0))
                stamp.nanosec = int(stamp_payload.get("nanosec", 0))
            transform = self._lookup_camera_to_base(frame_id, stamp)
            translation, quaternion = self._transform_values(transform)
            detections = []
            for index, item in enumerate(payload.get("objects", [])):
                center = item.get("center_m", item.get("position_m", item.get("centroid_m")))
                dimensions = item.get("dimensions_m", item.get("size_m"))
                orientation = item.get("orientation_xyzw")
                if orientation is None:
                    yaw_degrees = float(item.get("yaw_deg", 0.0))
                    half_yaw = math.radians(yaw_degrees) / 2.0
                    orientation = [0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)]
                if not isinstance(center, list) or len(center) != 3 or not isinstance(dimensions, list) or len(dimensions) != 3:
                    continue
                if any(not math.isfinite(float(value)) or float(value) <= 0 for value in dimensions):
                    continue
                center_values = [float(value) for value in center]
                if not all(math.isfinite(value) for value in center_values):
                    continue
                confidence = float(item.get("confidence", 1.0))
                if not math.isfinite(confidence) or not 0.0 <= confidence <= 1.0:
                    continue
                center_base = transform_points(np.asarray([center_values], dtype=float), translation, quaternion)[0]
                orientation_base = quaternion_multiply(quaternion, orientation)
                qx, qy, qz, qw = orientation_base
                yaw = math.degrees(math.atan2(2 * (qw*qz + qx*qy), 1 - 2 * (qy*qy + qz*qz)))
                position = [float(value) for value in center_base]
                detections.append({
                    "id": str(item.get("id") or f"boxernet_{self._frame_count}_{index}"),
                    "class_name": str(item.get("class_name") or item.get("label") or "object"),
                    "confidence": confidence, "shape": "box",
                    "uncertainty": item.get("uncertainty"),
                    "frame_id": str(self._parameter("base_frame")),
                    "position_m": position, "centroid_m": position,
                    "dimensions_m": [float(value) for value in dimensions],
                    "orientation_xyzw": orientation_base, "yaw_deg": yaw,
                    "valid_3d": True, "source_backend": "boxernet",
                })
            self._detections = detections
            self._frame_count += 1
            self._publish_detections(detections, type("Header", (), {"frame_id": frame_id, "stamp": stamp})())
            self._status("active", f"BoxerNet adapter: {len(detections)} 3D boxes")
            if self._single_shot:
                self._single_shot = False
                self._active = False
                self._boxernet_sub = None
                self._image_sub = None
                self._status("stopped", "One-shot BoxerNet import completed")
            if self._single_shot:
                self._single_shot = False
                self._active = False
                self._boxernet_sub = None
                self._image_sub = None
                self._status("stopped", "One-shot BoxerNet import completed")
        except (ValueError, TypeError, TransformException, KeyError) as exc:
            self._status("error", f"BoxerNet adapter: {exc}")

    def _publish_detections(self, detections: list[dict[str, Any]], header: Any) -> None:
        msg = String()
        msg.data = json.dumps({
            "frame_id": str(self._parameter("base_frame")),
            "camera_frame": str(header.frame_id),
            "frame_count": self._frame_count,
            "objects": detections,
        }, allow_nan=False)
        self._detections_pub.publish(msg)

    def _publish_preview(self, image: np.ndarray, header: Any) -> None:
        preview_width = max(320, int(self._parameter("preview_width")))
        if image.shape[1] > preview_width:
            scale = preview_width / image.shape[1]
            image = cv2.resize(image, (preview_width, max(1, int(image.shape[0] * scale))))
        quality = min(95, max(40, int(self._parameter("jpeg_quality"))))
        ok, encoded = cv2.imencode(".jpg", image, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
        if not ok:
            return
        msg = CompressedImage()
        msg.header = header
        msg.format = "jpeg"
        msg.data = encoded.tobytes()
        self._preview_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Cr5aVisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
