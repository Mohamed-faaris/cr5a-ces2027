# CR5A Vision module

The Vision tab is a separate ROS node and does not send motion commands. It offers two inference paths and one box-import adapter.

## Segmentation model choices

| Family | Size options | Typical use |
|---|---|---|
| YOLO26-seg | n, s, m, l, x | Latest Ultralytics instance-segmentation checkpoints; fixed pretrained COCO categories |
| YOLOE-26-seg | n, s, m, l, x | Open-vocabulary segmentation from comma-separated text prompts (recommended when labels include cans and specific packaging) |
| YOLO11-seg | n, s, m, l, x | Previous generation reference |
| YOLOv8-seg | n, s, m, l, x | Legacy reference |
| Custom | Local segmentation checkpoint | Fine-tuned checkpoint supplied by the operator |

The default is `yoloe-26n-seg.pt` with text prompts for Coca-Cola cans, Pringles tubes and boxes, image size 640. For fixed COCO categories without prompts, select YOLO26-seg instead. Ultralytics downloads an official checkpoint the first time the selected model name is loaded. Increase model scale or image resolution only if the compute budget permits. The UI exposes model scale separately from image resolution.

Install optional inference dependencies in the Python environment used to launch the ROS node:

```bash
python3 -m pip install -r src/cr5a-ces2027/cr5a_vision/requirements-yolo.txt
```

For Ubuntu images with externally-managed Python, create a dedicated virtual environment with system ROS packages visible, activate it before starting the vision launch, and install the optional requirements there. Verify the selected PyTorch build matches the host's CUDA runtime; CPU inference is available but slower.

YOLO26 and YOLOE-26 model names and size variants follow the official Ultralytics model catalog. Check the Ultralytics AGPL-3.0 licensing terms for the package and weights before deployment.

## RGB-D and calibrated TF

YOLO-seg uses synchronized RGB and aligned depth plus `CameraInfo.K`. The segmentation mask is deprojected to camera points, filtered by depth bounds, then transformed with a TF lookup at the image timestamp into `base_frame` (default `dummy_link`). A missing image frame, CameraInfo, depth stream, or base-to-camera TF blocks 3D output; the node does not guess metric depth or extrinsics.

Import the calibrated hand-eye YAML in the Vision tab. The currently accepted file schema is:

```yaml
calibrated: true
frames:
  parent: Link6
  child: cr5_wrist_camera_color_optical_frame
transform_parent_to_child:
  matrix:
    - [1, 0, 0, 0.02]
    - [0, 1, 0, 0.00]
    - [0, 0, 1, 0.10]
    - [0, 0, 0, 1]
```

The translation is metres and the matrix is `T_parent_child`. Change the configured parent frame if the real calibration mount is not `Link6`. The node publishes the transform as static TF. If an identical camera transform is already published by the robot description, the existing TF chain is used.

## BoxerNet 3D-box adapter

BoxerNet is not the same as a 2D detector. It expects 2D proposals, image features, camera/pose information and optional semi-dense depth, then predicts 3D oriented boxes. The repository does not include the upstream BoxerNet source or checkpoint. The Vision tab accepts a separate BoxerNet runner's metric 3D boxes on `/boxernet/detections_3d` (configurable):

```json
{
  "frame_id": "cr5_wrist_camera_color_optical_frame",
  "objects": [
    {
      "id": "boxer_001",
      "class_name": "can",
      "confidence": 0.93,
      "center_m": [0.12, -0.03, 0.62],
      "dimensions_m": [0.066, 0.066, 0.122],
      "yaw_deg": 15.0
    }
  ]
}
```

`orientation_xyzw` can be supplied instead of `yaw_deg`. The node transforms the OBB centre/orientation from `frame_id` to the configured base frame and preserves box dimensions/uncertainty. It then publishes normalized boxes for MoveIt and direct Pick & Place handoff.

The upstream Facebook Research Boxer weights are CC-BY-NC-4.0 (non-commercial). The full pipeline is multi-view/pose-aware and considerably heavier than YOLO-seg; verify upstream requirements and licensing before installing it on the robot computer.

## Conveyor-to-table routing

The Vision tab offers a configurable axis-aligned conveyor zone in the MoveIt base frame. If an object's transformed centroid lies inside the zone, the direct Pick & Place handoff recommends the configured table as destination. This is a routing suggestion only: the table collision geometry, clearance, reachability and grasp still require planning validation.

For an isolated optional Ultralytics runtime (recommended rather than adding PyTorch to ROS system Python), run from the workspace root:

```bash
bash src/cr5a-ces2027/cr5a_vision/scripts/setup_vision_runtime.sh
```

The demo's Vision tmux window automatically adds that virtual environment's site-packages to `PYTHONPATH` while continuing to use the ROS system interpreter. It also starts without the optional runtime; pressing **Start vision** then reports the missing dependency clearly.
