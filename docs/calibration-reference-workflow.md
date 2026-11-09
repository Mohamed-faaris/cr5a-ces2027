# Calibration reference workflow and legacy dataset import

The Calibration page previews existing CR5A wrist-eye datasets without duplicating
the ROS capture or solver pipeline. The reference implementation is in
`Mohamed-faaris/DOBOT_6Axis_ROS2_V4_private`, branch
`feature/v4-optimization`, under `cr5a_wrist_eye_calibration/`.

## Existing workflow findings

The legacy ROS 2 Humble package already provides:

- Synchronized D415 color image + CameraInfo capture, measured
  `base_link -> Link6` TF, and board detection.
- Per-observation JSON samples, source images, annotated images, rejected captures,
  a JSONL joint pose plan, and calibration result files.
- A spherical-dome viewpoint generator that uses the observed board pose, camera
  intrinsics/FOV, standoff, tilt, azimuth and optical roll.
- ChArUco, ArUco-grid and AprilTag-grid board detection.
- Dataset validation and OpenCV Tsai, Park, Horaud, Andreff, Daniilidis and
  joint-refinement solvers.

The reference ChArUco board is 17 rows × 24 columns, 30 mm squares, 22 mm markers,
using `aruco_dict_5x5`. The Calibration page defaults to those dimensions and a
5 px reprojection threshold. Confirm the physical print and camera settings before
using a different board.

### Reference run 002 test results

- Re-detected the ChArUco board in all 34 captured images. Recomputed mean
  reprojection RMS was 1.5349 px, matching the stored dataset metric.
- Re-ran all six hand-eye methods. All returned a result; best translation closure
  was 11.31 mm (`joint_refine`) and best rotation closure was 1.1068°
  (`horaud`). These residuals are not a measured absolute accuracy.
- Generated 6 geometric candidate targets from 288 sampled viewpoints and embedded
  all 34 compressed image thumbnails in the web preview bundle.
- The seed observation places the board center at approximately
  `[0.136, -0.921, -0.443] m` in the recorded `base_link` frame. Confirm this
  matches the real board placement and frame convention before trusting the targets.

## Generate candidate viewpoints from an existing observation

The existing generator can use the calibrated `Link6 -> camera optical` transform
from the dataset instead of its nominal mount guess. The following offline invocation
writes a separate JSONL plan and does not alter the source dataset:

```bash
cd /path/to/DOBOT_6Axis_ROS2_V4_private
PYTHONPATH=cr5a_wrist_eye_calibration python3 - <<'PY'
import yaml, numpy as np
from pathlib import Path
from cr5a_wrist_eye_calibration.pose_generator import generate_poses_from_seed

dataset = Path("cr5a_wrist_eye_calibration/data/002").resolve()
matrix = yaml.safe_load((dataset / "results/hand_eye_transform.yaml").read_text())[
    "transform_parent_to_child"
]["matrix"]
summary = generate_poses_from_seed(
    sample_path=dataset / "samples/sample_000.json",
    dataset_dir=dataset,
    output_file=Path("/tmp/generated-calibration-poses.jsonl"),
    num_poses=20,
    family="aruco_dict_5x5",
    board_type="charuco",
    columns=24, rows=17,
    marker_size_m=0.022, pitch_m=0.030, square_size_m=0.030,
    nominal_tcp_to_cam=np.asarray(matrix, dtype=np.float64),
)
print(summary)
PY
```

Important: the standalone generator call above is an offline geometry/FOV/reach-envelope
pass. It does not create a MoveIt IK client or perform collision checking. Its
`reachable_count` must not be interpreted as MoveIt-verified reachability. The
generated targets are preview-only until they are separately validated through the
active MoveIt planning scene and reviewed by an operator.

## Convert a legacy dataset for the web preview

Run the adapter from the CR5A web repository. It reads the old package's dataset
directory and writes one self-contained JSON file. It does not change source images,
start ROS, plan motion, or command the robot.

```bash
python3 scripts/import_legacy_calibration_dataset.py \
  --dataset-dir /path/to/DOBOT_6Axis_ROS2_V4_private/cr5a_wrist_eye_calibration/data/002 \
  --pose-plan /tmp/generated-calibration-poses.jsonl \
  --output cr5a_web/public/calibration/reference-dataset-002.json
```

Requirements for embedded image thumbnails: Python OpenCV (`python3-opencv`) and
PyYAML (`python3-yaml`). Add `--no-thumbnails` to import pose/metrics metadata only.
The adapter embeds compressed 480 px JPEG thumbnails; original images remain untouched.
It keeps captured observations and generated Cartesian targets in separate arrays.
Joint-only targets are not plotted because they need FK/MoveIt to produce base-frame
TCP poses.

Open **Calibration → 3D calibration workspace → Load bundled reference run 002**.
The page shows captured camera poses in the robot base frame, orientation axes,
per-sample D415 thumbnails, reprojection RMS, marker counts, solver summary and
generated TCP target poses. Selecting a captured sample shows its source image.

## Frame and safety semantics

- If `results/hand_eye_transform.yaml` is present and marked calibrated, the adapter
  composes `base_T_camera = base_T_tcp × tcp_T_camera`. The marker is the camera
  optical-frame origin, not the TCP origin.
- Without a usable hand-eye result, the adapter displays the measured TCP origin and
  labels it as TCP. It never silently treats TCP and camera frames as equivalent.
- Source frame names are preserved. A live CameraInfo frame-name mismatch is shown as
  a warning. If the dataset base frame differs from the MoveIt model base, the
  viewport uses an existing `/tf_static` transform; markers are hidden when no
  transform exists. The page never invents a static TF or camera mount transform.
- Historical samples and generated targets are reference data only. They are not
  sent to MoveIt, replayed, or activated as a calibration result.
- Reprojection RMS and hand-eye closure RMS are consistency metrics, not ground-truth
  accuracy. Validate on held-out poses before using any transform for robot motion.

## Re-run the original workflow offline

Use the original package from the legacy repository, in its ROS 2 Humble environment.
For an existing dataset, validate and solve it without launching the robot:

```bash
python3 cr5a_wrist_eye_calibration/cr5a_wrist_eye_calibration/validate_dataset.py \
  --dataset-dir cr5a_wrist_eye_calibration/data/002

python3 cr5a_wrist_eye_calibration/cr5a_wrist_eye_calibration/solve_handeye.py \
  --dataset-dir cr5a_wrist_eye_calibration/data/002 \
  --method joint_refine --max-board-rms-px 5.0 --compare-all
```

For new captures or pose generation, follow the original package README and verify
the active robot's TCP, MoveIt group, joint order, camera topics and TF tree. Do not
enable `execute_motion` on hardware until the target poses and safety state have
been independently reviewed.


## Calibration board in the 3D scene

The legacy ChArUco captures store `camera_T_board` for each accepted image. The importer reconstructs each board pose in the dataset base frame only when a calibrated `tcp_T_camera` is available, using `base_T_tcp * tcp_T_camera * camera_T_board`. It aggregates the fixed board pose across accepted detections (coordinate-wise median translation and sign-aligned quaternion mean) and records the number of contributing captures and translation spread. The reference run uses the original 24 × 17 ChArUco board, 30 mm squares, 22 mm markers, and a 10 mm white print margin.

The Calibration page loads the generated board texture at its reconstructed base-frame pose and automatically frames the board while preserving the current camera direction and zoom. A `Focus board` action is also available after the user moves the view. The board texture is loaded under its own Suspense boundary so it does not suspend or flicker the robot scene. It is a visual calibration-target preview, not a MoveIt collision object; no planning-scene state is changed. The viewport applies `/tf_static` when the dataset base differs from the MoveIt model base. If the transform is unavailable, it hides the board and shows a frame warning rather than guessing. The reconstructed board pose is only as accurate as the saved hand-eye transform and per-image detections; it is not a surveyed ground-truth location and must not be used as a motion target.
