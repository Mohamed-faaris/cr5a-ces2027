# AprilTag Platform Builder

The Platform Builder is a separate scene-tool route (`/platform-builder`). It does not command robot motion. It estimates marker poses, creates a solid collision object, and sends that object to MoveIt's planning scene only after operator confirmation.

## Camera and TF chain

The AprilTag detector subscribes to the RGB image and matching `CameraInfo` topics. `tag_size_m` is the physical black/white marker outer square size in metres (measure the printed tag, not the sheet). Pose estimation uses the camera matrix `K` and distortion coefficients `D`; no focal length or depth is guessed. Detected tags are broadcast as dynamic TF frames named `tag_<id>` under the image's optical frame.

For a wrist-mounted camera, the required chain is:

```text
MoveIt base frame -> robot link carrying the camera -> camera optical frame -> tag_<id>
```

The parent-to-camera edge must come from the robot URDF or the imported, validated hand-eye calibration. The detector only estimates camera-to-tag. The bridge resolves `T_base_tag` using TF before platform geometry is calculated. Missing or stale TF is an error; no camera transform is fabricated.

Default topics:

- RGB: `/cr5/wrist_camera/color/image_raw`
- CameraInfo: `/cr5/wrist_camera/color/camera_info`
- Status: `/cr5/apriltag/status`
- Detections: `/cr5/apriltag/detections`
- Annotated image: `/cr5/apriltag/annotated/compressed`

The detector uses the ROS Python OpenCV ArUco module and supports `tag36h11`, `tag25h9`, and `tag16h5`. The node starts in stopped state; apply family, marker size and comma-separated marker IDs in the Platform Builder, then start the detector. The node publishes no valid metric poses until `CameraInfo` and RGB frames arrive.

## Construction modes

All tag normals must be within 15 degrees of MoveIt base +Z; marker heights must agree within 35 mm. Geometry is rejected when those assumptions are violated.

1. **Four corners:** assign each ID to `front_left`, `front_right`, `back_right`, and `back_left`. The four measured points fit a horizontal rectangle; the corner-fit RMS is reported.
2. **Two opposite corners:** place two diagonal tags. Both tag local-X axes must agree within 15 degrees; that shared axis defines platform width and its perpendicular defines depth. The two markers do not independently determine the platform's rectangle orientation, so align them identically.
3. **Center + dimensions:** one center tag and manually supplied width/depth. The tag's horizontal local-X axis defines yaw.

The configured ground height is the **top surface** of the existing ground collision object. The generated solid box fills from the marker plane down to this ground height, so MoveIt cannot plan through the platform volume. Confirm this height and the box preview against the physical cell before loading. The default bridge ground plane top is approximately `-0.7429 m` in the configured base frame; verify the installed scene and robot model before using this value.

## Captures and reuse

Profiles store mode, tag family/size, IDs, TF frame names, dimensions and ground settings in browser local storage. Export profiles as JSON to move them to another browser/workstation. Capture snapshots store the resolved base-frame marker poses and fit metrics; imported snapshots are for review/preview, not automatic revalidation. Re-resolve live TF after loading a profile and before adding collisions.

Capture comparison ranks plane RMS plus corner-fit RMS. This is a geometric consistency score, not an estimate of absolute camera extrinsic accuracy. Multiple viewpoints/photos are not fused into a photogrammetry or bundle-adjustment solution in this version. An externally recalibrated TF/hand-eye result must be applied before captures are compared.

## Safety boundary

The builder only adds/removes MoveIt world collision objects. It does not execute trajectories or move the arm. Verify the tag family, print size, TF frame convention, camera intrinsics, hand-eye calibration, base frame, floor height and dimensions before pressing **Load collision box into MoveIt**.
