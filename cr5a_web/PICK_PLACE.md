# Pick & place planner

The `/pick-place` route is a guided, four-stage task workspace. It accepts synthetic presets, manual objects and a JSON vision-detection adapter. Inputs are normalized to metres, radians and kilograms before bridge integration. The initial UI preview is a schematic side elevation, not a robot collision result.

## Guided workflow

1. **Object** — select a Coke can, Pringles tube, wooden box or custom object; for camera input, load a detection JSON and ensure its frame matches the MoveIt base frame.
2. **Pick and place scene** — choose the shelf/pick bay and destination, then load the three baseline obstacles (`ground_plane`, `base_pedestal`, `Box_0`). The `Load 3 default obstacles` action is safe to repeat and restores any missing obstacle after startup or after clearing the scene. `Prepare scene & continue` refreshes the shelf, platform and object collision geometry in MoveIt.
3. **Grasp and validation** — tune the centroid/TCP target, yaw, approach and gripper strategy. Resolve all validation checks before proceeding.
4. **Plan** — request the staged MoveIt pick/place plan and inspect the status of each returned waypoint. Changes to scene inputs invalidate the prepared-scene stage; changes to grasp or payload inputs invalidate the previous plan.

Planning is not execution. The current pick/place page generates and displays candidate staged trajectories; it does not send them to the robot or run a physical pick/place action. Validate the complete task in Gazebo and establish a separate guarded execution workflow before commanding hardware.

The 3D viewport keeps the last parsed robot model visible during scene refreshes. Joint states are sampled by the render loop rather than causing the whole 3D React tree to reconcile for every incoming joint message.

## Editable configuration tables

The guided stages use the same values in their configuration tables, previews and bridge requests:

- **Object**: preset or custom geometry, X/Y/Z dimensions, collision shape and mass. Editing a preset promotes it to a custom object so edits are preserved. Wooden-box mass is recalculated from volume and density.
- **Pick/place scene**: shelf outside width/height/depth, number of bays, board thickness, shelf pose, pick and destination bays, clearance, and platform position and collision dimensions. These values are sent as metres and radians to `pick_place.prepare` and `pick_place.plan`.
- **Grasp/TCP**: object centroid and yaw, gripper yaw, approach tilt/distance, finger clearance and a full XYZ TCP offset. TCP offset is rotated by the grasp orientation before the target is generated.
- **MoveIt plan**: OMPL planner ID, attempts, allowed time, velocity and acceleration scaling, position and orientation tolerance, and lift distance. The UI converts millimetres/degrees to metres/radians before sending the request; attempts are forwarded to each waypoint planning request.
- **Advanced controller payload**: tool mass and tool COM are editable, with the combined tool/object COM shown as a derived value. `SetPayload` is a separate explicit action and requires hardware Tool0 and mass confirmation.

A shelf fit constraint is applied only when the object is picked from a shelf or placed into one. A free-space-to-platform task still creates the shelf collision geometry, but an unrelated oversized object is not rejected for failing to fit a shelf bay. Manually editing the object centroid selects a free-space pick, and changing scene geometry invalidates the previous prepared scene until it is refreshed.

The viewport is shorter on the pick-and-place page so the stage settings need less scrolling. If the browser cannot create WebGL, a local fallback message keeps the task configuration usable; it does not simulate or replace the robot model.

## Grasp variables

- **Centroid**: nominal target for the configured MoveIt end-effector link / TCP, expressed in the selected MoveIt base frame.
- **Gripper yaw**: relative rotation around the object Z axis. The absolute base-frame yaw is object yaw plus this relative angle. Candidate generation samples yaw around symmetric objects and uses detected object orientation for boxes.
- **Approach angle**: tilt of the approach vector, separate from yaw.
- **Approach distance**: pre-grasp offset along the approach vector.
- **TCP offset**: calibrated translation from the nominal object centroid to the actual TCP. Keep zero only when the selected link frame truly coincides with the grasp point.
- **Object geometry / pose**: dimensions, shape, centroid, orientation and reference frame from vision or synthetic scene generation.
- **Clearance / lift height**: shelf, obstacle and place margins; these are planning inputs, not safety-rated distances.
- **Payload mass**: used by a payload-aware controller/dynamics layer. MoveIt collision geometry does not calculate torque, gravity compensation or controller payload limits.

## Initial payload assumptions

| Object | Geometry used | Mass used | Basis |
| --- | --- | ---: | --- |
| Coca-Cola 330 ml can | 66 mm diameter × 122 mm | 0.355 kg | Approximate full 330 ml can; typical full 330 ml beverage can estimate. [Source](https://playhowheavy.com/objects/full-soda-can) |
| Pringles 165 g tube | 75 mm diameter × 240 mm | 0.215 kg | 165 g contents + approximately 50 g empty tube/lid. Empty 200 g-size tube measured at 50 g; size varies. [Source](https://weighschool.com/how-much-do-pringles-weigh/) and [Science Museum package dimensions](https://collection.sciencemuseumgroup.org.uk/objects/co8784028/packet-for-pringles-original-potato-crisps) |
| Wooden box | User-defined L × W × H | Volume × 650 kg/m³ | Adjustable engineering estimate. Wood varies widely: pine is around 400 kg/m³ and oak around 800 kg/m³. [Source](https://www.bioenergyprof.eu/handbooks/bisyplan/html-files-en/01-01.pdf) |

These are estimates, not measured product masses. Replace with scale measurements before configuring a real robot payload.

## PGEA extended-finger grasp model

The nominal PGEA-100-40-W mechanism stroke is 40 mm total. That is not the full contact envelope of the custom fingers in `dobot_rviz/urdf/cr5_pgea_moveit.urdf`: each finger mesh has a 45 mm lateral CAD span. The planner therefore exposes two distinct strategies:

- `parallel_jaw`: 40 mm nominal total stroke, minus the selected clearance.
- `pgea_finger_wrap`: preliminary 85 mm geometry envelope (40 mm stroke + 45 mm custom finger span), with 6 mm default object clearance.

The 66 mm Coke can and 75 mm Pringles tube both pass the preliminary `pgea_finger_wrap` envelope check. This matches the operator's reported ability to grasp both with the modified fingers. The 85 mm envelope is a CAD-derived planning allowance, not a DH-Robotics rated grasp width. MoveIt checks robot/environment path collisions, but the grasp-stage object/finger contact pair is intentionally permitted; it does not prove normal force, friction, retention or resistance to slipping. Gazebo contact simulation and guarded physical validation are still required before motion execution.

## Vision adapter shape

```json
{
  "frame_id": "dummy_link",
  "objects": [{
    "id": "detected_1",
    "class": "can",
    "shape": "cylinder",
    "centroid_m": [0.30, 0.00, 0.10],
    "dimensions_m": [0.066, 0.066, 0.122],
    "yaw_deg": 0,
    "mass_kg": 0.355
  }]
}
```

A detection's frame must be transformed into `model.base_frame` before using it as a MoveIt target. Never treat a camera optical-frame centroid as a base-frame target without TF transformation.

## Controller payload integration

The task planner combines the object mass with the tool assembly mass and COM, then applies a bounded timing derating to each MoveIt waypoint's velocity/acceleration scaling. This is a conservative timing heuristic, not a dynamics or torque model. The Dobot controller's `SetPayload(load,x,y,z)` command is separate: `load` is in kg and the eccentric COM offsets are in mm in the controller's default tool frame. The UI requires explicit operator confirmation before calling `set-payload`; the bridge keeps robot commands disabled by default and rejects loads above the configured CR5 5 kg limit. [Dobot SetPayload reference](https://github.com/Dobot-Edu/Dobot_CRtrainer/blob/main/scripts/dobot_api.py) and [Dobot load parameter guidance](https://docs.trossenrobotics.com/dobot_cr_cobots_docs/dobotstudiopro/settings/load_parameters.html).

The selected clean CR5A/PGEA URDF models a 1.30 kg tool assembly (0.40 kg connector slabs + 0.60 kg PGEA body + 0.15 kg per finger); it intentionally omits the former 0.30 kg extension-plate link. Its inertial origins give an estimated tool COM of approximately `[-0.08608, 0, -0.09262] m` relative to the new `tcp` frame. DH-Robotics publishes 0.60 kg for the PGEA-100-40 assembly, so the URDF's finger/connector masses remain engineering estimates. The UI exposes the tool-mass override and requires the operator to verify the mounted assembly and controller Tool0 frame before applying a payload. [PGEA specifications](https://www.dobot-robots.com/ecosystem/accessories/dh-robotics-pgea-servo-gripper.html).

The `tcp` link is the intended grasp target frame in the supplied PGEA URDF; it is attached directly to `Link6` at `(0, -0.117, 0.180) m` with a `-90°` Z rotation. The MoveIt wrapper preserves `dummy_link` as the configured base frame. Both cylindrical presets default to `pgea_finger_wrap`; boxes and custom objects default to `parallel_jaw`. The tool COM preset is derived from the supplied URDF inertial origins in the `tcp` frame; the user must verify the robot's active Tool0 against that frame before applying the physical `SetPayload` command. The URDF masses remain engineering estimates because the gripper manufacturer's 0.60 kg product mass may already include the fingers.
