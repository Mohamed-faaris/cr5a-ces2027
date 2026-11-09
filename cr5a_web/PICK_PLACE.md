# Pick & place planner

The `/pick-place` route is a reusable task workspace. It accepts synthetic presets, manual objects and a JSON vision-detection adapter. Inputs are normalized to metres, radians and kilograms before bridge integration. The initial UI preview is a schematic side elevation, not a robot collision result.

## Grasp variables

- **Centroid**: nominal target for the configured MoveIt end-effector link / TCP, expressed in the selected MoveIt base frame.
- **Gripper yaw**: rotation around the world Z axis. A future candidate generator should sample yaw around symmetric objects and use detected object orientation for boxes.
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

The current PGEA URDF sums to 1.60 kg for the tool assembly (0.30 kg extension plate + 0.40 kg connector + 0.60 kg gripper body + 0.15 kg per finger). DH-Robotics publishes 0.60 kg for the PGEA-100-40 assembly, so the URDF may double-count finger mass. The UI exposes the tool-mass override and requires the operator to verify the mounted assembly and controller Tool0 frame before applying a payload. The initial COM estimate is derived from the URDF assuming controller Tool0 matches the `tcp` link; it must be checked against the actual controller tool definition. [PGEA specifications](https://www.dobot-robots.com/ecosystem/accessories/dh-robotics-pgea-servo-gripper.html).

The `tcp` link is the intended grasp target frame in the current PGEA URDF. Both cylindrical presets default to `pgea_finger_wrap`; boxes and custom objects default to `parallel_jaw`. The tool COM preset is transformed from current URDF inertial origins into the `tcp` frame, but the user must verify the robot's active Tool0 against that frame before applying the physical `SetPayload` command. The URDF mass sum (1.60 kg) remains an editable estimate because the gripper manufacturer's 0.60 kg product mass may already include the fingers.
