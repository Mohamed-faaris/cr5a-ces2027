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

## Current gripper limitation

The configured PGEA-100-40-W has 20 mm travel per finger (40 mm nominal total stroke). The 66 mm Coke can and 75 mm Pringles tube exceed this opening. The UI must not claim those objects are graspable with the current gripper. Use a different gripper or a separately validated grasp/contact design before allowing execution.

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
