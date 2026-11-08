# Visualizer URL query parameters

The Scene visualizer reads supported query parameters on initial page load and keeps the address bar in sync as those inputs are edited. Use **Copy URL** to copy the current shareable link. Opening that link restores the saved targets; it does not connect to or move the robot.

## Parameters

| Parameter | Meaning | Units / format |
| --- | --- | --- |
| `tab` | Target panel selection | `tcp` or `joints` |
| `tcp` | MoveIt TCP target: X,Y,Z,RX,RY,RZ | mm, then RPY degrees |
| `joints` | Six CR5 joint targets in joint1…joint6 order | degrees |
| `direct` | Direct-movement tab | `movj` or `movl` |
| `movj` | Direct MovJ target: joint1…joint6 | degrees |
| `movl` | Direct MovL target: X,Y,Z,RX,RY,RZ | mm, then RPY degrees |

Tuple values are comma-separated finite numbers. A tuple is ignored unless it contains exactly six valid numbers. Missing or invalid values fall back to live telemetry or the page defaults.

## Examples

```text
/visualizer?tab=tcp&tcp=400,0,300,180,0,90
/visualizer?tab=joints&joints=0,-30,60,0,45,0
/visualizer?direct=movj&movj=0,-30,60,0,45,0
/visualizer?direct=movl&movl=400,0,300,180,0,90
```

## Safety and behavior

- Query parameters only populate editable UI fields. They never issue planning, path-check, gripper, or movement requests.
- Planning and direct-movement checks must still be triggered explicitly; execution remains subject to the existing bridge and robot safety gates.
- The TCP target is a MoveIt pose for the configured end-effector link in the MoveIt base frame. It is not guaranteed to match Dobot `ToolVectorActual` coordinates or controller user/tool frames.
- Joint values in the URL are degrees. The planner converts them to radians internally.
- The URL is updated with browser history replacement, so editing values does not create a history entry for every keystroke. Existing unrelated query parameters are preserved.
- Treat shared URLs as configuration data, not as authorization or a safety interlock.
