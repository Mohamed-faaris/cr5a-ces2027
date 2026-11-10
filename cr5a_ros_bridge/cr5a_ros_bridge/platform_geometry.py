"""AprilTag-based, gravity-aligned platform collision geometry."""
from __future__ import annotations
import math
from typing import Any


def _vec(value: Any, n: int, label: str) -> list[float]:
    if not isinstance(value, (list, tuple)) or len(value) != n:
        raise ValueError(f"{label} must contain {n} numbers")
    result = [float(v) for v in value]
    if not all(math.isfinite(v) for v in result):
        raise ValueError(f"{label} contains a non-finite value")
    return result


def _unit(a: list[float], label: str) -> list[float]:
    norm = math.sqrt(sum(x*x for x in a))
    if norm < 1e-9:
        raise ValueError(f"{label} has zero length")
    return [x/norm for x in a]


def _dot(a, b): return sum(x*y for x, y in zip(a, b))
def _add(a, b): return [x+y for x, y in zip(a, b)]
def _sub(a, b): return [x-y for x, y in zip(a, b)]
def _mul(a, k): return [x*k for x in a]
def _cross(a, b): return [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]]


def _quat(value: Any) -> list[float]:
    q = _vec(value, 4, "orientation_xyzw")
    norm = math.sqrt(sum(x*x for x in q))
    if norm < 1e-9:
        raise ValueError("marker quaternion has zero length")
    return [x/norm for x in q]


def _rotate(q, v):
    x, y, z, w = q
    qv = [x, y, z]
    c1, c2 = _cross(qv, v), _cross(qv, _cross(qv, v))
    return _add(v, _add(_mul(c1, 2*w), _mul(c2, 2)))


def _parse_marker(item: dict[str, Any]) -> dict[str, Any]:
    if not isinstance(item, dict):
        raise ValueError("each marker must be an object")
    marker_id = str(item.get("id", "")).strip()
    corner = str(item.get("corner", "")).strip().lower()
    if not marker_id:
        raise ValueError("each marker requires an id")
    position = _vec(item.get("position_m"), 3, f"marker {marker_id} position_m")
    orientation = _quat(item.get("orientation_xyzw"))
    normal = _unit(_rotate(orientation, [0, 0, 1]), f"marker {marker_id} normal")
    x_axis = _rotate(orientation, [1, 0, 0])
    x_axis = _unit([x_axis[0], x_axis[1], 0], f"marker {marker_id} horizontal X axis")
    return {"id": marker_id, "corner": corner, "position_m": position,
            "orientation_xyzw": orientation, "normal": normal, "x_axis": x_axis}


def build_platform_box(mode: str, markers: list[dict[str, Any]], ground_z_m: float, *,
                       width_m: float | None = None, depth_m: float | None = None,
                       margin_m: float = 0.0, max_marker_tilt_deg: float = 15.0,
                       max_height_spread_m: float = 0.035,
                       max_corner_fit_rms_m: float = 0.05) -> dict[str, Any]:
    """Return a solid table-volume OBB from the marker plane down to ground_z_m.

    All AprilTags must face base-frame +Z. Four-corner labels are front_left,
    front_right, back_right, back_left. The two-corner mode uses the common tag
    local-X direction as platform width axis. Center mode requires manual sizes.
    """
    if mode not in {"four_corners", "two_corners", "center_dimensions"}:
        raise ValueError("mode must be four_corners, two_corners or center_dimensions")
    required = {"four_corners": 4, "two_corners": 2, "center_dimensions": 1}[mode]
    if not isinstance(markers, list) or len(markers) != required:
        raise ValueError(f"{mode} requires exactly {required} marker(s)")
    parsed = [_parse_marker(item) for item in markers]
    if len({item["id"] for item in parsed}) != required:
        raise ValueError("marker IDs must be unique")

    ground, margin, max_tilt = float(ground_z_m), float(margin_m), float(max_marker_tilt_deg)
    if not all(math.isfinite(v) for v in (ground, margin, max_tilt, max_height_spread_m, max_corner_fit_rms_m)):
        raise ValueError("ground, margin and quality tolerances must be finite")
    if margin < 0 or not 0 < max_tilt < 45 or max_height_spread_m <= 0 or max_corner_fit_rms_m <= 0:
        raise ValueError("invalid margin or marker quality tolerance")
    vertical = [0.0, 0.0, 1.0]
    tilts = [math.degrees(math.acos(max(-1.0, min(1.0, _dot(m["normal"], vertical))))) for m in parsed]
    worst_tilt = max(tilts)
    if worst_tilt > max_tilt:
        raise ValueError(f"AprilTag(s) must face up: maximum marker-normal tilt is {worst_tilt:.1f}° (limit {max_tilt:.1f}°)")
    positions = [m["position_m"] for m in parsed]
    heights = [p[2] for p in positions]
    height_spread = max(heights)-min(heights)
    if height_spread > max_height_spread_m:
        raise ValueError(f"Marker heights differ by {height_spread*1000:.1f} mm; limit is {max_height_spread_m*1000:.1f} mm")
    top_z = sum(heights)/len(heights)
    if top_z <= ground+0.05:
        raise ValueError("platform top must be at least 50 mm above the ground plane")
    corner_fit_rms = 0.0

    if mode == "four_corners":
        by_corner = {m["corner"]: m for m in parsed}
        names = ("front_left", "front_right", "back_right", "back_left")
        if set(by_corner) != set(names):
            raise ValueError("four-corner mode requires valid corner assignments: front_left, front_right, back_right and back_left")
        fl, fr, br, bl = (by_corner[n]["position_m"] for n in names)
        u = _unit(_add(_sub(fr, fl), _sub(br, bl)), "platform width axis")
        u = _unit([u[0], u[1], 0], "platform width axis")
        v_raw = _mul(_add(_sub(bl, fl), _sub(br, fr)), 0.5)
        v_raw = _sub(v_raw, _mul(u, _dot(v_raw, u)))
        v = _unit([v_raw[0], v_raw[1], 0], "platform depth axis")
        if _cross(u, v)[2] < 0: v = _mul(v, -1)
        origin = [sum(p[i] for p in positions)/4 for i in range(3)]
        pu, pv = [_dot(_sub(p, origin), u) for p in positions], [_dot(_sub(p, origin), v) for p in positions]
        min_u, max_u, min_v, max_v = min(pu), max(pu), min(pv), max(pv)
        width, depth = max_u-min_u, max_v-min_v
        center = _add(origin, _add(_mul(u, (min_u+max_u)/2), _mul(v, (min_v+max_v)/2)))
        hu, hv = width/2, depth/2
        expected = {
            "front_left": _add(center, _add(_mul(u, -hu), _mul(v, -hv))),
            "front_right": _add(center, _add(_mul(u, hu), _mul(v, -hv))),
            "back_right": _add(center, _add(_mul(u, hu), _mul(v, hv))),
            "back_left": _add(center, _add(_mul(u, -hu), _mul(v, hv))),
        }
        errors = [math.dist(by_corner[n]["position_m"], expected[n]) for n in names]
        corner_fit_rms = math.sqrt(sum(e*e for e in errors)/4)
        if corner_fit_rms > max_corner_fit_rms_m:
            raise ValueError(f"Marker corners do not form a rectangle (fit RMS {corner_fit_rms*1000:.1f} mm; limit {max_corner_fit_rms_m*1000:.1f} mm)")
        confidence = "four-point fitted rectangle"
    elif mode == "two_corners":
        first, second = parsed
        if _dot(first["x_axis"], second["x_axis"]) < math.cos(math.radians(15)):
            raise ValueError("The two corner tags must share the same horizontal orientation (within 15°)")
        u = first["x_axis"]
        v = _unit(_cross(vertical, u), "platform depth axis")
        diagonal = _sub(second["position_m"], first["position_m"])
        width, depth = abs(_dot(diagonal, u)), abs(_dot(diagonal, v))
        if width < 0.05 or depth < 0.05:
            raise ValueError("Two diagonal markers must span at least 50 mm in both platform axes")
        center = _mul(_add(first["position_m"], second["position_m"]), 0.5)
        confidence = "diagonal pair; dimensions inferred from shared tag X/Y axes"
    else:
        marker = parsed[0]
        width = float(width_m if width_m is not None else 0.0)
        depth = float(depth_m if depth_m is not None else 0.0)
        if not math.isfinite(width) or not math.isfinite(depth) or width < 0.05 or depth < 0.05:
            raise ValueError("center-marker mode requires width_m and depth_m of at least 50 mm")
        u = marker["x_axis"]
        v = _unit(_cross(vertical, u), "platform depth axis")
        center = list(marker["position_m"])
        confidence = "single center tag; dimensions supplied manually"

    width += 2*margin
    depth += 2*margin
    thickness = top_z-ground
    yaw = math.atan2(u[1], u[0])
    return {
        "id": "platform_builder_platform", "frame_id": "", "shape": "box",
        "position_m": [center[0], center[1], (top_z+ground)/2],
        "dimensions_m": [width, depth, thickness],
        "orientation_xyzw": [0.0, 0.0, math.sin(yaw/2), math.cos(yaw/2)],
        "top_z_m": top_z, "ground_z_m": ground,
        "width_axis_xy": u[:2], "depth_axis_xy": v[:2],
        "quality": {
            "plane_rms_m": math.sqrt(sum((z-top_z)**2 for z in heights)/len(heights)),
            "height_spread_m": height_spread, "corner_fit_rms_m": corner_fit_rms,
            "max_marker_tilt_deg": worst_tilt, "marker_count": required,
            "source_confidence": confidence,
        },
        "warnings": [
            "This is a solid table-volume collision box extending from the marker plane down to the configured ground plane.",
            "Two-corner and center-marker modes infer geometry from tag orientation/manual dimensions; verify the physical result in the 3D preview.",
        ],
    }


def compose_marker_pose(transform_translation_m: list[float], transform_quaternion_xyzw: list[float],
                        marker_position_m: list[float], marker_orientation_xyzw: list[float]) -> dict[str, list[float]]:
    """Compose T_base_source with T_source_marker (ROS XYZW convention)."""
    t = _vec(transform_translation_m, 3, "TF translation")
    tq = _quat(transform_quaternion_xyzw)
    p = _vec(marker_position_m, 3, "marker position")
    q = _quat(marker_orientation_xyzw)
    x, y, z, w = tq
    vx, vy, vz = p
    qv = [x, y, z]
    rotated = _add(p, _add(_mul(_cross(qv, p), 2*w), _mul(_cross(qv, _cross(qv, p)), 2)))
    a, b, c, d = tq
    e, f, g, h = q
    composed_q = _quat([
        a*h + b*g - c*f + d*e,
        -a*g + b*h + c*e + d*f,
        a*f - b*e + c*h + d*g,
        -a*e - b*f - c*g + d*h,
    ])
    return {"position_m": _add(t, rotated), "orientation_xyzw": composed_q}
