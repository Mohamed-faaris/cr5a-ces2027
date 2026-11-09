"""Reusable pick/place scene geometry, payload estimates and grasp candidates.

All public dimensions and poses use SI units (metres, radians, kilograms).
The module is ROS-independent so geometry and feasibility can be tested on CI.
"""
from __future__ import annotations

import math
import re
from typing import Any


OBJECT_PRESETS = {
    "coca_cola_330ml": {
        "label": "Coca-Cola can 330 ml", "shape": "cylinder",
        "dimensions_m": [0.066, 0.066, 0.122], "mass_kg": 0.355,
        "mass_source": "Estimate: 330 ml beverage plus approximately 13 g aluminium can",
    },
    "pringles_165g": {
        "label": "Pringles tube 165 g", "shape": "cylinder",
        "dimensions_m": [0.075, 0.075, 0.240], "mass_kg": 0.215,
        "mass_source": "Estimate: 165 g contents plus approximately 50 g tube and lid",
    },
    "wood_box": {
        "label": "Pine box", "shape": "box",
        "dimensions_m": [0.080, 0.030, 0.120],
        "mass_source": "Solid volume × configurable pine density",
    },
}

SHELF_PRESETS = {
    "compact": {"width_m": 0.420, "height_m": 0.620, "depth_m": 0.300, "levels": 2, "board_m": 0.018},
    "standard": {"width_m": 0.600, "height_m": 0.900, "depth_m": 0.400, "levels": 3, "board_m": 0.018},
    "wide": {"width_m": 0.900, "height_m": 1.200, "depth_m": 0.450, "levels": 4, "board_m": 0.020},
    "deep": {"width_m": 0.700, "height_m": 1.000, "depth_m": 0.600, "levels": 3, "board_m": 0.022},
}


def _finite(value: Any, name: str, *, minimum: float | None = None) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"{name} must be a finite number")
    result = float(value)
    if minimum is not None and result < minimum:
        raise ValueError(f"{name} must be at least {minimum}")
    return result


def _vector(value: Any, length: int, name: str, *, minimum: float | None = None) -> list[float]:
    if not isinstance(value, (list, tuple)) or len(value) != length:
        raise ValueError(f"{name} must contain exactly {length} values")
    return [_finite(item, f"{name}[{index}]", minimum=minimum) for index, item in enumerate(value)]


def estimate_payload_mass(kind: str, dimensions_m: list[float] | None = None,
                          density_kg_m3: float = 650.0, supplied_mass_kg: float | None = None) -> dict:
    """Return a traceable payload estimate; supplied mass takes precedence."""
    if supplied_mass_kg is not None:
        mass = _finite(supplied_mass_kg, "mass_kg", minimum=0.001)
        return {"mass_kg": mass, "source": "supplied measurement / vision metadata", "estimated": False}
    if kind in OBJECT_PRESETS and "mass_kg" in OBJECT_PRESETS[kind]:
        item = OBJECT_PRESETS[kind]
        return {"mass_kg": item["mass_kg"], "source": item["mass_source"], "estimated": True}
    if dimensions_m is None:
        raise ValueError("dimensions_m is required to estimate a custom object's mass")
    x, y, z = _vector(dimensions_m, 3, "dimensions_m", minimum=0.001)
    density = _finite(density_kg_m3, "density_kg_m3", minimum=100)
    if density > 1400:
        raise ValueError("density_kg_m3 must not exceed 1400")
    return {
        "mass_kg": x * y * z * density,
        "source": f"solid volume × wood density ({density:g} kg/m³)",
        "estimated": True,
    }



def calculate_payload_model(object_mass_kg: float, *, tool_mass_kg: float = 0.0,
                            object_com_m: list[float] | None = None,
                            tool_com_m: list[float] | None = None,
                            base_velocity_scaling: float = 0.15,
                            base_acceleration_scaling: float = 0.15,
                            max_payload_kg: float = 5.0) -> dict:
    """Estimate combined load and apply a conservative mass-based planning heuristic.

    MoveIt geometric planners do not model payload dynamics. The returned
    velocity/acceleration factors only slow trajectory timing; they are not a
    torque, stability, or controller payload-limit proof.
    """
    object_mass = _finite(object_mass_kg, "object_mass_kg", minimum=0.001)
    tool_mass = _finite(tool_mass_kg, "tool_mass_kg", minimum=0.0)
    object_com = _vector(object_com_m or [0.0, 0.0, 0.0], 3, "object_com_m")
    tool_com = _vector(tool_com_m or [0.0, 0.0, 0.0], 3, "tool_com_m")
    velocity = _finite(base_velocity_scaling, "base_velocity_scaling", minimum=0.01)
    acceleration = _finite(base_acceleration_scaling, "base_acceleration_scaling", minimum=0.01)
    maximum = _finite(max_payload_kg, "max_payload_kg", minimum=0.1)
    if velocity > 1.0 or acceleration > 1.0:
        raise ValueError("base velocity and acceleration scaling must not exceed 1")
    total_mass = object_mass + tool_mass
    center = [(object_mass * object_com[i] + tool_mass * tool_com[i]) / total_mass
              for i in range(3)]
    weight = total_mass * 9.80665
    moment = weight * math.hypot(center[0], center[1])
    if total_mass > maximum:
        raise ValueError(f"combined payload {total_mass:.3f} kg exceeds configured limit {maximum:.3f} kg")
    # A bounded derating against the CR5 nominal payload rating. This changes
    # MoveIt trajectory timing only; it is not a dynamic/torque model.
    remaining_ratio = max(0.10, 1.0 - total_mass / maximum)
    velocity_factor = math.sqrt(remaining_ratio)
    acceleration_factor = remaining_ratio
    return {
        "object_mass_kg": object_mass, "tool_mass_kg": tool_mass,
        "total_mass_kg": total_mass, "center_of_mass_m": center,
        "static_weight_n": weight, "gravity_moment_nm": moment,
        "payload_scale_factor": velocity_factor,
        "max_velocity_scaling": min(velocity, velocity * velocity_factor),
        "max_acceleration_scaling": min(acceleration, acceleration * acceleration_factor),
        "max_payload_kg": maximum,
        "heuristic": "CR5 rated-payload timing derating; not dynamics validation",
    }


def evaluate_gripper_fit(dimensions_m: list[float], *, strategy: str = "parallel_jaw",
                         gripper_yaw_deg: float = 0.0, clearance_m: float = 0.006,
                         nominal_stroke_m: float = 0.040, finger_mesh_span_m: float = 0.045,
                         wrap_envelope_m: float = 0.085) -> dict:
    """Estimate whether the selected PGEA contact strategy spans an object.

    The wrap envelope is a preliminary CAD-derived estimate (40 mm mechanism
    stroke + 45 mm finger-mesh span). Yaw is relative to the object's local
    orientation. This is not a published grasp rating and does not prove
    friction, contact force or object retention.
    """
    dx, dy, _ = _vector(dimensions_m, 3, "dimensions_m", minimum=0.001)
    yaw = math.radians(_finite(gripper_yaw_deg, "gripper_yaw_deg"))
    clearance = _finite(clearance_m, "clearance_m", minimum=0.0)
    stroke = _finite(nominal_stroke_m, "nominal_stroke_m", minimum=0.001)
    finger_span = _finite(finger_mesh_span_m, "finger_mesh_span_m", minimum=0.0)
    wrap_span = _finite(wrap_envelope_m, "wrap_envelope_m", minimum=stroke)
    if strategy == "pgea_finger_wrap":
        envelope = wrap_span
    elif strategy == "parallel_jaw":
        envelope = stroke
    else:
        raise ValueError("grasp_strategy must be pgea_finger_wrap or parallel_jaw")
    projected_width = abs(math.cos(yaw)) * dx + abs(math.sin(yaw)) * dy
    required = projected_width + clearance
    return {
        "strategy": strategy, "projected_object_width_m": projected_width,
        "clearance_m": clearance, "required_span_m": required,
        "nominal_total_stroke_m": stroke, "finger_mesh_span_m": finger_span,
        "estimated_wrap_envelope_m": wrap_span, "usable_envelope_m": envelope,
        "valid": required <= envelope,
        "basis": "CAD-derived estimate; physical contact/retention still requires validation",
    }

def normalize_object(payload: dict) -> dict:
    """Normalize synthetic, UI, or vision object payloads to one metric schema."""
    if not isinstance(payload, dict):
        raise ValueError("object must be a JSON object")
    kind = str(payload.get("kind", payload.get("class", "custom")))
    preset = OBJECT_PRESETS.get(kind)
    shape = str(payload.get("shape", preset["shape"] if preset else "box")).lower()
    if shape not in ("box", "cylinder", "sphere"):
        raise ValueError("shape must be box, cylinder, or sphere")
    dims = payload.get("dimensions_m", preset["dimensions_m"] if preset else None)
    if shape == "sphere":
        if payload.get("radius_m") is not None:
            radius = _finite(payload.get("radius_m"), "radius_m", minimum=0.001)
        else:
            sphere_dims = _vector(dims, 3, "dimensions_m", minimum=0.001)
            if max(sphere_dims) - min(sphere_dims) > 1e-6:
                raise ValueError("sphere dimensions_m must be equal on all axes")
            radius = sphere_dims[0] / 2
        dimensions = [radius * 2, radius * 2, radius * 2]
    else:
        dimensions = _vector(dims, 3, "dimensions_m", minimum=0.001)
    centroid = _vector(payload.get("centroid_m", payload.get("position_m", [0.3, 0.0, 0.1])), 3, "centroid_m")
    rpy = _vector(payload.get("rpy_rad", [0.0, 0.0, math.radians(float(payload.get("yaw_deg", 0.0)))]), 3, "rpy_rad")
    mass = estimate_payload_mass(kind, dimensions, payload.get("density_kg_m3", 650.0), payload.get("mass_kg"))
    object_id = str(payload.get("id", "pick_object"))
    if not re.fullmatch(r"[A-Za-z0-9_.-]{1,64}", object_id):
        raise ValueError("object id must be 1-64 letters, digits, dot, underscore or hyphen")
    frame_id = str(payload.get("frame_id", "dummy_link"))
    if not frame_id or len(frame_id) > 128:
        raise ValueError("frame_id must contain 1-128 characters")
    return {
        "id": object_id, "label": str(payload.get("label", preset["label"] if preset else object_id)),
        "kind": kind, "shape": shape, "dimensions_m": dimensions,
        "centroid_m": centroid, "rpy_rad": rpy, "frame_id": frame_id, **mass,
    }


def quaternion_from_rpy(roll: float, pitch: float, yaw: float) -> list[float]:
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return [sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy]


def _box(object_id: str, position: list[float], dimensions: list[float], frame_id: str) -> dict:
    return {"id": object_id, "shape": "box", "frame_id": frame_id,
            "position_m": position, "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
            "dimensions_m": dimensions}


def generate_shelf_scene(shelf: dict | str = "standard", *, frame_id: str = "dummy_link",
                         level: int = 0, clearance_m: float = 0.012,
                         object_dimensions_m: list[float] | None = None,
                         platform: dict | None = None,
                         origin_m: list[float] | None = None,
                         yaw_rad: float = 0.0,
                         object_yaw_rad: float = 0.0) -> dict:
    """Build an open-front rack as independent collision boxes and a safe bay pose."""
    spec = dict(SHELF_PRESETS[shelf]) if isinstance(shelf, str) and shelf in SHELF_PRESETS else dict(shelf) if isinstance(shelf, dict) else None
    if spec is None:
        raise ValueError("Unknown shelf profile")
    width = _finite(spec.get("width_m"), "shelf.width_m", minimum=0.1)
    height = _finite(spec.get("height_m"), "shelf.height_m", minimum=0.1)
    depth = _finite(spec.get("depth_m"), "shelf.depth_m", minimum=0.1)
    board = _finite(spec.get("board_m", 0.018), "shelf.board_m", minimum=0.005)
    levels = spec.get("levels", 3)
    if isinstance(levels, bool) or not isinstance(levels, int) or not 1 <= levels <= 8:
        raise ValueError("shelf.levels must be an integer from 1 to 8")
    if board * (levels + 1) >= height:
        raise ValueError("Shelf boards leave no usable bay height")
    if not 0 <= level < levels:
        raise ValueError(f"level must be between 0 and {levels - 1}")
    bay_height = (height - board * (levels + 1)) / levels
    bay_width, bay_depth = width - 2 * board, depth - board
    origin = _vector(origin_m or [0.55, 0.0, 0.0], 3, "shelf.origin_m")
    shelf_yaw = _finite(yaw_rad, "shelf.yaw_rad")
    shelf_q = quaternion_from_rpy(0.0, 0.0, shelf_yaw)
    objects = [
        _box("shelf_left", [-width/2 + board/2, 0.0, height/2], [board, depth, height], frame_id),
        _box("shelf_right", [width/2 - board/2, 0.0, height/2], [board, depth, height], frame_id),
        _box("shelf_back", [0.0, -depth/2 + board/2, height/2], [width-2*board, board, height], frame_id),
    ]
    for index in range(levels + 1):
        z = board/2 + index * (bay_height + board)
        objects.append(_box(f"shelf_board_{index}", [0.0, 0.0, z], [width, depth, board], frame_id))
    bay_floor = board + level * (bay_height + board)
    target_local = [0.0, 0.0, bay_floor + bay_height/2]
    fit = True
    fit_reasons = []
    if object_dimensions_m is not None:
        dx, dy, dz = _vector(object_dimensions_m, 3, "object_dimensions_m", minimum=0.001)
        relative_yaw = _finite(object_yaw_rad, "object_yaw_rad") - shelf_yaw
        projected_width = abs(math.cos(relative_yaw))*dx + abs(math.sin(relative_yaw))*dy
        projected_depth = abs(math.sin(relative_yaw))*dx + abs(math.cos(relative_yaw))*dy
        if projected_width + 2*clearance_m > bay_width:
            fit, fit_reasons = False, fit_reasons + ["object width exceeds shelf bay with clearance at selected yaw"]
        if projected_depth + 2*clearance_m > bay_depth:
            fit, fit_reasons = False, fit_reasons + ["object depth exceeds shelf bay with clearance at selected yaw"]
        if dz + 2*clearance_m > bay_height:
            fit, fit_reasons = False, fit_reasons + ["object height exceeds shelf bay with clearance"]
        target_local[2] = bay_floor + clearance_m + dz/2
    for item in objects:
        rotated = rotate_vector(shelf_q, item["position_m"])
        item["position_m"] = [origin[i] + rotated[i] for i in range(3)]
        item["orientation_xyzw"] = quaternion_multiply(shelf_q, item["orientation_xyzw"])
    target_rotated = rotate_vector(shelf_q, target_local)
    target = [origin[i] + target_rotated[i] for i in range(3)]
    if platform:
        px, py, pz = _vector(platform.get("position_m", [0.45, 0.0, 0.0]), 3, "platform.position_m")
        pd = _vector(platform.get("dimensions_m", [0.30, 0.24, 0.02]), 3, "platform.dimensions_m", minimum=0.001)
        objects.append(_box(str(platform.get("id", "place_platform")), [px, py, pz + pd[2]/2], pd, frame_id))
        place = [px, py, pz + pd[2]]
    else:
        place = None
    return {
        "frame_id": frame_id, "shelf": {"width_m": width, "height_m": height, "depth_m": depth,
            "board_m": board, "levels": levels, "selected_level": level,
            "origin_m": origin, "yaw_rad": shelf_yaw,
            "bay_dimensions_m": [bay_width, bay_depth, bay_height]},
        "collision_objects": objects, "pick_slot_center_m": target,
        "place_surface_center_m": place,
        "fit": {"valid": fit, "reasons": fit_reasons},
    }


def generate_grasp_candidates(obj: dict, *, gripper_yaw_deg: float | None = None,
                              approach_angle_deg: float = 0.0,
                              approach_distance_m: float = 0.08,
                              tcp_offset_m: list[float] | None = None,
                              yaw_samples_deg: list[float] | None = None) -> list[dict]:
    """Use object centroid as the nominal TCP target; vary yaw/approach explicitly."""
    approach = math.radians(_finite(approach_angle_deg, "approach_angle_deg"))
    distance = _finite(approach_distance_m, "approach_distance_m", minimum=0.01)
    offset = _vector(tcp_offset_m or [0.0, 0.0, 0.0], 3, "tcp_offset_m")
    yaw_values = [gripper_yaw_deg] if gripper_yaw_deg is not None else (yaw_samples_deg or [0, 45, 90, 135])
    centroid = obj["centroid_m"]
    candidates = []
    object_yaw = float(obj.get("rpy_rad", [0.0, 0.0, 0.0])[2])
    for yaw_deg in yaw_values:
        relative_yaw = math.radians(_finite(yaw_deg, "gripper_yaw_deg"))
        yaw = relative_yaw + object_yaw
        q = quaternion_from_rpy(0.0, approach, yaw)
        offset_world = rotate_vector(q, offset)
        tcp = [centroid[i] + offset_world[i] for i in range(3)]
        approach_vector = [math.sin(yaw) * math.cos(approach), -math.cos(yaw) * math.cos(approach), math.sin(approach)]
        pregrasp = [tcp[i] - approach_vector[i] * distance for i in range(3)]
        candidates.append({"tcp_target_m": tcp, "pregrasp_target_m": pregrasp,
            "orientation_xyzw": q, "gripper_yaw_deg": math.degrees(yaw),
            "gripper_yaw_relative_deg": float(yaw_deg),
            "approach_angle_deg": float(approach_angle_deg), "approach_distance_m": distance})
    return candidates


def build_task_waypoints(obj: dict, grasp: dict, place_surface_center_m: list[float],
                         *, lift_distance_m: float = 0.10) -> list[dict]:
    """Describe the plan-only sequence. The caller plans each waypoint with MoveIt."""
    lift = _finite(lift_distance_m, "lift_distance_m", minimum=0.02)
    x, y, z = _vector(place_surface_center_m, 3, "place_surface_center_m")
    object_height = obj["dimensions_m"][2]
    place_tcp = [x, y, z + object_height/2 + 0.002]
    preplace_tcp = [x, y, z + object_height/2 + grasp["approach_distance_m"] + 0.002]
    grasp_tcp = grasp["tcp_target_m"]
    pregrasp = grasp["pregrasp_target_m"]
    return [
        {"name": "pregrasp", "position_m": pregrasp, "orientation_xyzw": grasp["orientation_xyzw"], "object_attached": False},
        {"name": "grasp", "position_m": grasp_tcp, "orientation_xyzw": grasp["orientation_xyzw"], "object_attached": False},
        {"name": "lift", "position_m": [grasp_tcp[0], grasp_tcp[1], grasp_tcp[2] + lift], "orientation_xyzw": grasp["orientation_xyzw"], "object_attached": True},
        {"name": "preplace", "position_m": preplace_tcp, "orientation_xyzw": grasp["orientation_xyzw"], "object_attached": True},
        {"name": "place", "position_m": place_tcp, "orientation_xyzw": grasp["orientation_xyzw"], "object_attached": True},
        {"name": "retreat", "position_m": [x, y, place_tcp[2] + grasp["approach_distance_m"]], "orientation_xyzw": grasp["orientation_xyzw"], "object_attached": False},
    ]


def quaternion_multiply(a: list[float], b: list[float]) -> list[float]:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return [aw*bx + ax*bw + ay*bz - az*by,
            aw*by - ax*bz + ay*bw + az*bx,
            aw*bz + ax*by - ay*bx + az*bw,
            aw*bw - ax*bx - ay*by - az*bz]


def rotate_vector(quaternion: list[float], vector: list[float]) -> list[float]:
    x, y, z, w = quaternion
    vx, vy, vz = vector
    tx, ty, tz = 2*(y*vz-z*vy), 2*(z*vx-x*vz), 2*(x*vy-y*vx)
    return [vx + w*tx + y*tz-z*ty,
            vy + w*ty + z*tx-x*tz,
            vz + w*tz + x*ty-y*tx]


def pose_relative_to(parent_position: list[float], parent_orientation: list[float],
                     child_position: list[float], child_orientation: list[float]) -> dict:
    """Express child pose in parent frame; quaternion order is xyzw."""
    inverse = [-parent_orientation[0], -parent_orientation[1], -parent_orientation[2], parent_orientation[3]]
    delta = [child_position[i] - parent_position[i] for i in range(3)]
    return {"position_m": rotate_vector(inverse, delta),
            "orientation_xyzw": quaternion_multiply(inverse, child_orientation)}


def pose_compose(parent_position: list[float], parent_orientation: list[float],
                 relative_position: list[float], relative_orientation: list[float]) -> dict:
    """Compose a relative child pose into the world frame."""
    rotated = rotate_vector(parent_orientation, relative_position)
    return {"position_m": [parent_position[i] + rotated[i] for i in range(3)],
            "orientation_xyzw": quaternion_multiply(parent_orientation, relative_orientation)}
