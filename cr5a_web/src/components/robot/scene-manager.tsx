import { useEffect, useRef, useState, type ChangeEvent, type FormEvent } from "react";
import { ChevronDown, Download, Ellipsis, Eye, EyeOff, FileJson, Pencil, Plus, ShieldCheck, ShieldOff, Tag, Trash2, Upload, X } from "lucide-react";
import { Button } from "@/components/ui/button";
import type { CollisionObject, PlanningScene } from "@/lib/types";
import { useRobotStore } from "@/store/robot-store";

type Shape = "box" | "sphere" | "cylinder";
type Notice = { tone: "success" | "error" | "neutral"; message: string };
type Draft = {
  id: string; shape: Shape;
  x: number; y: number; z: number; rx: number; ry: number; rz: number;
  sizeX: number; sizeY: number; sizeZ: number; radius: number; height: number;
};
type Payload = {
  id?: string; shape: Shape; frame_id: string;
  position_m: number[]; orientation_xyzw: number[];
  dimensions_m?: number[]; radius_m?: number; height_m?: number;
};
type Request = (method: string, params?: Record<string, unknown>) => Promise<any>;

const SCENE_SESSION_STORAGE_KEY = "cr5a-visualizer-scene-session-v1";

const initialDraft = (): Draft => ({
  id: "", shape: "box", x: 300, y: 0, z: 100, rx: 0, ry: 0, rz: 0,
  sizeX: 100, sizeY: 100, sizeZ: 100, radius: 50, height: 100,
});

function toQuaternion(roll: number, pitch: number, yaw: number): number[] {
  const r = roll * Math.PI / 360, p = pitch * Math.PI / 360, y = yaw * Math.PI / 360;
  const cr = Math.cos(r), sr = Math.sin(r), cp = Math.cos(p), sp = Math.sin(p), cy = Math.cos(y), sy = Math.sin(y);
  return [sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy];
}

function fromQuaternion(q: number[]): [number, number, number] {
  const [x, y, z, w] = q;
  const r = Math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
  const s = 2 * (w * y - z * x);
  const p = Math.abs(s) >= 1 ? Math.sign(s) * Math.PI / 2 : Math.asin(s);
  const yv = Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
  return [r, p, yv].map((v) => v * 180 / Math.PI) as [number, number, number];
}

function composeObjectPose(object: CollisionObject, localPose: NonNullable<CollisionObject["primitive_poses"]>[number]) {
  const basePosition = object.pose?.position ?? { x: 0, y: 0, z: 0 };
  const baseOrientation = object.pose?.orientation ?? { x: 0, y: 0, z: 0, w: 1 };
  const [qx, qy, qz, qw] = [baseOrientation.x, baseOrientation.y, baseOrientation.z, baseOrientation.w];
  const p = [localPose.position.x, localPose.position.y, localPose.position.z];
  const cross = [qy * p[2] - qz * p[1], qz * p[0] - qx * p[2], qx * p[1] - qy * p[0]];
  const twiceCross = cross.map((value) => value * 2);
  const rotated = [
    p[0] + qw * twiceCross[0] + qy * twiceCross[2] - qz * twiceCross[1],
    p[1] + qw * twiceCross[1] + qz * twiceCross[0] - qx * twiceCross[2],
    p[2] + qw * twiceCross[2] + qx * twiceCross[1] - qy * twiceCross[0],
  ];
  const [x, y, z, w] = [localPose.orientation.x, localPose.orientation.y, localPose.orientation.z, localPose.orientation.w];
  return {
    position: { x: basePosition.x + rotated[0], y: basePosition.y + rotated[1], z: basePosition.z + rotated[2] },
    orientation: {
      x: qw * x + qx * w + qy * z - qz * y,
      y: qw * y - qx * z + qy * w + qz * x,
      z: qw * z + qx * y - qy * x + qz * w,
      w: qw * w - qx * x - qy * y - qz * z,
    },
  };
}

function toDraft(object: CollisionObject): Draft | null {
  if (object.primitives?.length !== 1 || object.primitive_poses?.length !== 1) return null;
  const primitive = object.primitives[0], pose = composeObjectPose(object, object.primitive_poses[0]), d = primitive.dimensions ?? [];
  const shape: Shape | null = primitive.type === 1 ? "box" : primitive.type === 2 ? "sphere" : primitive.type === 3 ? "cylinder" : null;
  if (!shape || (shape === "box" && d.length < 3) || (shape === "sphere" && d.length < 1) || (shape === "cylinder" && d.length < 2)) return null;
  const xyz = [pose.position.x, pose.position.y, pose.position.z];
  const q = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w];
  if (![...xyz, ...q, ...d].every(Number.isFinite) || q.every((n) => n === 0)) return null;
  const [rx, ry, rz] = fromQuaternion(q);
  return {
    id: object.id, shape, x: xyz[0] * 1000, y: xyz[1] * 1000, z: xyz[2] * 1000, rx, ry, rz,
    sizeX: (d[0] ?? 0) * 1000, sizeY: (d[1] ?? 0) * 1000, sizeZ: (d[2] ?? 0) * 1000,
    radius: (shape === "sphere" ? d[0] : d[1]) * 1000,
    height: (shape === "cylinder" ? d[0] : 100) * 1000,
  };
}

function toPayload(draft: Draft, frameId: string): Payload {
  const payload: Payload = {
    ...(draft.id.trim() ? { id: draft.id.trim() } : {}),
    shape: draft.shape, frame_id: frameId,
    position_m: [draft.x, draft.y, draft.z].map((v) => v / 1000),
    orientation_xyzw: toQuaternion(draft.rx, draft.ry, draft.rz),
  };
  if (draft.shape === "box") payload.dimensions_m = [draft.sizeX, draft.sizeY, draft.sizeZ].map((v) => v / 1000);
  if (draft.shape === "sphere") payload.radius_m = draft.radius / 1000;
  if (draft.shape === "cylinder") {
    payload.height_m = draft.height / 1000;
    payload.radius_m = draft.radius / 1000;
  }
  return payload;
}

function exportObject(object: CollisionObject, fallbackFrame: string): Record<string, unknown> | null {
  if (object.primitives?.length !== 1 || object.primitive_poses?.length !== 1) return null;
  const primitive = object.primitives[0], pose = composeObjectPose(object, object.primitive_poses[0]);
  const shape: Shape | null = primitive.type === 1 ? "box" : primitive.type === 2 ? "sphere" : primitive.type === 3 ? "cylinder" : null;
  if (!shape) return null;
  const out: Record<string, unknown> = {
    id: object.id, shape, frame_id: object.header?.frame_id || fallbackFrame,
    position_m: [pose.position.x, pose.position.y, pose.position.z],
    orientation_xyzw: [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w],
  };
  const d = primitive.dimensions ?? [];
  if (shape === "box" && d.length >= 3) out.dimensions_m = d.slice(0, 3);
  else if (shape === "sphere" && d.length >= 1) out.radius_m = d[0];
  else if (shape === "cylinder" && d.length >= 2) { out.height_m = d[0]; out.radius_m = d[1]; }
  else return null;
  return out;
}

export function collisionObjectToPayload(object: CollisionObject, fallbackFrame: string): Payload | null {
  return exportObject(object, fallbackFrame) as Payload | null;
}

const DEFAULT_CONSTRAINTS_JSON = `{
  "constraints": {
    "ground_plane": {
      "id": "ground_plane",
      "kind": "obstacle",
      "size_xy_m": [10.0, 10.0],
      "thickness_m": 0.05,
      "tag_id": 4,
      "default_surface_z_m": -0.7429
    },
    "base_pedestal": {
      "id": "base_pedestal",
      "kind": "obstacle",
      "size_xy_m": [0.15, 0.15],
      "top_offset_m": -0.01,
      "center_xy_m": [0.0, 0.0],
      "enabled": true
    },
    "Box_0": {
      "id": "Box_0",
      "kind": "obstacle",
      "size_m": [0.0405395, 0.0405395, 0.0405395],
      "position_m": [0.0, 0.15, 0.05],
      "orientation_quat": [0.0, 0.0, 0.0, 1.0],
      "enabled": true
    }
  }
}`;

function convertConstraintConfig(data: any, frameId: string): Payload[] {
  const constraints = data?.constraints ?? data;
  if (!constraints || typeof constraints !== "object" || Array.isArray(constraints)) {
    throw new Error("Expected a JSON object containing a constraints object.");
  }
  const result: Payload[] = [];
  const ids = new Set<string>();
  const readNumber = (value: unknown, field: string) => {
    if (typeof value !== "number" || !Number.isFinite(value)) throw new Error(`${field} must be a finite number.`);
    return value;
  };
  const readPair = (value: unknown, field: string) => {
    if (!Array.isArray(value) || value.length !== 2) throw new Error(`${field} must contain [x, y].`);
    return value.map((entry, index) => readNumber(entry, `${field}[${index}]`));
  };
  const readVector = (value: unknown, length: number, field: string) => {
    if (!Array.isArray(value) || value.length !== length) {
      throw new Error(`${field} must contain ${length} values.`);
    }
    return value.map((entry, index) => readNumber(entry, `${field}[${index}]`));
  };
  const add = (
    name: string, config: any, position: number[], dimensions: number[],
    orientation: number[] = [0, 0, 0, 1],
  ) => {
    const id = String(config.id ?? name);
    if (!/^[A-Za-z0-9_.-]{1,64}$/.test(id) || ids.has(id)) throw new Error(`Constraint "${name}" has an invalid or duplicate ID.`);
    if (dimensions.some((value) => !Number.isFinite(value) || value <= 0)) throw new Error(`Constraint "${name}" dimensions must be positive.`);
    ids.add(id);
    result.push({
      id, shape: "box", frame_id: frameId,
      position_m: position, orientation_xyzw: orientation,
      dimensions_m: dimensions,
    });
  };

  const ground = constraints.ground_plane;
  let groundSurfaceZ: number | null = null;
  if (ground && ground.enabled !== false) {
    if (ground.kind && ground.kind !== "obstacle") throw new Error('ground_plane.kind must be "obstacle".');
    const [sizeX, sizeY] = readPair(ground.size_xy_m, "ground_plane.size_xy_m");
    const thickness = readNumber(ground.thickness_m, "ground_plane.thickness_m");
    groundSurfaceZ = readNumber(ground.default_surface_z_m, "ground_plane.default_surface_z_m");
    const [centerX, centerY] = ground.center_xy_m === undefined ? [0, 0] : readPair(ground.center_xy_m, "ground_plane.center_xy_m");
    add("ground_plane", ground, [centerX, centerY, groundSurfaceZ - thickness / 2], [sizeX, sizeY, thickness]);
  }

  const pedestal = constraints.base_pedestal;
  if (pedestal && pedestal.enabled !== false) {
    if (pedestal.kind && pedestal.kind !== "obstacle") throw new Error('base_pedestal.kind must be "obstacle".');
    if (groundSurfaceZ === null) throw new Error("base_pedestal needs an enabled ground_plane to calculate its height.");
    const [sizeX, sizeY] = readPair(pedestal.size_xy_m, "base_pedestal.size_xy_m");
    const [centerX, centerY] = pedestal.center_xy_m === undefined ? [0, 0] : readPair(pedestal.center_xy_m, "base_pedestal.center_xy_m");
    const topZ = readNumber(pedestal.top_offset_m, "base_pedestal.top_offset_m");
    const height = topZ - groundSurfaceZ;
    if (height <= 0) throw new Error("base_pedestal.top_offset_m must be above ground_plane.default_surface_z_m.");
    add("base_pedestal", pedestal, [centerX, centerY, groundSurfaceZ + height / 2], [sizeX, sizeY, height]);
  }

  for (const [name, config] of Object.entries(constraints)) {
    if (name === "ground_plane" || name === "base_pedestal") continue;
    if (!config || typeof config !== "object" || (config as any).enabled === false) continue;
    const obstacle = config as any;
    if (obstacle.kind !== "obstacle") continue;
    const dimensions = readVector(obstacle.size_m, 3, `${name}.size_m`);
    const position = readVector(obstacle.position_m, 3, `${name}.position_m`);
    const orientation = readVector(obstacle.orientation_quat, 4, `${name}.orientation_quat`);
    if (orientation.every((value) => value === 0)) throw new Error(`${name}.orientation_quat cannot be a zero quaternion.`);
    add(name, obstacle, position, dimensions, orientation);
  }

  if (result.length === 0) throw new Error("No enabled obstacle constraints were found.");
  if (result.length > 100) throw new Error("Constraint conversion is limited to 100 obstacles.");
  return result;
}

function importObjects(data: any, frameId: string, allowExternalFrames = false): Payload[] {
  if (data?.format !== "cr5a-moveit-scene" || data?.version !== 1 || !Array.isArray(data.objects)) {
    throw new Error("Unsupported JSON. Use a MoveIt scene export (cr5a-moveit-scene, version 1).");
  }
  if (data.objects.length > 100) throw new Error("Import is limited to 100 objects.");
  const ids = new Set<string>();
  return data.objects.map((item: any, index: number) => {
    const shape = item?.shape as Shape;
    if (!["box", "sphere", "cylinder"].includes(shape)) throw new Error(`Object ${index + 1}: unsupported shape.`);
    const id = item.id === undefined ? undefined : String(item.id);
    if (id && (!/^[A-Za-z0-9_.-]{1,64}$/.test(id) || ids.has(id))) throw new Error(`Object ${index + 1}: invalid or duplicate ID.`);
    if (id) ids.add(id);
    const frame = String(item.frame_id ?? data.frame_id ?? frameId);
    if (!frame || (!allowExternalFrames && frame !== frameId)) {
      throw new Error(`Object ${id ?? index + 1}: frame "${frame}" does not match "${frameId}".`);
    }
    const numbers = (value: unknown, length: number, name: string) => {
      if (!Array.isArray(value) || value.length !== length || !value.every((v) => typeof v === "number" && Number.isFinite(v))) {
        throw new Error(`Object ${id ?? index + 1}: ${name} must contain ${length} finite numbers.`);
      }
      return value as number[];
    };
    const position = numbers(item.position_m, 3, "position_m");
    const orientation = numbers(item.orientation_xyzw ?? [0, 0, 0, 1], 4, "orientation_xyzw");
    if (orientation.every((v) => v === 0)) throw new Error(`Object ${id ?? index + 1}: invalid quaternion.`);
    const payload: Payload = { ...(id ? { id } : {}), shape, frame_id: frame, position_m: position, orientation_xyzw: orientation };
    if (shape === "box") {
      payload.dimensions_m = numbers(item.dimensions_m, 3, "dimensions_m");
      if (payload.dimensions_m.some((v) => v <= 0)) throw new Error("Box dimensions must be positive.");
    } else {
      if (typeof item.radius_m !== "number" || !Number.isFinite(item.radius_m) || item.radius_m <= 0) throw new Error("Radius must be positive.");
      payload.radius_m = item.radius_m;
      if (shape === "cylinder") {
        if (typeof item.height_m !== "number" || !Number.isFinite(item.height_m) || item.height_m <= 0) throw new Error("Cylinder height must be positive.");
        payload.height_m = item.height_m;
      }
    }
    return payload;
  });
}

function Field({ label, value, unit, onChange }: { label: string; value: number; unit: string; onChange: (n: number) => void }) {
  return <label className="block min-w-0">
    <span className="mb-1 block text-[10px] font-medium uppercase tracking-wider text-zinc-500">{label}</span>
    <div className="flex h-8 items-center rounded-md border border-zinc-200 bg-white focus-within:border-zinc-500">
      <input type="number" step="any" value={Number.isFinite(value) ? Number(value.toFixed(3)) : 0}
        onChange={(e) => onChange(Number(e.currentTarget.value))}
        className="min-w-0 flex-1 bg-transparent px-2 font-mono text-xs outline-none"/>
      <span className="px-2 text-[10px] text-zinc-400">{unit}</span>
    </div>
  </label>;
}

export function SceneManager({ scene, frameId, connected, modelReady, request, onNotice, onSceneChanged,
  objectLabels, onToggleObjectLabel, hiddenObjects, onToggleObjectVisibility,
  disabledObjects, onToggleCollision, onRemoveDisabledObject, onClearDisabledObjects, collisionBusy
}: {
  scene: PlanningScene | null; frameId: string; connected: boolean; modelReady: boolean; request: Request;
  onNotice: (notice: Notice | null) => void; onSceneChanged: () => void;
  objectLabels: Record<string, boolean>; onToggleObjectLabel: (id: string) => void;
  hiddenObjects: Record<string, boolean>; onToggleObjectVisibility: (id: string) => void;
  disabledObjects: Record<string, CollisionObject>; onToggleCollision: (object: CollisionObject) => void;
  onRemoveDisabledObject: (id: string) => void; onClearDisabledObjects: () => void;
  collisionBusy: string | null;
}) {
  const world = scene?.world?.collision_objects ?? [];
  const inactiveObjects = Object.values(disabledObjects);
  const objects = [...world, ...inactiveObjects.filter((object) => !world.some((active) => active.id === object.id))];
  const attached = scene?.robot_state?.attached_collision_objects ?? [];
  const sceneReady = scene !== null;
  const [open, setOpen] = useState(false);
  const [editingId, setEditingId] = useState<string | null>(null);
  const [draft, setDraft] = useState<Draft>(initialDraft);
  const [busy, setBusy] = useState(false);
  const [converterOpen, setConverterOpen] = useState(false);
  const [constraintsText, setConstraintsText] = useState(DEFAULT_CONSTRAINTS_JSON);
  const [convertedObjects, setConvertedObjects] = useState<Payload[] | null>(null);
  const defaultLoadStarted = useRef(false);
  const sceneSessionHydrated = useRef(false);
  const initiallyDisabledIds = useRef(new Set(Object.keys(disabledObjects)));
  const disabledCleanupInFlight = useRef(new Set<string>());
  const existingIds = new Set(objects.map((object) => object.id));
  const attachedIds = new Set(attached.map((item) => item.object?.id).filter((id): id is string => Boolean(id)));

  const refresh = async () => {
    const current = await request("moveit.get_scene");
    useRobotStore.getState().setScene(current);
    return current as PlanningScene;
  };

  // The bridge may seed its built-in objects before this page restores the
  // browser's disabled-object list. Reconcile those persisted disables once they appear.
  useEffect(() => {
    if (!connected || !initiallyDisabledIds.current.size) return;
    const toRemove = world.flatMap((object) => {
      const id = object.id;
      return id && initiallyDisabledIds.current.has(id) && !disabledCleanupInFlight.current.has(id)
        ? [{ object, id }]
        : [];
    });
    if (!toRemove.length) return;
    void (async () => {
      for (const { object, id } of toRemove) {
        disabledCleanupInFlight.current.add(id);
        try {
          await request("scene.remove", { id });
        } catch (error) {
          onNotice({ tone: "error", message: `Could not restore disabled collision state for ${id}: ${error instanceof Error ? error.message : String(error)}` });
        } finally {
          disabledCleanupInFlight.current.delete(id);
        }
      }
      await refresh();
      onSceneChanged();
    })();
  }, [connected, world, request, onNotice, onSceneChanged]);

  useEffect(() => {
    if (!connected || !modelReady || !sceneReady || defaultLoadStarted.current) return;
    defaultLoadStarted.current = true;
    const initializeScene = async () => {
      try {
        const latest = await request("moveit.get_scene") as PlanningScene;
        const currentIds = new Set<string>((latest?.world?.collision_objects ?? []).flatMap((object) => object.id ? [object.id] : []));
        const currentAttachedIds = new Set<string>((latest?.robot_state?.attached_collision_objects ?? [])
          .flatMap((item) => item.object?.id ? [item.object.id] : []));
        const disabledIds = new Set(Object.keys(disabledObjects));

        // Restore the last browser-managed scene; default collision geometry is seeded by the ROS bridge.
        let restoredCount = 0;
        try {
          const saved = localStorage.getItem(SCENE_SESSION_STORAGE_KEY);
          if (saved) {
            const parsed = JSON.parse(saved);
            const savedObjects = importObjects(parsed, frameId, true);
            for (const object of savedObjects) {
              const id = object.id;
              if (!id || currentIds.has(id) || currentAttachedIds.has(id) || disabledIds.has(id)) continue;
              await request("scene.add", object);
              currentIds.add(id);
              restoredCount += 1;
            }
          }
        } catch (error) {
          onNotice({ tone: "error", message: `Saved scene could not be restored: ${error instanceof Error ? error.message : String(error)}` });
        }

        sceneSessionHydrated.current = true;
        await refresh();
        onSceneChanged();
        if (restoredCount) {
          onNotice({ tone: "success", message: `Restored ${restoredCount} saved scene object(s) into MoveIt.` });
        }
      } catch (error) {
        defaultLoadStarted.current = false;
        onNotice({ tone: "error", message: `Could not initialize MoveIt scene: ${error instanceof Error ? error.message : String(error)}` });
      }
    };
    void initializeScene();
  }, [connected, modelReady, sceneReady, frameId, request, onNotice, onSceneChanged, disabledObjects]);

  useEffect(() => {
    if (!sceneSessionHydrated.current || !scene) return;
    const savedObjects = (scene.world?.collision_objects ?? [])
      .map((object) => exportObject(object, frameId))
      .filter((object): object is Record<string, unknown> => object !== null);
    try {
      localStorage.setItem(SCENE_SESSION_STORAGE_KEY, JSON.stringify({
        format: "cr5a-moveit-scene", version: 1, frame_id: frameId, objects: savedObjects,
      }));
    } catch (error) {
      onNotice({ tone: "error", message: `Could not persist scene session: ${error instanceof Error ? error.message : String(error)}` });
    }
  }, [scene, frameId, onNotice]);
  const add = () => { setEditingId(null); setDraft(initialDraft()); setOpen(true); };
  const edit = (object: CollisionObject) => {
    if ((object.header?.frame_id ?? frameId) !== frameId) {
      onNotice({ tone: "error", message: `Edit currently supports objects in ${frameId} only.` }); return;
    }
    const next = toDraft(object);
    if (!next) { onNotice({ tone: "error", message: "Only single box, sphere, or cylinder objects can be edited." }); return; }
    setEditingId(object.id); setDraft(next); setOpen(true);
  };
  const field = (key: keyof Draft, value: string | number) => setDraft((old) => ({ ...old, [key]: value }));

  const save = async (event: FormEvent<HTMLFormElement>) => {
    event.preventDefault();
    if (busy || !connected) return;
    const values = [draft.x, draft.y, draft.z, draft.rx, draft.ry, draft.rz,
      ...(draft.shape === "box" ? [draft.sizeX, draft.sizeY, draft.sizeZ] : draft.shape === "sphere" ? [draft.radius] : [draft.radius, draft.height])];
    if (!values.every(Number.isFinite)) { onNotice({ tone: "error", message: "All values must be finite numbers." }); return; }
    const sizes = draft.shape === "box" ? [draft.sizeX, draft.sizeY, draft.sizeZ] : draft.shape === "sphere" ? [draft.radius] : [draft.radius, draft.height];
    if (sizes.some((value) => value <= 0)) { onNotice({ tone: "error", message: "Dimensions must be greater than zero." }); return; }
    if (draft.id.trim() && !/^[A-Za-z0-9_.-]{1,64}$/.test(draft.id.trim())) {
      onNotice({ tone: "error", message: "ID may contain letters, numbers, dots, underscores and hyphens (max 64 characters)." }); return;
    }
    if (!editingId && draft.id.trim() && (existingIds.has(draft.id.trim()) || attachedIds.has(draft.id.trim()))) {
      onNotice({ tone: "error", message: "That ID already exists in the planning scene. Edit the existing object or choose another ID." }); return;
    }
    setBusy(true); onNotice(null);
    try {
      await request(editingId ? "scene.update" : "scene.add", toPayload({ ...draft, id: editingId ?? draft.id }, frameId));
      await refresh(); onSceneChanged(); setOpen(false); setEditingId(null);
      onNotice({ tone: "success", message: editingId ? `Updated "${editingId}" in MoveIt.` : "Added collision object to the MoveIt scene." });
    } catch (error) {
      onNotice({ tone: "error", message: error instanceof Error ? error.message : String(error) });
    } finally { setBusy(false); }
  };

  const remove = async (id: string) => {
    if (disabledObjects[id] && !world.some((object) => object.id === id)) {
      onRemoveDisabledObject(id);
      onNotice({ tone: "success", message: `Removed inactive object "${id}" from the visualizer.` });
      return;
    }
    if (busy || !connected) return;
    setBusy(true); onNotice(null);
    try {
      await request("scene.remove", { id }); await refresh(); onSceneChanged();
      if (editingId === id) { setOpen(false); setEditingId(null); }
      onNotice({ tone: "success", message: `Removed "${id}" from MoveIt.` });
    } catch (error) { onNotice({ tone: "error", message: error instanceof Error ? error.message : String(error) }); }
    finally { setBusy(false); }
  };

  const clear = async () => {
    if (busy || !connected || (world.length === 0 && !inactiveObjects.length) ||
      !window.confirm(`Remove all ${world.length + inactiveObjects.length} scene objects, including collision-disabled objects?`)) return;
    setBusy(true); onNotice(null);
    try {
      await request("scene.clear"); await refresh(); onClearDisabledObjects(); onSceneChanged(); setOpen(false); setEditingId(null);
      onNotice({ tone: "success", message: "Cleared world collision objects. Attached objects were left unchanged." });
    } catch (error) { onNotice({ tone: "error", message: error instanceof Error ? error.message : String(error) }); }
    finally { setBusy(false); }
  };

  const importFile = async (event: ChangeEvent<HTMLInputElement>) => {
    const file = event.currentTarget.files?.[0]; event.currentTarget.value = "";
    if (!file) return;
    if (file.size > 1024 * 1024) { onNotice({ tone: "error", message: "Scene file exceeds 1 MB." }); return; }
    if (busy || !connected) return;
    setBusy(true); onNotice(null); let imported = 0;
    try {
      const payloads = importObjects(JSON.parse(await file.text()), frameId);
      const activeIds = new Set(world.map((object) => object.id));
      for (const payload of payloads) {
        if (payload.id && disabledObjects[payload.id]) {
          throw new Error(`ID "${payload.id}" is currently collision-disabled. Enable or remove it before importing a replacement.`);
        }
        if (payload.id && attachedIds.has(payload.id) && !activeIds.has(payload.id)) {
          throw new Error(`ID "${payload.id}" belongs to an attached object. Detach or rename it before importing.`);
        }
        await request(payload.id && activeIds.has(payload.id) ? "scene.update" : "scene.add", payload);
        imported += 1;
      }
      await refresh(); if (imported) onSceneChanged();
      onNotice({ tone: "success", message: `Imported ${imported} object(s) into MoveIt.` });
    } catch (error) {
      if (imported) { try { await refresh(); onSceneChanged(); } catch { /* preserve import error */ } }
      onNotice({ tone: "error", message: `Import stopped after ${imported} object(s): ${error instanceof Error ? error.message : String(error)}` });
    } finally { setBusy(false); }
  };

  const loadConstraintsFile = async (event: ChangeEvent<HTMLInputElement>) => {
    const file = event.currentTarget.files?.[0];
    event.currentTarget.value = "";
    if (!file) return;
    if (file.size > 1024 * 1024) {
      onNotice({ tone: "error", message: "Constraint JSON file exceeds 1 MB." });
      return;
    }
    try {
      const text = await file.text();
      JSON.parse(text);
      setConstraintsText(text);
      setConvertedObjects(null);
      onNotice({ tone: "success", message: `Loaded ${file.name}. Convert it to preview the obstacles.` });
    } catch (error) {
      onNotice({ tone: "error", message: error instanceof Error ? error.message : "Could not read the JSON file." });
    }
  };

  const toggleConstraintConverter = () => {
    if (converterOpen) {
      setConverterOpen(false);
      return;
    }
    setConverterOpen(true);
    try {
      setConvertedObjects(convertConstraintConfig(JSON.parse(constraintsText), frameId));
      onNotice(null);
    } catch (error) {
      setConvertedObjects(null);
      onNotice({ tone: "error", message: error instanceof Error ? error.message : String(error) });
    }
  };

  const convertConstraints = () => {
    try {
      const converted = convertConstraintConfig(JSON.parse(constraintsText), frameId);
      setConvertedObjects(converted);
      onNotice({ tone: "success", message: `Converted ${converted.length} obstacle(s) for MoveIt.` });
    } catch (error) {
      setConvertedObjects(null);
      onNotice({ tone: "error", message: error instanceof Error ? error.message : String(error) });
    }
  };

  const addConvertedConstraints = async () => {
    if (!convertedObjects || busy || !connected) return;
    setBusy(true);
    onNotice(null);
    let added = 0;
    try {
      for (const payload of convertedObjects) {
        if (payload.id && attachedIds.has(payload.id) && !existingIds.has(payload.id)) {
          throw new Error(`ID "${payload.id}" belongs to an attached object.`);
        }
        await request(payload.id && existingIds.has(payload.id) ? "scene.update" : "scene.add", payload);
        added += 1;
      }
      await refresh();
      onSceneChanged();
      setConverterOpen(false);
      setConvertedObjects(null);
      onNotice({ tone: "success", message: `Added ${added} converted obstacle(s) to the MoveIt planning scene.` });
    } catch (error) {
      if (added) {
        try { await refresh(); onSceneChanged(); } catch { /* preserve partial-add error */ }
      }
      onNotice({ tone: "error", message: `Added ${added} obstacle(s) before conversion import stopped: ${error instanceof Error ? error.message : String(error)}` });
    } finally { setBusy(false); }
  };

  const downloadConvertedJson = () => {
    if (!convertedObjects) return;
    const doc = {
      format: "cr5a-moveit-scene", version: 1, frame_id: frameId,
      units: { position: "m", dimensions: "m", orientation: "quaternion_xyzw" },
      objects: convertedObjects,
    };
    const url = URL.createObjectURL(new Blob([JSON.stringify(doc, null, 2)], { type: "application/json" }));
    const anchor = document.createElement("a");
    anchor.href = url; anchor.download = "moveit-scene-converted.json"; anchor.click();
    URL.revokeObjectURL(url);
  };

  const exportFile = () => {
    const objects = world.flatMap((object) => {
      if (object.header?.frame_id && object.header.frame_id !== frameId) return [];
      const item = exportObject(object, frameId);
      return item ? [item] : [];
    });
    if (!objects.length) { onNotice({ tone: "error", message: "No supported world objects to export." }); return; }
    const doc = { format: "cr5a-moveit-scene", version: 1, frame_id: frameId,
      units: { position: "m", dimensions: "m", orientation: "quaternion_xyzw" }, objects };
    const url = URL.createObjectURL(new Blob([JSON.stringify(doc, null, 2)], { type: "application/json" }));
    const anchor = document.createElement("a"); anchor.href = url; anchor.download = "moveit-scene.json"; anchor.click(); URL.revokeObjectURL(url);
    const skipped = world.length - objects.length;
    onNotice({ tone: skipped ? "neutral" : "success", message: `Exported ${objects.length} world object(s)${skipped ? `; skipped ${skipped} unsupported object(s)` : ""}. Attached objects are not included.` });
  };

  return <section className="panel">
    <div className="panel-header flex-wrap">
      <div><div className="text-sm font-semibold">Scene objects</div><div className="mt-0.5 text-[11px] text-zinc-400">{objects.length} objects · {world.length} collision active · {attached.length} attached</div></div>
      <div className="flex flex-wrap items-center gap-1.5">
        <Button size="sm" onClick={add} disabled={!connected || busy}><Plus size={13}/> Add</Button>
        <details className="relative">
          <summary title="Import, export, convert or clear planning-scene objects" className="flex h-8 cursor-pointer list-none items-center gap-1.5 rounded-md border border-zinc-300 bg-white px-2.5 text-xs font-medium text-zinc-700 hover:bg-zinc-50">
            More <ChevronDown size={12}/>
          </summary>
          <div className="absolute right-0 z-30 mt-1 flex w-48 flex-col gap-1 rounded-lg border border-zinc-200 bg-white p-2 shadow-xl">
            <button type="button" title="Convert a constraints JSON document into MoveIt collision objects." disabled={!connected || busy} onClick={toggleConstraintConverter} className="flex items-center gap-2 rounded px-2 py-2 text-left text-xs hover:bg-zinc-100 disabled:opacity-40"><FileJson size={13}/> Convert constraints</button>
            <label title="Import supported collision objects from a JSON file." className={`flex cursor-pointer items-center gap-2 rounded px-2 py-2 text-xs hover:bg-zinc-100 ${!connected || busy ? "pointer-events-none opacity-40" : ""}`}>
              <Upload size={13}/> Import JSON
              <input type="file" accept=".json,application/json" className="hidden" disabled={!connected || busy} onChange={importFile}/>
            </label>
            <button type="button" title="Export supported world collision objects. Attached objects are not included." disabled={!world.length || busy} onClick={exportFile} className="flex items-center gap-2 rounded px-2 py-2 text-left text-xs hover:bg-zinc-100 disabled:opacity-40"><Download size={13}/> Export JSON</button>
            <button type="button" title="Remove all world collision objects. Attached objects remain unchanged." disabled={!connected || !objects.length || busy} onClick={clear} className="flex items-center gap-2 rounded px-2 py-2 text-left text-xs text-red-600 hover:bg-red-50 disabled:opacity-40"><Trash2 size={13}/> Clear scene</button>
          </div>
        </details>
      </div>
    </div>

    {converterOpen && <div className="space-y-3 border-b border-zinc-100 px-4 py-4">
      <div className="flex items-center justify-between gap-2">
        <div><div className="text-xs font-semibold">Constraint config converter</div><div className="mt-0.5 text-[10px] text-zinc-500">Default constraints are preloaded. Edit, paste, or load another JSON file.</div></div>
        <Button type="button" size="icon" variant="ghost" className="h-7 w-7" aria-label="Close converter" onClick={() => { setConverterOpen(false); setConvertedObjects(null); }}><X size={14}/></Button>
      </div>
      <div className="flex justify-end">
        <label className="inline-flex h-8 cursor-pointer items-center gap-1.5 rounded-md border border-zinc-300 bg-white px-2.5 text-xs font-medium text-zinc-800 hover:bg-zinc-50">
          <Upload size={13}/> Load JSON file
          <input type="file" accept=".json,application/json" className="hidden" onChange={loadConstraintsFile}/>
        </label>
      </div>
      <textarea value={constraintsText} onChange={(event) => { setConstraintsText(event.currentTarget.value); setConvertedObjects(null); }}
        spellCheck={false} rows={13} className="w-full resize-y rounded-md border border-zinc-200 bg-zinc-50 p-3 font-mono text-[11px] leading-5 outline-none focus:border-zinc-500"/>
      <div className="rounded-md bg-zinc-50 p-3 text-[11px] leading-5 text-zinc-600">
        Conversion rules: <code>ground_plane.default_surface_z_m</code> is treated as the plane top face; its box centre is half the thickness below it. <code>base_pedestal.top_offset_m</code> is treated as the pedestal top Z in the selected MoveIt frame; its bottom rests on the ground-plane top. <code>tag_id</code> is metadata and is not collision geometry.
      </div>
      <div className="flex flex-wrap items-center justify-between gap-2">
        <Button type="button" size="sm" variant="outline" onClick={convertConstraints} disabled={busy}>Convert to MoveIt JSON</Button>
        {convertedObjects && <div className="flex flex-wrap items-center gap-2">
          <span className="text-[11px] text-zinc-500">{convertedObjects.length} obstacle(s) · frame {frameId}</span>
          <Button type="button" size="sm" variant="outline" onClick={downloadConvertedJson}><Download size={13}/> JSON</Button>
          <Button type="button" size="sm" onClick={() => void addConvertedConstraints()} disabled={!connected || busy}>{busy ? "Adding…" : "Add to scene"}</Button>
        </div>}
      </div>
      {convertedObjects && <pre className="max-h-56 overflow-auto rounded-md border border-zinc-200 bg-zinc-950 p-3 text-[10px] leading-4 text-zinc-100">{JSON.stringify({
        format: "cr5a-moveit-scene", version: 1, frame_id: frameId,
        units: { position: "m", dimensions: "m", orientation: "quaternion_xyzw" },
        objects: convertedObjects,
      }, null, 2)}</pre>}
    </div>}

    {open && <form onSubmit={save} className="space-y-3 border-b border-zinc-100 px-4 py-4">
      <div className="flex items-center justify-between"><div className="text-xs font-semibold">{editingId ? `Edit · ${editingId}` : "Add collision object"}</div>
        <Button type="button" size="icon" variant="ghost" className="h-7 w-7" aria-label="Close editor" onClick={() => { setOpen(false); setEditingId(null); }}><X size={14}/></Button>
      </div>
      <div className="grid grid-cols-2 gap-3">
        <label className="block"><span className="mb-1 block text-[10px] uppercase tracking-wider text-zinc-500">Shape</span>
          <select value={draft.shape} onChange={(e) => field("shape", e.currentTarget.value as Shape)} className="h-8 w-full rounded-md border border-zinc-200 bg-white px-2 text-xs">
            <option value="box">Box</option><option value="sphere">Sphere</option><option value="cylinder">Cylinder</option>
          </select>
        </label>
        <label className="block"><span className="mb-1 block text-[10px] uppercase tracking-wider text-zinc-500">Object ID (optional)</span>
          <input value={draft.id} disabled={!!editingId} placeholder="Auto-generated" onChange={(e) => field("id", e.currentTarget.value)} className="h-8 w-full rounded-md border border-zinc-200 px-2 font-mono text-xs disabled:bg-zinc-50"/>
        </label>
      </div>
      <div><div className="mb-2 text-[10px] font-medium uppercase tracking-wider text-zinc-500">Position · {frameId} · mm</div>
        <div className="grid grid-cols-3 gap-2">
          <Field label="X" value={draft.x} unit="mm" onChange={(v) => field("x", v)}/><Field label="Y" value={draft.y} unit="mm" onChange={(v) => field("y", v)}/><Field label="Z" value={draft.z} unit="mm" onChange={(v) => field("z", v)}/>
        </div>
      </div>
      <div><div className="mb-2 text-[10px] font-medium uppercase tracking-wider text-zinc-500">Orientation · RPY degrees</div>
        <div className="grid grid-cols-3 gap-2">
          <Field label="RX" value={draft.rx} unit="°" onChange={(v) => field("rx", v)}/><Field label="RY" value={draft.ry} unit="°" onChange={(v) => field("ry", v)}/><Field label="RZ" value={draft.rz} unit="°" onChange={(v) => field("rz", v)}/>
        </div>
      </div>
      {draft.shape === "box" && <div><div className="mb-2 text-[10px] font-medium uppercase tracking-wider text-zinc-500">Dimensions · mm</div><div className="grid grid-cols-3 gap-2">
        <Field label="Width X" value={draft.sizeX} unit="mm" onChange={(v) => field("sizeX", v)}/><Field label="Depth Y" value={draft.sizeY} unit="mm" onChange={(v) => field("sizeY", v)}/><Field label="Height Z" value={draft.sizeZ} unit="mm" onChange={(v) => field("sizeZ", v)}/>
      </div></div>}
      {draft.shape === "sphere" && <Field label="Radius" value={draft.radius} unit="mm" onChange={(v) => field("radius", v)}/>}
      {draft.shape === "cylinder" && <div className="grid grid-cols-2 gap-2"><Field label="Height" value={draft.height} unit="mm" onChange={(v) => field("height", v)}/><Field label="Radius" value={draft.radius} unit="mm" onChange={(v) => field("radius", v)}/></div>}
      <div className="flex justify-end gap-2"><Button type="button" size="sm" variant="outline" onClick={() => { setOpen(false); setEditingId(null); }}>Cancel</Button><Button type="submit" size="sm" disabled={!connected || busy}>{busy ? "Saving…" : editingId ? "Save changes" : "Add to scene"}</Button></div>
    </form>}

    <div className="max-h-80 divide-y divide-zinc-100 overflow-auto">
      {objects.map((object) => {
        const collisionEnabled = world.some((active) => active.id === object.id);
        const hidden = hiddenObjects[object.id] === true;
        const labelVisible = objectLabels[object.id] !== false;
        return <div key={object.id} className="flex items-center justify-between gap-2 px-4 py-2.5">
        <div className="flex min-w-0 items-center gap-2"><FileJson size={13} className="shrink-0 text-zinc-400"/><div className="min-w-0"><div className="truncate text-xs font-medium text-zinc-700">{object.id}</div>
          <div className="text-[10px] text-zinc-400">{object.header?.frame_id && object.header.frame_id !== frameId ? "other frame" : toDraft(object)?.shape ?? "unsupported geometry"} · {object.header?.frame_id ?? frameId}</div></div></div>
        <div className="flex shrink-0 items-center gap-1">
          <span title={collisionEnabled ? "Collision enabled in MoveIt" : "Collision disabled in MoveIt"}
            className={collisionEnabled ? "mr-1 h-2 w-2 shrink-0 rounded-full bg-emerald-500" : "mr-1 h-2 w-2 shrink-0 rounded-full bg-amber-500"}/>
          <Button size="icon" variant="ghost" className="h-7 w-7"
            title={hidden ? "Show object in the 3D viewer. This does not alter collision checking." : "Hide object in the 3D viewer. This does not alter collision checking."}
            aria-label={hidden ? `Show ${object.id} in 3D viewer` : `Hide ${object.id} in 3D viewer`}
            onClick={() => onToggleObjectVisibility(object.id)}>
            {hidden ? <EyeOff size={13}/> : <Eye size={13}/>}
          </Button>
          <Button size="icon" variant="ghost" className={collisionEnabled ? "h-7 w-7 text-emerald-700" : "h-7 w-7 text-amber-700"}
            title={collisionEnabled ? "Disable this object's collision participation in MoveIt. It remains visible." : "Enable this object's collision participation in MoveIt."}
            aria-label={collisionEnabled ? `Disable collision for ${object.id}` : `Enable collision for ${object.id}`}
            disabled={!connected || busy || collisionBusy !== null || !toDraft(object)}
            onClick={() => onToggleCollision(object)}>
            {collisionEnabled ? <ShieldCheck size={13}/> : <ShieldOff size={13}/>}
          </Button>
          <details className="relative">
            <summary title="More object actions" aria-label={`More actions for ${object.id}`} className="flex h-7 w-7 cursor-pointer list-none items-center justify-center rounded-md text-zinc-500 hover:bg-zinc-100"><Ellipsis size={15}/></summary>
            <div className="absolute right-0 z-30 mt-1 flex w-40 flex-col gap-1 rounded-lg border border-zinc-200 bg-white p-1.5 shadow-xl">
              <button type="button" title={labelVisible ? "Hide this object's name in the viewer." : "Show this object's name in the viewer."} onClick={() => onToggleObjectLabel(object.id)} className="flex items-center gap-2 rounded px-2 py-1.5 text-left text-xs hover:bg-zinc-100"><Tag size={12}/>{labelVisible ? "Hide label" : "Show label"}</button>
              <button type="button" title="Edit geometry and pose. Only supported, active world objects in the MoveIt base frame can be edited." disabled={!connected || busy || !collisionEnabled || !toDraft(object) || (!!object.header?.frame_id && object.header.frame_id !== frameId)} onClick={() => edit(object)} className="flex items-center gap-2 rounded px-2 py-1.5 text-left text-xs hover:bg-zinc-100 disabled:opacity-40"><Pencil size={12}/> Edit object</button>
              <button type="button" title="Remove this object from the MoveIt planning scene." disabled={!connected || busy} onClick={() => void remove(object.id)} className="flex items-center gap-2 rounded px-2 py-1.5 text-left text-xs text-red-600 hover:bg-red-50 disabled:opacity-40"><Trash2 size={12}/> Remove object</button>
            </div>
          </details>
        </div>
        </div>;
      })}
      {attached.map((item) => <div key={item.object?.id} className="flex items-center justify-between gap-3 px-4 py-2.5"><span className="truncate text-xs text-zinc-700">{item.object?.id ?? "Attached object"}</span><span className="text-[10px] text-zinc-400">{item.link_name ?? "link ?"}</span></div>)}
      {!objects.length && !attached.length && <div className="px-4 py-5 text-xs text-zinc-400">No collision objects in the current MoveIt planning scene.</div>}
    </div>
    <div className="border-t border-zinc-100 px-4 py-2.5 text-[10px] text-zinc-400">MoveIt only · editor uses mm · JSON uses metres · attached objects are not editable or exported here.</div>
  </section>;
}
