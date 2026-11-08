import { useState, type ChangeEvent, type FormEvent } from "react";
import { Download, FileJson, Pencil, Plus, Trash2, Upload, X } from "lucide-react";
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

function toDraft(object: CollisionObject): Draft | null {
  if (object.primitives?.length !== 1 || object.primitive_poses?.length !== 1) return null;
  const primitive = object.primitives[0], pose = object.primitive_poses[0], d = primitive.dimensions ?? [];
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
  const primitive = object.primitives[0], pose = object.primitive_poses[0];
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

function importObjects(data: any, frameId: string): Payload[] {
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
    if (frame !== frameId) throw new Error(`Object ${id ?? index + 1}: frame "${frame}" does not match "${frameId}".`);
    const numbers = (value: unknown, length: number, name: string) => {
      if (!Array.isArray(value) || value.length !== length || !value.every((v) => typeof v === "number" && Number.isFinite(v))) {
        throw new Error(`Object ${id ?? index + 1}: ${name} must contain ${length} finite numbers.`);
      }
      return value as number[];
    };
    const position = numbers(item.position_m, 3, "position_m");
    const orientation = numbers(item.orientation_xyzw ?? [0, 0, 0, 1], 4, "orientation_xyzw");
    if (orientation.every((v) => v === 0)) throw new Error(`Object ${id ?? index + 1}: invalid quaternion.`);
    const payload: Payload = { ...(id ? { id } : {}), shape, frame_id: frameId, position_m: position, orientation_xyzw: orientation };
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

export function SceneManager({ scene, frameId, connected, request, onNotice, onSceneChanged }: {
  scene: PlanningScene | null; frameId: string; connected: boolean; request: Request;
  onNotice: (notice: Notice | null) => void; onSceneChanged: () => void;
}) {
  const world = scene?.world?.collision_objects ?? [];
  const attached = scene?.robot_state?.attached_collision_objects ?? [];
  const [open, setOpen] = useState(false);
  const [editingId, setEditingId] = useState<string | null>(null);
  const [draft, setDraft] = useState<Draft>(initialDraft);
  const [busy, setBusy] = useState(false);
  const existingIds = new Set(world.map((object) => object.id));
  const attachedIds = new Set(attached.map((item) => item.object?.id).filter((id): id is string => Boolean(id)));

  const refresh = async () => {
    const current = await request("moveit.get_scene");
    useRobotStore.getState().setScene(current);
  };
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
    if (busy || !connected || world.length === 0 || !window.confirm(`Remove all ${world.length} world objects from MoveIt?`)) return;
    setBusy(true); onNotice(null);
    try {
      await request("scene.clear"); await refresh(); onSceneChanged(); setOpen(false); setEditingId(null);
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
      for (const payload of payloads) {
        if (payload.id && attachedIds.has(payload.id) && !existingIds.has(payload.id)) {
          throw new Error(`ID "${payload.id}" belongs to an attached object. Detach or rename it before importing.`);
        }
        await request(payload.id && existingIds.has(payload.id) ? "scene.update" : "scene.add", payload);
        imported += 1;
      }
      await refresh(); if (imported) onSceneChanged();
      onNotice({ tone: "success", message: `Imported ${imported} object(s) into MoveIt.` });
    } catch (error) {
      if (imported) { try { await refresh(); onSceneChanged(); } catch { /* preserve import error */ } }
      onNotice({ tone: "error", message: `Import stopped after ${imported} object(s): ${error instanceof Error ? error.message : String(error)}` });
    } finally { setBusy(false); }
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
      <div><div className="text-sm font-semibold">Scene objects</div><div className="mt-0.5 text-[11px] text-zinc-400">{world.length} world · {attached.length} attached</div></div>
      <div className="flex flex-wrap items-center gap-1.5">
        <Button size="sm" onClick={add} disabled={!connected || busy}><Plus size={13}/> Add</Button>
        <label className={`inline-flex h-8 cursor-pointer items-center gap-1.5 rounded-md border border-zinc-300 bg-white px-2.5 text-xs font-medium hover:bg-zinc-50 ${!connected || busy ? "pointer-events-none opacity-40" : ""}`}>
          <Upload size={13}/> Import
          <input type="file" accept=".json,application/json" className="hidden" disabled={!connected || busy} onChange={importFile}/>
        </label>
        <Button size="sm" variant="outline" onClick={exportFile} disabled={!world.length || busy}><Download size={13}/> Export</Button>
        <Button size="sm" variant="ghost" onClick={clear} disabled={!connected || !world.length || busy}><Trash2 size={13}/> Clear</Button>
      </div>
    </div>

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

    <div className="max-h-60 divide-y divide-zinc-100 overflow-auto">
      {world.map((object) => <div key={object.id} className="flex items-center justify-between gap-2 px-4 py-2.5">
        <div className="flex min-w-0 items-center gap-2"><FileJson size={13} className="shrink-0 text-zinc-400"/><div className="min-w-0"><div className="truncate text-xs font-medium text-zinc-700">{object.id}</div>
          <div className="text-[10px] text-zinc-400">{object.header?.frame_id && object.header.frame_id !== frameId ? "other frame" : toDraft(object)?.shape ?? "unsupported geometry"} · {object.header?.frame_id ?? frameId}</div></div></div>
        <div className="flex shrink-0 items-center gap-1">
          <Button size="icon" variant="ghost" className="h-7 w-7" title="Edit object" aria-label={`Edit ${object.id}`} disabled={!connected || busy || !toDraft(object) || (!!object.header?.frame_id && object.header.frame_id !== frameId)} onClick={() => edit(object)}><Pencil size={13}/></Button>
          <Button size="icon" variant="ghost" className="h-7 w-7 text-red-600 hover:bg-red-50 hover:text-red-700" title="Remove object" aria-label={`Remove ${object.id}`} disabled={!connected || busy} onClick={() => void remove(object.id)}><Trash2 size={13}/></Button>
        </div>
      </div>)}
      {attached.map((item) => <div key={item.object?.id} className="flex items-center justify-between gap-3 px-4 py-2.5"><span className="truncate text-xs text-zinc-700">{item.object?.id ?? "Attached object"}</span><span className="text-[10px] text-zinc-400">{item.link_name ?? "link ?"}</span></div>)}
      {!world.length && !attached.length && <div className="px-4 py-5 text-xs text-zinc-400">No collision objects in the current MoveIt planning scene.</div>}
    </div>
    <div className="border-t border-zinc-100 px-4 py-2.5 text-[10px] text-zinc-400">MoveIt only · editor uses mm · JSON uses metres · attached objects are not editable or exported here.</div>
  </section>;
}
