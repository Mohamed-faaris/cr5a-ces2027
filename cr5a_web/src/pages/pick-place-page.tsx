import { useMemo, useState } from "react";
import { Box, ChevronDown, ClipboardCopy, Layers3, Package, RotateCcw, Ruler, ScanLine, ShieldCheck, Target } from "lucide-react";
import { Button } from "@/components/ui/button";
import { useRobotStore } from "@/store/robot-store";

type ObjectKind = "coke" | "pringles" | "wood_box" | "custom";
type InputSource = "synthetic" | "vision" | "manual";
type Dimensions = { x: number; y: number; z: number };
type Workpiece = { kind: ObjectKind; label: string; shape: "cylinder" | "box"; dimensions: Dimensions; massKg: number; massSource: string; densityKgM3?: number };

const OBJECTS: Record<Exclude<ObjectKind,"custom">,Workpiece> = {
  coke: { kind:"coke", label:"Coca-Cola can · 330 ml", shape:"cylinder", dimensions:{x:66,y:66,z:122}, massKg:0.355, massSource:"Estimated filled 330 ml can: ~342 g beverage + ~13 g can" },
  pringles: { kind:"pringles", label:"Pringles tube · 165 g", shape:"cylinder", dimensions:{x:76,y:76,z:240}, massKg:0.205, massSource:"165 g contents + estimated 40 g tube / lid" },
  wood_box: { kind:"wood_box", label:"Wooden box · configurable", shape:"box", dimensions:{x:120,y:90,z:80}, massKg:0.648, massSource:"Estimated from 120×90×80 mm solid volume × 750 kg/m³", densityKgM3:750 },
};

const SHELVES = [
  { id:"compact", label:"Compact", width:420, height:620, depth:300, levels:2, board:18 },
  { id:"standard", label:"Standard", width:600, height:900, depth:400, levels:3, board:18 },
  { id:"wide", label:"Wide", width:900, height:1200, depth:450, levels:4, board:20 },
  { id:"deep", label:"Deep storage", width:700, height:1000, depth:600, levels:3, board:22 },
] as const;

const number = (value: number, digits=1) => Number.isFinite(value) ? value.toFixed(digits) : "—";

function Field({ label, value, unit, min=0, max=2000, step=1, onChange }: {label:string;value:number;unit:string;min?:number;max?:number;step?:number;onChange:(value:number)=>void}) {
  return <label className="block min-w-0"><span className="mb-1.5 block text-[10px] font-semibold uppercase tracking-wider text-zinc-500">{label}</span><span className="flex h-9 items-center rounded-md border border-zinc-200 bg-white"><input className="min-w-0 flex-1 bg-transparent px-2.5 font-mono text-sm outline-none" type="number" min={min} max={max} step={step} value={value} onChange={e=>onChange(Number(e.target.value))}/><span className="px-2 text-[10px] text-zinc-400">{unit}</span></span></label>;
}

function ShelfPreview({ shelf, object, slot, clearance, platform, approach }: {shelf:typeof SHELVES[number];object:Workpiece;slot:number;clearance:number;platform:{x:number;y:number;z:number};approach:number}) {
  const scale = 430 / shelf.width;
  const wall = shelf.board * scale;
  const w = shelf.width * scale;
  const h = shelf.height * scale;
  const floorY = 268;
  const shelfTop = floorY - h;
  const bayH = (h - wall * (shelf.levels + 1)) / shelf.levels;
  const bayW = w - wall * 2;
  const itemW = Math.min(bayW * .64, Math.max(20, object.dimensions.x * scale));
  const itemH = Math.min(bayH * .7, Math.max(18, object.dimensions.z * scale));
  const itemX = 24 + wall + (bayW-itemW)/2;
  const itemY = floorY - wall - slot * (bayH + wall) - itemH;
  const innerX = 24 + wall;
  const fit = object.dimensions.x + 2*clearance <= shelf.width - 2*shelf.board && object.dimensions.z + 2*clearance <= (shelf.height - shelf.board*(shelf.levels+1))/shelf.levels;
  const destX = 510 + Math.min(100, platform.x * .12);
  const destY = floorY - Math.min(105, platform.z * .25);
  return <svg viewBox="0 0 680 310" className="h-full w-full" role="img" aria-label="Side view of generated shelf, selected object and destination platform">
    <defs><pattern id="grid" width="20" height="20" patternUnits="userSpaceOnUse"><path d="M 20 0 L 0 0 0 20" fill="none" stroke="#e4e4e7" strokeWidth=".7"/></pattern></defs>
    <rect width="680" height="310" fill="url(#grid)"/>
    <path d={`M ${innerX} ${shelfTop} V ${floorY} M ${innerX+bayW} ${shelfTop} V ${floorY} M ${innerX} ${shelfTop} H ${innerX+bayW}`} stroke="#52525b" strokeWidth={wall*1.5} fill="none"/>
    {Array.from({length:shelf.levels+1},(_,i)=>{const y=floorY-wall-i*(bayH+wall);return <rect key={i} x={innerX-wall} y={y} width={bayW+wall*2} height={wall} fill="#a1a1aa" stroke="#71717a" strokeWidth="1"/>;})}
    <rect x={itemX} y={itemY} width={itemW} height={itemH} rx={object.shape==="cylinder"?itemW/2:2} fill="#ef4444" stroke="#991b1b" strokeWidth="1.5"/>
    <path d={`M ${itemX+itemW/2} ${itemY-15} V ${itemY-3}`} stroke="#2563eb" strokeWidth="2"/><path d={`M ${itemX+itemW/2-5} ${itemY-9} L ${itemX+itemW/2} ${itemY-3} L ${itemX+itemW/2+5} ${itemY-9}`} fill="none" stroke="#2563eb" strokeWidth="2"/>
    <text x={innerX} y={shelfTop-10} fontSize="10" fill="#52525b">SHELF · {shelf.width} × {shelf.height} × {shelf.depth} mm</text>
    <text x={innerX+bayW/2} y={itemY+itemH+13} fontSize="9" fill="#52525b" textAnchor="middle">Pick slot {slot+1}</text>
    <rect x="495" y={floorY-9} width="150" height="9" fill="#71717a"/><rect x="520" y={destY} width="92" height={Math.max(15,itemH*.72)} rx="3" fill="#0ea5e9" fillOpacity=".85" stroke="#0369a1"/>
    <path d={`M ${itemX+itemW/2} ${itemY-18} C 350 65, 440 80, ${destX+46} ${destY-22}`} fill="none" stroke="#2563eb" strokeWidth="2" strokeDasharray="5 4"/>
    <text x="570" y={floorY+17} textAnchor="middle" fontSize="10" fill="#52525b">PLACE PLATFORM</text>
    <text x="570" y={floorY+30} textAnchor="middle" fontSize="9" fill="#71717a">{platform.x} × {platform.y} mm</text>
    <text x="24" y="298" fontSize="9" fill={fit?"#15803d":"#b91c1c"}>{fit?"DIMENSION CHECK: FITS WITH CURRENT CLEARANCE":"DIMENSION CHECK: OBJECT DOES NOT FIT THIS BAY"}</text>
    <text x="656" y="298" fontSize="9" fill="#71717a" textAnchor="end">Approach {approach}°</text>
  </svg>;
}

export function PickPlacePage() {
  const model = useRobotStore(s=>s.model);
  const bridgeState = useRobotStore(s=>s.bridgeState);
  const simulation = useRobotStore(s=>s.simulation);
  const [source,setSource]=useState<InputSource>("synthetic");
  const [objectKind,setObjectKind]=useState<ObjectKind>("coke");
  const [custom,setCustom]=useState<Workpiece>({kind:"custom",label:"Custom object",shape:"box",dimensions:{x:100,y:80,z:80},massKg:.5,massSource:"User supplied"});
  const [shelfId,setShelfId]=useState<string>("standard");
  const [slot,setSlot]=useState(1);
  const [clearance,setClearance]=useState(12);
  const [gripperAngle,setGripperAngle]=useState(0);
  const [approachAngle,setApproachAngle]=useState(0);
  const [approachDistance,setApproachDistance]=useState(80);
  const [tcpOffset,setTcpOffset]=useState(0);
  const [platform,setPlatform]=useState({x:300,y:240,z:20});
  const [pick,setPick]=useState({x:300,y:0,z:100});
  const [density,setDensity]=useState(750);
  const [planState,setPlanState]=useState<"idle"|"ready"|"blocked">("idle");
  const [visionJson,setVisionJson]=useState('{\n  "frame_id": "dummy_link",\n  "objects": [{ "id": "vision_object_1", "class": "can", "shape": "cylinder", "centroid_m": [0.30, 0.00, 0.10], "dimensions_m": [0.066, 0.066, 0.122], "yaw_deg": 0, "mass_kg": 0.355 }]\n}');
  const shelf=SHELVES.find(item=>item.id===shelfId)??SHELVES[1];
  const object=objectKind==="custom"?custom:OBJECTS[objectKind];
  const massKg=objectKind==="wood_box"?object.dimensions.x*object.dimensions.y*object.dimensions.z/1e9*density:object.massKg;
  const bayWidth=shelf.width-2*shelf.board;
  const bayHeight=(shelf.height-shelf.board*(shelf.levels+1))/shelf.levels;
  const fit=object.dimensions.x+2*clearance<=bayWidth&&object.dimensions.y+2*clearance<=shelf.depth&&object.dimensions.z+2*clearance<=bayHeight;
  const grasp=useMemo(()=>({tcp:{x:pick.x,y:pick.y,z:pick.z+object.dimensions.z/2+tcpOffset},yaw:gripperAngle,approach:approachAngle,distance:approachDistance}),[pick,object,gripperAngle,approachAngle,approachDistance,tcpOffset]);
  const checks=[{label:"Object fits selected shelf bay",ok:fit,detail:`Bay ${bayWidth} × ${shelf.depth} × ${Math.round(bayHeight)} mm`},{label:"Approach clearance",ok:approachDistance>=Math.max(40,object.dimensions.z*.25),detail:`${approachDistance} mm approach distance`},{label:"Gripper width",ok:object.dimensions.x<=100,detail:`${object.dimensions.x} mm object width; PGEA max opening ~40 mm per side pair total stroke 40 mm`},{label:"Robot model available",ok:!!model,detail:model?`${model.group_name} · ${model.base_frame} → ${model.end_effector_link}`:"Waiting for bridge model"}];
  const allValid=checks.every(item=>item.ok);
  const createPlan=()=>{setPlanState(allValid?"ready":"blocked");};
  const parseVision=()=>{try{const data=JSON.parse(visionJson);const first=data.objects?.[0];if(!first)throw new Error("No objects in vision payload");const dims=first.dimensions_m??[.1,.1,.1];const shape=first.shape==="cylinder"?"cylinder":"box";setCustom({kind:"custom",label:first.class??first.id??"Vision object",shape,dimensions:{x:dims[0]*1000,y:dims[1]*1000,z:dims[2]*1000},massKg:Number(first.mass_kg??.5),massSource:"Vision payload / estimated"});setObjectKind("custom");if(Array.isArray(first.centroid_m))setPick({x:first.centroid_m[0]*1000,y:first.centroid_m[1]*1000,z:first.centroid_m[2]*1000});setSource("vision");setPlanState("idle");}catch(error){window.alert(error instanceof Error?error.message:"Invalid vision payload");}};
  const copyGrasp=()=>{void navigator.clipboard?.writeText(JSON.stringify({frame_id:model?.base_frame??"dummy_link",link_name:model?.end_effector_link??"Link6",tcp_target_m:[grasp.tcp.x/1000,grasp.tcp.y/1000,grasp.tcp.z/1000],gripper_yaw_deg:grasp.yaw,approach_angle_deg:grasp.approach,approach_distance_m:grasp.distance/1000,object_mass_kg:massKg,object_dimensions_m:[object.dimensions.x,object.dimensions.y,object.dimensions.z].map(v=>v/1000)},null,2)).catch(()=>{});};
  return <div className="space-y-5">
    <div className="flex flex-wrap items-start justify-between gap-3"><div><div className="eyebrow">MOTION WORKSPACE / TASK PLANNING</div><h1 className="page-title">Pick & place planner</h1><p className="mt-1 max-w-2xl text-sm text-zinc-500">Configure objects, synthesize shelf scenes, derive grasp targets and validate placement clearances before planning.</p></div><div className="flex items-center gap-2"><span className={`rounded-full px-2.5 py-1 text-[10px] font-semibold ${simulation?"bg-amber-50 text-amber-700":"bg-zinc-100 text-zinc-600"}`}>{simulation?"SIMULATION":"PLAN ONLY"}</span><span className="rounded-full bg-zinc-100 px-2.5 py-1 text-[10px] text-zinc-600">{bridgeState}</span></div></div>
    <div className="grid gap-4 xl:grid-cols-[minmax(0,1fr)_340px]">
      <div className="space-y-4 min-w-0">
        <section className="panel overflow-hidden"><div className="panel-header"><div className="flex items-center gap-2"><Layers3 size={16}/><div><div className="text-sm font-semibold">Scene preview</div><div className="text-[11px] text-zinc-500">Generated geometry · side elevation · not a collision validation result</div></div></div><Button size="sm" variant="outline" onClick={()=>{setShelfId("standard");setSlot(1);setPlanState("idle");}}><RotateCcw size={13}/> Reset scene</Button></div><div className="h-[310px] p-3"><ShelfPreview shelf={shelf} object={object} slot={slot} clearance={clearance} platform={platform} approach={approachAngle}/></div><div className="grid grid-cols-2 gap-3 border-t border-zinc-100 px-5 py-3 text-xs sm:grid-cols-4"><div><div className="text-zinc-400">Selected object</div><div className="mt-1 font-medium">{object.label}</div></div><div><div className="text-zinc-400">Estimated payload</div><div className="mt-1 font-mono font-semibold">{number(massKg*1000,0)} g</div></div><div><div className="text-zinc-400">Shelf bay</div><div className="mt-1 font-mono">{Math.round(bayWidth)} × {Math.round(bayHeight)} mm</div></div><div><div className="text-zinc-400">Fit status</div><div className={`mt-1 font-semibold ${fit?"text-emerald-700":"text-red-700"}`}>{fit?"Fits with clearance":"Does not fit"}</div></div></div></section>
        <section className="panel"><div className="panel-header"><div className="flex items-center gap-2"><Package size={16}/><span className="text-sm font-semibold">Object & input source</span></div><div className="flex gap-1 rounded-lg bg-zinc-100 p-1">{(["synthetic","vision","manual"] as const).map(v=><button key={v} onClick={()=>setSource(v)} className={`rounded-md px-2.5 py-1 text-[10px] font-medium capitalize ${source===v?"bg-white shadow-sm":"text-zinc-500"}`}>{v}</button>)}</div></div><div className="space-y-4 p-5"><div className="grid gap-3 sm:grid-cols-2"><label className="block"><span className="mb-1.5 block text-[10px] font-semibold uppercase tracking-wider text-zinc-500">Object preset</span><select value={objectKind} onChange={e=>{setObjectKind(e.target.value as ObjectKind);setPlanState("idle");}} className="h-9 w-full rounded-md border border-zinc-200 bg-white px-2 text-sm"><option value="coke">Coca-Cola can · 330 ml</option><option value="pringles">Pringles tube · 165 g</option><option value="wood_box">Wooden box · density model</option><option value="custom">Custom / vision object</option></select></label><div className="rounded-lg bg-zinc-50 px-3 py-2"><div className="text-[10px] uppercase tracking-wider text-zinc-400">Mass model</div><div className="mt-1 font-mono text-sm font-semibold">{number(massKg,3)} kg</div><div className="text-[10px] text-zinc-500">{object.massSource}</div></div></div>
        {objectKind==="custom"||objectKind==="wood_box"?<div className="grid grid-cols-2 gap-3 sm:grid-cols-4"><Field label="Length" value={object.dimensions.x} unit="mm" onChange={v=>{setCustom(o=>({...o,dimensions:{...o.dimensions,x:v}}));if(objectKind==="wood_box")setObjectKind("custom");}}/><Field label="Width" value={object.dimensions.y} unit="mm" onChange={v=>{setCustom(o=>({...o,dimensions:{...o.dimensions,y:v}}));if(objectKind==="wood_box")setObjectKind("custom");}}/><Field label="Height" value={object.dimensions.z} unit="mm" onChange={v=>{setCustom(o=>({...o,dimensions:{...o.dimensions,z:v}}));if(objectKind==="wood_box")setObjectKind("custom");}}/><Field label="Wood density" value={density} unit="kg/m³" min={100} max={1400} onChange={setDensity}/></div>:<div className="grid grid-cols-3 gap-3"><div className="rounded-md border border-zinc-100 p-3"><div className="text-[10px] text-zinc-400">Diameter / X</div><div className="mt-1 font-mono text-sm">{object.dimensions.x} mm</div></div><div className="rounded-md border border-zinc-100 p-3"><div className="text-[10px] text-zinc-400">Depth / Y</div><div className="mt-1 font-mono text-sm">{object.dimensions.y} mm</div></div><div className="rounded-md border border-zinc-100 p-3"><div className="text-[10px] text-zinc-400">Height / Z</div><div className="mt-1 font-mono text-sm">{object.dimensions.z} mm</div></div></div>}
        {source==="vision"?<div className="space-y-2"><textarea value={visionJson} onChange={e=>setVisionJson(e.target.value)} rows={5} className="w-full rounded-md border border-zinc-200 p-3 font-mono text-xs"/><Button size="sm" onClick={parseVision}><ScanLine size={13}/> Load vision detection</Button><p className="text-[10px] text-zinc-500">Adapter contract: frame_id, objects[].id/class/shape/centroid_m/dimensions_m/yaw_deg/mass_kg. Vision data is parsed into the same normalized workpiece model as synthetic inputs.</p></div>:null}</div></section>
        <section className="panel"><div className="panel-header"><div className="flex items-center gap-2"><Ruler size={16}/><span className="text-sm font-semibold">Shelf & placement geometry</span></div><span className="text-[10px] text-zinc-400">All dimensions in mm</span></div><div className="grid gap-4 p-5 md:grid-cols-2"><div className="space-y-3"><label className="block"><span className="mb-1.5 block text-[10px] font-semibold uppercase tracking-wider text-zinc-500">Shelf profile</span><select value={shelfId} onChange={e=>{setShelfId(e.target.value);setPlanState("idle");}} className="h-9 w-full rounded-md border border-zinc-200 bg-white px-2 text-sm">{SHELVES.map(s=><option key={s.id} value={s.id}>{s.label} · {s.width} × {s.height} × {s.depth}</option>)}</select></label><div className="grid grid-cols-2 gap-3"><Field label="Shelf width" value={shelf.width} unit="mm" onChange={()=>{}}/><Field label="Shelf depth" value={shelf.depth} unit="mm" onChange={()=>{}}/><Field label="Levels" value={shelf.levels} unit="bays" onChange={()=>{}}/><Field label="Board thickness" value={shelf.board} unit="mm" onChange={()=>{}}/></div><label className="block"><span className="mb-1.5 block text-[10px] font-semibold uppercase tracking-wider text-zinc-500">Target shelf level · {slot+1}</span><input className="range-control w-full" type="range" min={0} max={shelf.levels-1} value={slot} onChange={e=>setSlot(Number(e.target.value))}/></label></div><div className="space-y-3"><div className="text-xs font-semibold">Placement platform</div><div className="grid grid-cols-2 gap-3"><Field label="Platform X" value={platform.x} unit="mm" onChange={v=>setPlatform(p=>({...p,x:v}))}/><Field label="Platform Y" value={platform.y} unit="mm" onChange={v=>setPlatform(p=>({...p,y:v}))}/><Field label="Platform Z" value={platform.z} unit="mm" onChange={v=>setPlatform(p=>({...p,z:v}))}/><Field label="Clearance margin" value={clearance} unit="mm" min={0} max={100} onChange={setClearance}/></div><p className="text-[10px] leading-relaxed text-zinc-500">Shelf profile dimensions generate an open-front rack with multiple bays. Platform pose is a separate placement target; the shelf is not assumed to be the place destination.</p></div></div></section>
      </div>
      <aside className="space-y-4"><section className="panel"><div className="panel-header"><div className="flex items-center gap-2"><Target size={16}/><span className="text-sm font-semibold">Grasp generation</span></div><span className="rounded bg-zinc-100 px-2 py-1 text-[9px] font-semibold text-zinc-500">TCP TARGET</span></div><div className="space-y-4 p-5"><div className="text-[10px] uppercase tracking-wider text-zinc-400">Object centroid · base frame</div><div className="grid grid-cols-3 gap-2">{(["x","y","z"] as const).map(axis=><Field key={axis} label={axis.toUpperCase()} value={pick[axis]} unit="mm" onChange={v=>setPick(p=>({...p,[axis]:v}))}/>)}</div><div className="grid grid-cols-2 gap-3"><Field label="Gripper yaw" value={gripperAngle} unit="deg" min={-180} max={180} onChange={setGripperAngle}/><Field label="Approach tilt" value={approachAngle} unit="deg" min={-90} max={90} onChange={setApproachAngle}/><Field label="Approach distance" value={approachDistance} unit="mm" min={20} max={250} onChange={setApproachDistance}/><Field label="TCP Z offset" value={tcpOffset} unit="mm" min={-100} max={100} onChange={setTcpOffset}/></div><div className="rounded-lg bg-zinc-950 p-3 text-white"><div className="text-[9px] uppercase tracking-wider text-zinc-400">Derived grasp TCP · {model?.end_effector_link??"Link6"}</div><div className="mt-2 grid grid-cols-3 gap-2 font-mono text-xs">{Object.entries(grasp.tcp).map(([k,v])=><div key={k}><div className="text-zinc-500">{k.toUpperCase()}</div><div>{number(v)} mm</div></div>)}</div><div className="mt-2 flex justify-between border-t border-zinc-700 pt-2 text-[10px] text-zinc-400"><span>Yaw {grasp.yaw}° · Approach {grasp.approach}°</span><span>{grasp.distance} mm</span></div><Button size="sm" variant="secondary" className="mt-3 w-full" onClick={copyGrasp}><ClipboardCopy size={13}/> Copy grasp target JSON</Button></div></div></section>
      <section className="panel"><div className="panel-header"><div className="flex items-center gap-2"><ShieldCheck size={16}/><span className="text-sm font-semibold">Pre-plan validation</span></div><span className="text-[10px] text-zinc-400">{checks.filter(c=>c.ok).length}/{checks.length}</span></div><div className="space-y-3 p-4">{checks.map(c=><div key={c.label} className="flex items-start gap-2"><span className={`mt-0.5 h-2 w-2 rounded-full ${c.ok?"bg-emerald-500":"bg-red-500"}`}/><div className="min-w-0"><div className="text-xs font-medium">{c.label}</div><div className="text-[10px] text-zinc-500">{c.detail}</div></div></div>)}<div className="rounded-md bg-amber-50 p-3 text-[10px] leading-relaxed text-amber-800">Payload mass is metadata only at this UI stage. It must be applied to the robot's payload / dynamics configuration before execution; MoveIt geometric planning alone does not model gravity, motor torque or load capacity.</div><Button className="w-full" disabled={!allValid||!model} onClick={createPlan}><Target size={14}/> {planState==="ready"?"Checks passed · ready for MoveIt":"Validate task & prepare plan"}</Button>{planState==="blocked"?<p className="text-xs text-red-700">Resolve failed checks before planning.</p>:planState==="ready"?<p className="text-xs text-emerald-700">Geometry checks passed. MoveIt task planning and collision validation are not connected in this UI commit.</p>:null}</div></section>
      <section className="rounded-xl border border-zinc-200 bg-zinc-50 p-4"><div className="text-xs font-semibold">Mass assumptions</div><p className="mt-2 text-[10px] leading-relaxed text-zinc-500">Coke: 355 g full-can estimate. Pringles: 205 g (165 g contents + estimated 40 g packaging). Wood box: volume × editable density, default 750 kg/m³. Replace estimates with scale measurements before robot payload configuration.</p><div className="mt-2 text-[10px] text-zinc-400">PGEA gripper opening is limited to approximately 40 mm total stroke. Cylindrical object diameter presets exceed that range; grasp feasibility requires a validated side-grasp/contact strategy, not a simple width check.</div></section></aside>
    </div>
  </div>;
}
