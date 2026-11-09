import { useEffect, useMemo, useState } from "react";
import { Box, ChevronDown, ClipboardCopy, Layers3, Package, RotateCcw, Ruler, ScanLine, ShieldCheck, Target } from "lucide-react";
import { Button } from "@/components/ui/button";
import { useRobotStore } from "@/store/robot-store";
import { useRobotSocket } from "@/lib/socket";
import { RobotViewport } from "@/components/robot/robot-viewport";
import {
  DEFAULT_VISUALIZER_VIEW_PREFERENCES, loadVisualizerViewPreferences,
  saveVisualizerViewPreferences, type VisualizerViewPreferences,
} from "@/lib/visualizer-view-preferences";

type ObjectKind = "coke" | "pringles" | "wood_box" | "custom";
type InputSource = "synthetic" | "vision" | "manual";
type Dimensions = { x: number; y: number; z: number };
type Workpiece = { kind: ObjectKind; label: string; shape: "cylinder" | "box" | "sphere"; dimensions: Dimensions; massKg: number; massSource: string; densityKgM3?: number };

const OBJECTS: Record<Exclude<ObjectKind,"custom">,Workpiece> = {
  coke: { kind:"coke", label:"Coca-Cola can · 330 ml", shape:"cylinder", dimensions:{x:66,y:66,z:122}, massKg:0.355, massSource:"Estimated filled 330 ml can: ~342 g beverage + ~13 g can" },
  pringles: { kind:"pringles", label:"Pringles tube · 165 g", shape:"cylinder", dimensions:{x:75,y:75,z:240}, massKg:0.215, massSource:"165 g contents + approximately 50 g empty tube / lid" },
  wood_box: { kind:"wood_box", label:"Wooden box · configurable", shape:"box", dimensions:{x:120,y:90,z:80}, massKg:0.5616, massSource:"Estimated from 120×90×80 mm solid volume × 650 kg/m³", densityKgM3:650 },
};

const SHELVES = [
  { id:"compact", label:"Compact", width:420, height:620, depth:300, levels:2, board:18 },
  { id:"standard", label:"Standard", width:600, height:900, depth:400, levels:3, board:18 },
  { id:"wide", label:"Wide", width:900, height:1200, depth:450, levels:4, board:20 },
  { id:"deep", label:"Deep storage", width:700, height:1000, depth:600, levels:3, board:22 },
] as const;

const number = (value: number, digits=1) => Number.isFinite(value) ? value.toFixed(digits) : "—";

function Field({ label, value, unit, min=0, max=2000, step=1, onChange, disabled=false }: {label:string;value:number;unit:string;min?:number;max?:number;step?:number;onChange:(value:number)=>void;disabled?:boolean}) {
  return <label className="block min-w-0"><span className="mb-1.5 block text-[10px] font-semibold uppercase tracking-wider text-zinc-500">{label}</span><span className="flex h-9 items-center rounded-md border border-zinc-200 bg-white"><input className="min-w-0 flex-1 bg-transparent px-2.5 font-mono text-sm outline-none" type="number" min={min} max={max} step={step} value={value} disabled={disabled} onChange={e=>onChange(Number(e.target.value))}/><span className="px-2 text-[10px] text-zinc-400">{unit}</span></span></label>;
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
    <rect x={itemX} y={itemY} width={itemW} height={itemH} rx={object.shape==="cylinder"||object.shape==="sphere"?itemW/2:2} fill="#ef4444" stroke="#991b1b" strokeWidth="1.5"/>
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
  const modelLoading = useRobotStore(s=>s.modelLoading);
  const modelError = useRobotStore(s=>s.modelError);
  const bridgeState = useRobotStore(s=>s.bridgeState);
  const simulation = useRobotStore(s=>s.simulation);
  const joints = useRobotStore(s=>s.joints);
  const scene = useRobotStore(s=>s.scene);
  const sceneLoading = useRobotStore(s=>s.sceneLoading);
  const sceneError = useRobotStore(s=>s.sceneError);
  const plan = useRobotStore(s=>s.plan);
  const [viewPreferences,setViewPreferences]=useState<VisualizerViewPreferences>(loadVisualizerViewPreferences);
  const [source,setSource]=useState<InputSource>("synthetic");
  const [objectKind,setObjectKind]=useState<ObjectKind>("coke");
  const [custom,setCustom]=useState<Workpiece>({kind:"custom",label:"Custom object",shape:"box",dimensions:{x:100,y:80,z:80},massKg:.5,massSource:"User supplied"});
  const [shelfId,setShelfId]=useState<string>("standard");
  const [slot,setSlot]=useState(1);
  const [clearance,setClearance]=useState(12);
  const [gripperAngle,setGripperAngle]=useState(0);
  const [approachAngle,setApproachAngle]=useState(0);
  const [approachDistance,setApproachDistance]=useState(250);
  const [tcpOffset,setTcpOffset]=useState(0);
  const [platform,setPlatform]=useState({x:350,y:-350,z:0});
  const [shelfPose,setShelfPose]=useState({x:550,y:0,z:0,yaw:90});
  const [pick,setPick]=useState({x:0,y:0,z:400});
  const [density,setDensity]=useState(650);
  const [planState,setPlanState]=useState<"idle"|"ready"|"blocked">("idle");
  const [planResult,setPlanResult]=useState<any>(null);
  const [graspStrategy,setGraspStrategy]=useState("pgea_finger_wrap");
  const [toolMassKg,setToolMassKg]=useState(1.6);
  const [toolCom,setToolCom]=useState({x:-86.6,y:0,z:-109.6});
  const [controllerToolVerified,setControllerToolVerified]=useState(false);
  const [payloadBusy,setPayloadBusy]=useState(false);
  const [sceneBusy,setSceneBusy]=useState(false);
  const [payloadStatus,setPayloadStatus]=useState<string|null>(null);
  const [visionFrame,setVisionFrame]=useState<string|null>(null);
  const [objectYaw,setObjectYaw]=useState(90);
  const {request}=useRobotSocket();
  useEffect(()=>saveVisualizerViewPreferences(viewPreferences),[viewPreferences]);
  const [visionJson,setVisionJson]=useState('{\n  "frame_id": "dummy_link",\n  "objects": [{ "id": "vision_object_1", "class": "can", "shape": "cylinder", "centroid_m": [0.30, 0.00, 0.10], "dimensions_m": [0.066, 0.066, 0.122], "yaw_deg": 0, "mass_kg": 0.355 }]\n}');
  const shelf=SHELVES.find(item=>item.id===shelfId)??SHELVES[1];
  const object=objectKind==="custom"?custom:OBJECTS[objectKind];
  const massKg=objectKind==="wood_box"?object.dimensions.x*object.dimensions.y*object.dimensions.z/1e9*density:object.massKg;
  const bayWidth=shelf.width-2*shelf.board;
  const bayHeight=(shelf.height-shelf.board*(shelf.levels+1))/shelf.levels;
  const generatedPick={x:shelfPose.x,y:shelfPose.y,z:shelfPose.z+shelf.board+slot*(bayHeight+shelf.board)+clearance+object.dimensions.z/2};
  const activePick=source==="synthetic"?generatedPick:pick;
  const shelfRelativeObjectYaw=(objectYaw-shelfPose.yaw)*Math.PI/180;
  const projectedShelfWidth=Math.abs(Math.cos(shelfRelativeObjectYaw))*object.dimensions.x+Math.abs(Math.sin(shelfRelativeObjectYaw))*object.dimensions.y;
  const projectedShelfDepth=Math.abs(Math.sin(shelfRelativeObjectYaw))*object.dimensions.x+Math.abs(Math.cos(shelfRelativeObjectYaw))*object.dimensions.y;
  const fit=projectedShelfWidth+2*clearance<=bayWidth&&projectedShelfDepth+2*clearance<=shelf.depth-shelf.board&&object.dimensions.z+2*clearance<=bayHeight;
  const grasp=useMemo(()=>{const yaw=(gripperAngle+objectYaw)*Math.PI/180;const pitch=approachAngle*Math.PI/180;return {tcp:{x:activePick.x+Math.cos(yaw)*Math.sin(pitch)*tcpOffset,y:activePick.y+Math.sin(yaw)*Math.sin(pitch)*tcpOffset,z:activePick.z+Math.cos(pitch)*tcpOffset},yaw:gripperAngle+objectYaw,approach:approachAngle,distance:approachDistance};},[activePick,object,gripperAngle,objectYaw,approachAngle,approachDistance,tcpOffset]);
  const yawRad=(gripperAngle+objectYaw)*Math.PI/180;
  const relativeJawYawRad=gripperAngle*Math.PI/180;
  const objectWidthMm=Math.abs(Math.cos(relativeJawYawRad))*object.dimensions.x+Math.abs(Math.sin(relativeJawYawRad))*object.dimensions.y;
  const gripperEnvelopeMm=graspStrategy==="pgea_finger_wrap"?85:40;
  const gripperWidthFeasible=objectWidthMm+6<=gripperEnvelopeMm;
  const totalPayloadKg=toolMassKg+massKg;
  const approachRad=approachAngle*Math.PI/180;
  const objectComOffsetMm={
    x:-Math.cos(yawRad)*Math.sin(approachRad)*tcpOffset,
    y:-Math.sin(yawRad)*Math.sin(approachRad)*tcpOffset,
    z:-Math.cos(approachRad)*tcpOffset,
  };
  const payloadCom={
    x:(toolMassKg*toolCom.x+massKg*objectComOffsetMm.x)/totalPayloadKg,
    y:(toolMassKg*toolCom.y+massKg*objectComOffsetMm.y)/totalPayloadKg,
    z:(toolMassKg*toolCom.z+massKg*objectComOffsetMm.z)/totalPayloadKg,
  };
  const visionFrameValid=source!=="vision"||(!!visionFrame&&visionFrame===model?.base_frame);
  const checks=[{label:"Object fits selected shelf bay",ok:fit,detail:`Bay ${bayWidth} × ${shelf.depth} × ${Math.round(bayHeight)} mm`},{label:"Approach clearance",ok:approachDistance>=Math.max(40,object.dimensions.z*.25),detail:`${approachDistance} mm approach distance`},{label:"Gripper geometry",ok:gripperWidthFeasible,detail:`${objectWidthMm} mm object + 6 mm clearance vs ${gripperEnvelopeMm} mm ${graspStrategy==="pgea_finger_wrap"?"extended-finger wrap envelope (CAD estimate)":"nominal jaw travel"}`},{label:"Vision frame aligned",ok:visionFrameValid,detail:source!=="vision"?"Synthetic / manual input":`Detection frame ${visionFrame??"missing"} · robot base ${model?.base_frame??"unknown"}`},{label:"Robot model available",ok:!!model,detail:model?`${model.group_name} · ${model.base_frame} → tcp`:"Waiting for bridge model"}];
  const allValid=checks.every(item=>item.ok);
  const prepareScene=async()=>{
    if(!model||!allValid||bridgeState!=="connected"||sceneBusy)return;
    const kind=objectKind==="coke"?"coca_cola_330ml":objectKind==="pringles"?"pringles_165g":objectKind==="wood_box"?"wood_box":"custom";
    const objectPayload:Record<string,unknown>={
      id:"pick_place_object",kind,label:object.label,shape:object.shape,
      dimensions_m:[object.dimensions.x,object.dimensions.y,object.dimensions.z].map(v=>v/1000),
      centroid_m:[activePick.x,activePick.y,activePick.z].map(v=>v/1000),
      rpy_rad:[0,0,objectYaw*Math.PI/180],density_kg_m3:density,
    };
    if(objectKind!=="wood_box")objectPayload.mass_kg=massKg;
    setSceneBusy(true);setPayloadStatus(null);
    try{
      const result=await request("pick_place.prepare",{
        frame_id:source==="vision"?visionFrame:model.base_frame,group_name:model.group_name,object:objectPayload,
        shelf:{width_m:shelf.width/1000,height_m:shelf.height/1000,depth_m:shelf.depth/1000,levels:shelf.levels,board_m:shelf.board/1000},
        shelf_pose:{position_m:[shelfPose.x,shelfPose.y,shelfPose.z].map(v=>v/1000),yaw_rad:shelfPose.yaw*Math.PI/180},
        level:slot,clearance_m:clearance/1000,
        platform:{id:"place_platform",position_m:[platform.x,platform.y,platform.z].map(v=>v/1000),dimensions_m:[0.3,0.24,0.02]},
        gripper_yaw_deg:gripperAngle,approach_angle_deg:approachAngle,grasp_strategy:graspStrategy,gripper_clearance_m:0.006,
        approach_distance_m:approachDistance/1000,tcp_offset_m:[0,0,tcpOffset/1000],
      });
      setPlanResult(result);setPayloadStatus(`Scene prepared · ${result.scene_object_ids?.length??0} collision objects · ${result.frame_id}`);
    }catch(error){setPayloadStatus(error instanceof Error?error.message:String(error));}
    finally{setSceneBusy(false);}
  };
  const runPlan=async()=>{
    if(!model||!allValid)return;
    setPlanState("idle");setPlanResult({status:"PLANNING"});
    const kind=objectKind==="coke"?"coca_cola_330ml":objectKind==="pringles"?"pringles_165g":objectKind==="wood_box"?"wood_box":"custom";
    const objectPayload:Record<string,unknown>={
      id:"pick_place_object",kind,label:object.label,shape:object.shape,
      dimensions_m:[object.dimensions.x,object.dimensions.y,object.dimensions.z].map(v=>v/1000),
      centroid_m:[activePick.x,activePick.y,activePick.z].map(v=>v/1000),rpy_rad:[0,0,objectYaw*Math.PI/180],
      density_kg_m3:density,
    };
    if(objectKind!=="wood_box")objectPayload.mass_kg=massKg;
    try{
      const taskPayload={
        frame_id:source==="vision"?visionFrame:model.base_frame,group_name:model.group_name,object:objectPayload,
        shelf:{width_m:shelf.width/1000,height_m:shelf.height/1000,depth_m:shelf.depth/1000,levels:shelf.levels,board_m:shelf.board/1000},
        shelf_pose:{position_m:[shelfPose.x,shelfPose.y,shelfPose.z].map(v=>v/1000),yaw_rad:shelfPose.yaw*Math.PI/180},
        level:slot,clearance_m:clearance/1000,
        platform:{id:"place_platform",position_m:[platform.x,platform.y,platform.z].map(v=>v/1000),dimensions_m:[0.3,0.24,0.02]},
        gripper_yaw_deg:gripperAngle,approach_angle_deg:approachAngle,grasp_strategy:graspStrategy,gripper_clearance_m:0.006,
        approach_distance_m:approachDistance/1000,tcp_offset_m:[0,0,tcpOffset/1000],
        lift_distance_m:0.1,allowed_planning_time:5,max_velocity_scaling:0.15,max_acceleration_scaling:0.15,
        tool_mass_kg:toolMassKg,tool_com_m:[toolCom.x/1000,toolCom.y/1000,toolCom.z/1000],object_com_m:[objectComOffsetMm.x/1000,objectComOffsetMm.y/1000,objectComOffsetMm.z/1000],max_payload_kg:5,
      };
      const result=await request("pick_place.plan",taskPayload);
      setPlanResult(result);setPlanState(result.status==="PLANNED"?"ready":"blocked");
    }catch(error){setPlanResult({status:"FAILED",error:error instanceof Error?error.message:String(error)});setPlanState("blocked");}
  };
  const parseVision=()=>{try{const data=JSON.parse(visionJson);const first=data.objects?.[0];if(!first)throw new Error("No objects in vision payload");if(typeof data.frame_id!=="string"||!data.frame_id)throw new Error("Vision payload must include frame_id");const dims=first.dimensions_m??[.1,.1,.1];const shape=first.shape==="cylinder"?"cylinder":first.shape==="sphere"?"sphere":"box";setCustom({kind:"custom",label:first.class??first.id??"Vision object",shape,dimensions:{x:dims[0]*1000,y:dims[1]*1000,z:dims[2]*1000},massKg:Number(first.mass_kg??.5),massSource:"Vision payload / estimated"});const detectedClass=String(first.class??first.class_name??first.kind??"").toLowerCase();const preset=detectedClass.includes("pringles")?"pringles":detectedClass.includes("coke")||detectedClass.includes("cola")?"coke":detectedClass.includes("wood")?"wood_box":"custom";setObjectKind(preset);setGraspStrategy(preset==="coke"||preset==="pringles"?"pgea_finger_wrap":"parallel_jaw");if(Array.isArray(first.centroid_m))setPick({x:first.centroid_m[0]*1000,y:first.centroid_m[1]*1000,z:first.centroid_m[2]*1000});setObjectYaw(Number(first.yaw_deg??((first.rpy_rad?.[2]??0)*180/Math.PI)));setVisionFrame(data.frame_id);setSource("vision");setPlanState("idle");}catch(error){window.alert(error instanceof Error?error.message:"Invalid vision payload");}};
  const copyGrasp=()=>{void navigator.clipboard?.writeText(JSON.stringify({frame_id:model?.base_frame??"dummy_link",link_name:"tcp",tcp_target_m:[grasp.tcp.x/1000,grasp.tcp.y/1000,grasp.tcp.z/1000],gripper_yaw_deg:grasp.yaw,approach_angle_deg:grasp.approach,approach_distance_m:grasp.distance/1000,object_yaw_deg:objectYaw,object_mass_kg:massKg,object_dimensions_m:[object.dimensions.x,object.dimensions.y,object.dimensions.z].map(v=>v/1000)},null,2)).catch(()=>{});};
  const applyPayload=async()=>{
    if(bridgeState!=="connected"||!controllerToolVerified||payloadBusy||totalPayloadKg>5)return;
    setPayloadBusy(true);setPayloadStatus(null);
    try{
      const result=await request("robot.command",{command:"set-payload",payload:{load:Number(totalPayloadKg.toFixed(4)),x:Number(payloadCom.x.toFixed(2)),y:Number(payloadCom.y.toFixed(2)),z:Number(payloadCom.z.toFixed(2))}});
      setPayloadStatus(result?.res===0?`SetPayload accepted · ${JSON.stringify(result)}`:`Controller rejected SetPayload · ${JSON.stringify(result)}`);
    }catch(error){setPayloadStatus(error instanceof Error?error.message:String(error));}
    finally{setPayloadBusy(false);}
  };
  return <div className="space-y-5">
    <div className="flex flex-wrap items-start justify-between gap-3"><div><div className="eyebrow">MOTION WORKSPACE / TASK PLANNING</div><h1 className="page-title">Pick & place planner</h1><p className="mt-1 max-w-2xl text-sm text-zinc-500">Configure objects, synthesize shelf scenes, derive grasp targets and validate placement clearances before planning.</p></div><div className="flex items-center gap-2"><span className={`rounded-full px-2.5 py-1 text-[10px] font-semibold ${simulation?"bg-amber-50 text-amber-700":"bg-zinc-100 text-zinc-600"}`}>{simulation?"SIMULATION":"PLAN ONLY"}</span><span className="rounded-full bg-zinc-100 px-2.5 py-1 text-[10px] text-zinc-600">{bridgeState}</span></div></div>
    <section className="panel overflow-hidden">
      <div className="panel-header"><div><div className="eyebrow">3D / MOVEIT PLANNING SCENE</div><h2 className="mt-1 text-sm font-semibold">Robot, shelf and workpiece</h2></div><div className="text-[10px] text-zinc-400">{scene?.world?.collision_objects?.length??0} world · {scene?.robot_state?.attached_collision_objects?.length??0} attached</div></div>
      <RobotViewport model={model} jointState={joints} scene={scene} plan={plan} targetTcp={{x:grasp.tcp.x,y:grasp.tcp.y,z:grasp.tcp.z,rx:0,ry:approachAngle,rz:grasp.yaw}}
        loading={modelLoading||sceneLoading} error={modelError??sceneError}
        viewPreferences={viewPreferences} onViewPreferencesChange={setViewPreferences}
        objectLabels={viewPreferences.objectLabels} disabledObjects={{}} hiddenObjects={viewPreferences.hiddenObjects}/>
      <div className="flex flex-wrap items-center justify-between gap-2 border-t border-zinc-100 px-4 py-2 text-[10px] text-zinc-500">
        <span>MoveIt scene data · {model?.base_frame??"waiting for base frame"}</span>
        <Button size="sm" variant="outline" disabled={!model||!allValid||sceneBusy||bridgeState!=="connected"} onClick={prepareScene}><Layers3 size={13}/>{sceneBusy?"Preparing…":"Prepare / refresh collision scene"}</Button>
      </div>
    </section>
    <div className="grid gap-4 xl:grid-cols-[minmax(0,1fr)_340px]">
      <div className="space-y-4 min-w-0">
        <section className="panel overflow-hidden"><div className="panel-header"><div className="flex items-center gap-2"><Layers3 size={16}/><div><div className="text-sm font-semibold">Scene preview</div><div className="text-[11px] text-zinc-500">Generated geometry · side elevation · not a collision validation result</div></div></div><Button size="sm" variant="outline" onClick={()=>{setShelfId("standard");setSlot(1);setPlanState("idle");}}><RotateCcw size={13}/> Reset scene</Button></div><div className="h-[310px] p-3"><ShelfPreview shelf={shelf} object={object} slot={slot} clearance={clearance} platform={platform} approach={approachAngle}/></div><div className="grid grid-cols-2 gap-3 border-t border-zinc-100 px-5 py-3 text-xs sm:grid-cols-4"><div><div className="text-zinc-400">Selected object</div><div className="mt-1 font-medium">{object.label}</div></div><div><div className="text-zinc-400">Estimated payload</div><div className="mt-1 font-mono font-semibold">{number(massKg*1000,0)} g</div></div><div><div className="text-zinc-400">Shelf bay</div><div className="mt-1 font-mono">{Math.round(bayWidth)} × {Math.round(bayHeight)} mm</div></div><div><div className="text-zinc-400">Fit status</div><div className={`mt-1 font-semibold ${fit?"text-emerald-700":"text-red-700"}`}>{fit?"Fits with clearance":"Does not fit"}</div></div></div></section>
        <section className="panel"><div className="panel-header"><div className="flex items-center gap-2"><Package size={16}/><span className="text-sm font-semibold">Object & input source</span></div><div className="flex gap-1 rounded-lg bg-zinc-100 p-1">{(["synthetic","vision","manual"] as const).map(v=><button key={v} onClick={()=>{if(v==="manual"&&source==="synthetic")setPick(activePick);setSource(v);}} className={`rounded-md px-2.5 py-1 text-[10px] font-medium capitalize ${source===v?"bg-white shadow-sm":"text-zinc-500"}`}>{v}</button>)}</div></div><div className="space-y-4 p-5"><div className="grid gap-3 sm:grid-cols-2"><label className="block"><span className="mb-1.5 block text-[10px] font-semibold uppercase tracking-wider text-zinc-500">Object preset</span><select value={objectKind} onChange={e=>{const next=e.target.value as ObjectKind;setObjectKind(next);setGraspStrategy(next==="coke"||next==="pringles"?"pgea_finger_wrap":"parallel_jaw");setPlanState("idle");}} className="h-9 w-full rounded-md border border-zinc-200 bg-white px-2 text-sm"><option value="coke">Coca-Cola can · 330 ml</option><option value="pringles">Pringles tube · 165 g</option><option value="wood_box">Wooden box · density model</option><option value="custom">Custom / vision object</option></select></label><div className="rounded-lg bg-zinc-50 px-3 py-2"><div className="text-[10px] uppercase tracking-wider text-zinc-400">Mass model</div><div className="mt-1 font-mono text-sm font-semibold">{number(massKg,3)} kg</div><div className="text-[10px] text-zinc-500">{object.massSource}</div></div></div>
        {objectKind==="custom"||objectKind==="wood_box"?<div className="grid grid-cols-2 gap-3 sm:grid-cols-4"><Field label="Length" value={object.dimensions.x} unit="mm" onChange={v=>{setCustom(o=>({...o,dimensions:{...o.dimensions,x:v},massKg:v*o.dimensions.y*o.dimensions.z/1e9*density}));if(objectKind==="wood_box")setObjectKind("custom");}}/><Field label="Width" value={object.dimensions.y} unit="mm" onChange={v=>{setCustom(o=>({...o,dimensions:{...o.dimensions,y:v},massKg:o.dimensions.x*v*o.dimensions.z/1e9*density}));if(objectKind==="wood_box")setObjectKind("custom");}}/><Field label="Height" value={object.dimensions.z} unit="mm" onChange={v=>{setCustom(o=>({...o,dimensions:{...o.dimensions,z:v},massKg:o.dimensions.x*o.dimensions.y*v/1e9*density}));if(objectKind==="wood_box")setObjectKind("custom");}}/><Field label="Wood density" value={density} unit="kg/m³" min={100} max={1400} onChange={v=>{setDensity(v);setCustom(o=>({...o,massKg:o.dimensions.x*o.dimensions.y*o.dimensions.z/1e9*v}));}}/><Field label="Custom mass" value={custom.massKg} unit="kg" step={0.001} min={0.001} max={100} onChange={v=>setCustom(o=>({...o,massKg:v,massSource:"User supplied mass"}))}/><label className="block"><span className="mb-1.5 block text-[10px] font-semibold uppercase tracking-wider text-zinc-500">Geometry</span><select value={custom.shape} onChange={e=>setCustom(o=>({...o,shape:e.target.value as Workpiece["shape"]}))} className="h-9 w-full rounded-md border border-zinc-200 bg-white px-2 text-sm"><option value="box">Box</option><option value="cylinder">Cylinder</option><option value="sphere">Sphere</option></select></label></div>:<div className="grid grid-cols-3 gap-3"><div className="rounded-md border border-zinc-100 p-3"><div className="text-[10px] text-zinc-400">Diameter / X</div><div className="mt-1 font-mono text-sm">{object.dimensions.x} mm</div></div><div className="rounded-md border border-zinc-100 p-3"><div className="text-[10px] text-zinc-400">Depth / Y</div><div className="mt-1 font-mono text-sm">{object.dimensions.y} mm</div></div><div className="rounded-md border border-zinc-100 p-3"><div className="text-[10px] text-zinc-400">Height / Z</div><div className="mt-1 font-mono text-sm">{object.dimensions.z} mm</div></div></div>}
        {source==="vision"?<div className="space-y-2"><textarea value={visionJson} onChange={e=>setVisionJson(e.target.value)} rows={5} className="w-full rounded-md border border-zinc-200 p-3 font-mono text-xs"/><Button size="sm" onClick={parseVision}><ScanLine size={13}/> Load vision detection</Button><p className="text-[10px] text-zinc-500">Adapter contract: frame_id, objects[].id/class/shape/centroid_m/dimensions_m/yaw_deg/mass_kg. Vision data is parsed into the same normalized workpiece model as synthetic inputs.</p></div>:null}</div></section>
        <section className="panel"><div className="panel-header"><div className="flex items-center gap-2"><Ruler size={16}/><span className="text-sm font-semibold">Shelf & placement geometry</span></div><span className="text-[10px] text-zinc-400">All dimensions in mm</span></div><div className="grid gap-4 p-5 md:grid-cols-2"><div className="space-y-3"><label className="block"><span className="mb-1.5 block text-[10px] font-semibold uppercase tracking-wider text-zinc-500">Shelf profile</span><select value={shelfId} onChange={e=>{setShelfId(e.target.value);setPlanState("idle");}} className="h-9 w-full rounded-md border border-zinc-200 bg-white px-2 text-sm">{SHELVES.map(s=><option key={s.id} value={s.id}>{s.label} · {s.width} × {s.height} × {s.depth}</option>)}</select></label><div className="grid grid-cols-2 gap-3"><Field label="Shelf X" value={shelfPose.x} unit="mm" min={-2000} max={2000} onChange={v=>setShelfPose(p=>({...p,x:v}))}/><Field label="Shelf Y" value={shelfPose.y} unit="mm" min={-2000} max={2000} onChange={v=>setShelfPose(p=>({...p,y:v}))}/><Field label="Shelf Z" value={shelfPose.z} unit="mm" min={-500} max={2000} onChange={v=>setShelfPose(p=>({...p,z:v}))}/><Field label="Shelf yaw" value={shelfPose.yaw} unit="deg" min={-180} max={180} onChange={v=>setShelfPose(p=>({...p,yaw:v}))}/><Field label="Shelf width" value={shelf.width} unit="mm" onChange={()=>{}} disabled/><Field label="Shelf depth" value={shelf.depth} unit="mm" onChange={()=>{}} disabled/><Field label="Levels" value={shelf.levels} unit="bays" onChange={()=>{}} disabled/><Field label="Board thickness" value={shelf.board} unit="mm" onChange={()=>{}} disabled/></div><label className="block"><span className="mb-1.5 block text-[10px] font-semibold uppercase tracking-wider text-zinc-500">Target shelf level · {slot+1}</span><input className="range-control w-full" type="range" min={0} max={shelf.levels-1} value={slot} onChange={e=>setSlot(Number(e.target.value))}/></label></div><div className="space-y-3"><div className="text-xs font-semibold">Placement platform</div><div className="grid grid-cols-2 gap-3"><Field label="Platform X" value={platform.x} unit="mm" onChange={v=>setPlatform(p=>({...p,x:v}))}/><Field label="Platform Y" value={platform.y} unit="mm" min={-2000} max={2000} onChange={v=>setPlatform(p=>({...p,y:v}))}/><Field label="Platform Z" value={platform.z} unit="mm" onChange={v=>setPlatform(p=>({...p,z:v}))}/><Field label="Clearance margin" value={clearance} unit="mm" min={0} max={100} onChange={setClearance}/></div><p className="text-[10px] leading-relaxed text-zinc-500">Shelf profile dimensions generate an open-front rack with multiple bays. Platform pose is a separate placement target; the shelf is not assumed to be the place destination.</p></div></div></section>
      </div>
      <aside className="space-y-4"><section className="panel"><div className="panel-header"><div className="flex items-center gap-2"><Target size={16}/><span className="text-sm font-semibold">Grasp generation</span></div><span className="rounded bg-zinc-100 px-2 py-1 text-[9px] font-semibold text-zinc-500">TCP TARGET</span></div><div className="space-y-4 p-5"><div className="text-[10px] uppercase tracking-wider text-zinc-400">Object centroid · base frame</div><div className="grid grid-cols-3 gap-2">{(["x","y","z"] as const).map(axis=><Field key={axis} label={axis.toUpperCase()} value={activePick[axis]} unit="mm" onChange={v=>{setPick(p=>({...p,[axis]:v}));setSource("manual");}}/>)}</div><Field label="Object yaw" value={objectYaw} unit="deg" min={-180} max={180} onChange={setObjectYaw}/><label className="block"><span className="mb-1.5 block text-[10px] font-semibold uppercase tracking-wider text-zinc-500">Grasp strategy</span><select value={graspStrategy} onChange={e=>setGraspStrategy(e.target.value)} className="h-9 w-full rounded-md border border-zinc-200 bg-white px-2 text-sm"><option value="pgea_finger_wrap">PGEA extended-finger wrap (Coke / Pringles)</option><option value="parallel_jaw">Parallel-jaw contact</option></select></label><div className="grid grid-cols-2 gap-3"><Field label="Gripper yaw" value={gripperAngle} unit="deg" min={-180} max={180} onChange={setGripperAngle}/><Field label="Approach tilt" value={approachAngle} unit="deg" min={-90} max={90} onChange={setApproachAngle}/><Field label="Approach distance" value={approachDistance} unit="mm" min={20} max={250} onChange={setApproachDistance}/><Field label="TCP Z offset" value={tcpOffset} unit="mm" min={-100} max={100} onChange={setTcpOffset}/></div><div className="rounded-lg bg-zinc-950 p-3 text-white"><div className="text-[9px] uppercase tracking-wider text-zinc-400">Derived grasp TCP · tcp</div><div className="mt-2 grid grid-cols-3 gap-2 font-mono text-xs">{Object.entries(grasp.tcp).map(([k,v])=><div key={k}><div className="text-zinc-500">{k.toUpperCase()}</div><div>{number(v)} mm</div></div>)}</div><div className="mt-2 flex justify-between border-t border-zinc-700 pt-2 text-[10px] text-zinc-400"><span>Yaw {grasp.yaw}° · Approach {grasp.approach}°</span><span>{grasp.distance} mm</span></div><Button size="sm" variant="secondary" className="mt-3 w-full" onClick={copyGrasp}><ClipboardCopy size={13}/> Copy grasp target JSON</Button></div></div></section>
      <section className="panel"><div className="panel-header"><div className="flex items-center gap-2"><ShieldCheck size={16}/><span className="text-sm font-semibold">Pre-plan validation</span></div><span className="text-[10px] text-zinc-400">{checks.filter(c=>c.ok).length}/{checks.length}</span></div><div className="space-y-3 p-4">{checks.map(c=><div key={c.label} className="flex items-start gap-2"><span className={`mt-0.5 h-2 w-2 rounded-full ${c.ok?"bg-emerald-500":c.label==="Gripper width"?"bg-amber-500":"bg-red-500"}`}/><div className="min-w-0"><div className="text-xs font-medium">{c.label}</div><div className="text-[10px] text-zinc-500">{c.detail}</div></div></div>)}<div className="rounded-md bg-amber-50 p-3 text-[10px] leading-relaxed text-amber-800">The PGEA's 40 mm figure is actuator travel, not the entire custom finger contact envelope. The CAD finger meshes add a 45 mm lateral span, giving an estimated 85 mm wrap envelope. Coke (66 mm) and Pringles (75 mm) fit this preliminary geometry envelope. MoveIt plans collision-free robot paths but does not prove contact force, friction or retention; validate contact in simulation and with a guarded physical test.</div><Button className="w-full" disabled={!allValid||!model||bridgeState!=="connected"||planResult?.status==="PLANNING"} onClick={runPlan}><Target size={14}/> {planResult?.status==="PLANNING"?"Planning task…":planState==="ready"?"Re-plan task":"Generate MoveIt pick & place plan"}</Button>
        {planResult?<div className="space-y-2 rounded-md border border-zinc-200 p-3"><div className="flex items-center justify-between"><span className="text-[10px] font-semibold uppercase tracking-wider">MoveIt task result</span><span className={`rounded px-2 py-1 text-[9px] font-semibold ${planResult.status==="PLANNED"?"bg-emerald-50 text-emerald-700":planResult.status==="PLANNING"?"bg-blue-50 text-blue-700":"bg-red-50 text-red-700"}`}>{planResult.status}</span></div>{planResult.error?<p className="text-[10px] text-red-700">{planResult.error}</p>:null}{planResult.failed_stage?<p className="text-[10px] text-red-700">Failed at: {planResult.failed_stage} · MoveIt error {planResult.error_code??"—"}</p>:null}{Array.isArray(planResult.plans)?planResult.plans.map((stage:any)=><div key={stage.name} className="flex justify-between text-[10px]"><span className="capitalize">{stage.name}</span><span className={stage.status==="SUCCEEDED"?"text-emerald-700":"text-red-700"}>{stage.status} {stage.planning_time?`· ${number(stage.planning_time,2)} s`:""}</span></div>):null}{planResult.planning_time_total!==undefined?<div className="border-t border-zinc-100 pt-2 text-[10px]">Total planning time <b>{number(planResult.planning_time_total,2)} s</b></div>:null}{planResult.payload_model?<div className="space-y-1 border-t border-zinc-100 pt-2 text-[10px]"><div className="font-semibold">Payload-aware MoveIt timing</div><div className="flex justify-between"><span>Combined payload</span><b>{number(planResult.payload_model.total_mass_kg,3)} kg</b></div><div className="flex justify-between"><span>Velocity scaling</span><b>{number(planResult.payload_model.max_velocity_scaling*100,1)}%</b></div><div className="flex justify-between"><span>Acceleration scaling</span><b>{number(planResult.payload_model.max_acceleration_scaling*100,1)}%</b></div><div className="text-zinc-400">{planResult.payload_model.heuristic}</div></div>:null}</div>:null}</div></section>
      <section className="panel"><div className="panel-header"><div><div className="eyebrow">PAYLOAD / DOBOT CONTROLLER</div><div className="mt-1 text-sm font-semibold">Controller load configuration</div></div><span className="rounded bg-zinc-100 px-2 py-1 text-[9px] text-zinc-500">EXPLICIT ACTION</span></div><div className="space-y-3 p-4"><div className="grid grid-cols-2 gap-3"><Field label="URDF tool mass" value={toolMassKg} unit="kg" step={0.01} min={0.1} max={4} onChange={setToolMassKg}/><div><div className="text-[10px] font-semibold uppercase tracking-wider text-zinc-500">Total controller load</div><div className="mt-2 font-mono text-lg font-semibold">{number(totalPayloadKg,3)} kg</div><div className="text-[10px] text-zinc-500">Tool + object</div></div></div><div className="text-[10px] leading-relaxed text-zinc-500">URDF tool assembly: 0.30 kg extension plate + 0.40 kg connector + 0.60 kg gripper body + 0.15 kg per finger = 1.60 kg. Manufacturer lists the complete PGEA gripper at 0.60 kg, so verify whether the URDF finger masses are additional before applying a physical payload.</div><div className="text-[10px] font-semibold uppercase tracking-wider text-zinc-500">Combined centre of mass · controller Tool0 (mm)</div><div className="grid grid-cols-3 gap-2"><Field label="X" value={payloadCom.x} unit="mm" step={0.1} min={-500} max={500} onChange={v=>setToolCom(c=>({...c,x:v*totalPayloadKg/toolMassKg}))}/><Field label="Y" value={payloadCom.y} unit="mm" step={0.1} min={-500} max={500} onChange={v=>setToolCom(c=>({...c,y:v*totalPayloadKg/toolMassKg}))}/><Field label="Z" value={payloadCom.z} unit="mm" step={0.1} min={-500} max={500} onChange={v=>setToolCom(c=>({...c,z:(v*totalPayloadKg+massKg*tcpOffset)/toolMassKg}))}/></div><p className="text-[10px] leading-relaxed text-zinc-500">Initial tool COM estimate from the current PGEA URDF inertial origins, transformed into the `tcp` frame: X −113.8, Y 17.8, Z −109.6 mm. The object COM is assumed to coincide with the centroid TCP. These offsets are valid only if the controller's default Tool0 matches this URDF `tcp` frame.</p><label className="flex items-start gap-2 text-[10px] text-amber-800"><input type="checkbox" checked={controllerToolVerified} onChange={e=>setControllerToolVerified(e.target.checked)} className="mt-0.5"/><span>I verified the controller Tool0 frame and the total tool mass against the mounted hardware.</span></label><Button size="sm" className="w-full" disabled={bridgeState!=="connected"||!controllerToolVerified||payloadBusy||totalPayloadKg>5} onClick={applyPayload}><ShieldCheck size={13}/>{payloadBusy?"Applying controller payload…":"Apply payload with SetPayload"}</Button>{totalPayloadKg>5?<p className="text-[10px] text-red-700">Total payload exceeds the configured CR5 5 kg limit.</p>:null}{payloadStatus?<p className="break-words text-[10px] text-zinc-600">{payloadStatus}</p>:null}<p className="text-[10px] text-zinc-400">This is separate from MoveIt planning. The bridge rejects the command unless robot.allow_commands is enabled; it does not run automatically during planning.</p></div></section></aside>
    </div>
  </div>;
}
