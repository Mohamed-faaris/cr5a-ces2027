import { useEffect, useMemo, useState, type ReactNode } from "react";
import { useLocation, useNavigate } from "react-router-dom";
import { ChevronDown, ClipboardCopy, Layers3, ScanLine, ShieldCheck, Target } from "lucide-react";
import { Button } from "@/components/ui/button";
import { useRobotStore } from "@/store/robot-store";
import { useRobotSocket } from "@/lib/socket";
import { RobotViewport } from "@/components/robot/robot-viewport";
import {
  loadVisualizerViewPreferences,
  saveVisualizerViewPreferences, type VisualizerViewPreferences,
} from "@/lib/visualizer-view-preferences";

type WorkflowMode = "platform_to_platform" | "shelf_to_platform";
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

function ConfigTable({title,description,children}:{title:string;description?:string;children:ReactNode}) {
  return <section className="overflow-hidden rounded-lg border border-zinc-200 bg-white">
    <div className="border-b border-zinc-100 px-3 py-2"><h3 className="text-xs font-semibold text-zinc-800">{title}</h3>{description&&<p className="mt-0.5 text-[10px] text-zinc-500">{description}</p>}</div>
    <div className="overflow-x-auto"><table className="w-full min-w-[360px] text-left text-xs"><tbody className="divide-y divide-zinc-100">{children}</tbody></table></div>
  </section>;
}
function ConfigNumberRow({label,value,unit,min=-2000,max=2000,step=1,onChange,hint}:{label:string;value:number;unit:string;min?:number;max?:number;step?:number;onChange:(value:number)=>void;hint?:string}) {
  return <tr><th scope="row" className="w-[42%] px-3 py-2 align-middle font-medium text-zinc-700">{label}{hint&&<div className="mt-0.5 text-[10px] font-normal text-zinc-400">{hint}</div>}</th><td className="px-2 py-1.5"><input aria-label={label} type="number" min={min} max={max} step={step} value={Number.isFinite(value)?value:0} onChange={event=>onChange(Number(event.currentTarget.value))} className="h-8 w-full min-w-0 rounded-md border border-zinc-200 bg-white px-2 font-mono text-xs outline-none focus:border-zinc-500"/></td><td className="w-16 px-3 py-1.5 text-right text-[10px] text-zinc-400">{unit}</td></tr>;
}
function ConfigSelectRow({label,value,options,onChange,hint}:{label:string;value:string;options:{value:string;label:string}[];onChange:(value:string)=>void;hint?:string}) {
  return <tr><th scope="row" className="w-[42%] px-3 py-2 align-middle font-medium text-zinc-700">{label}{hint&&<div className="mt-0.5 text-[10px] font-normal text-zinc-400">{hint}</div>}</th><td colSpan={2} className="px-2 py-1.5"><select aria-label={label} value={value} onChange={event=>onChange(event.currentTarget.value)} className="h-8 w-full rounded-md border border-zinc-200 bg-white px-2 text-xs outline-none focus:border-zinc-500">{options.map(option=><option key={option.value} value={option.value}>{option.label}</option>)}</select></td></tr>;
}

function ShelfPreview({ shelf, object, slot, placeSlot, placeDestination, clearance, platform, approach, objectYaw, shelfYaw, fitRequired }: {shelf:{id:string;label:string;width:number;height:number;depth:number;levels:number;board:number};object:Workpiece;slot:number;placeSlot:number;placeDestination:"platform"|"shelf";clearance:number;platform:{x:number;y:number;z:number};approach:number;objectYaw:number;shelfYaw:number;fitRequired:boolean}) {
  const scale = Math.min(430 / shelf.width, 210 / shelf.height);
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
  const relativeYaw=(objectYaw-shelfYaw)*Math.PI/180;
  const projectedWidth=Math.abs(Math.cos(relativeYaw))*object.dimensions.x+Math.abs(Math.sin(relativeYaw))*object.dimensions.y;
  const projectedDepth=Math.abs(Math.sin(relativeYaw))*object.dimensions.x+Math.abs(Math.cos(relativeYaw))*object.dimensions.y;
  const fit = projectedWidth + 2*clearance <= shelf.width - 2*shelf.board && projectedDepth + 2*clearance <= shelf.depth-shelf.board && object.dimensions.z + 2*clearance <= (shelf.height - shelf.board*(shelf.levels+1))/shelf.levels;
  const destX = 510 + Math.min(100, platform.x * .12);
  const destY = floorY - Math.min(105, platform.z * .25);
  const placeBayY = floorY-wall-placeSlot*(bayH+wall);
  return <svg viewBox="0 0 680 310" className="h-full w-full" role="img" aria-label="Side view of generated shelf, selected object and destination platform">
    <defs><pattern id="grid" width="20" height="20" patternUnits="userSpaceOnUse"><path d="M 20 0 L 0 0 0 20" fill="none" stroke="#e4e4e7" strokeWidth=".7"/></pattern></defs>
    <rect width="680" height="310" fill="url(#grid)"/>
    <path d={`M ${innerX} ${shelfTop} V ${floorY} M ${innerX+bayW} ${shelfTop} V ${floorY} M ${innerX} ${shelfTop} H ${innerX+bayW}`} stroke="#52525b" strokeWidth={wall*1.5} fill="none"/>
    {Array.from({length:shelf.levels+1},(_,i)=>{const y=floorY-wall-i*(bayH+wall);return <rect key={i} x={innerX-wall} y={y} width={bayW+wall*2} height={wall} fill="#a1a1aa" stroke="#71717a" strokeWidth="1"/>;})}
    <rect x={itemX} y={itemY} width={itemW} height={itemH} rx={object.shape==="cylinder"||object.shape==="sphere"?itemW/2:2} fill="#ef4444" stroke="#991b1b" strokeWidth="1.5"/>
    <path d={`M ${itemX+itemW/2} ${itemY-15} V ${itemY-3}`} stroke="#2563eb" strokeWidth="2"/><path d={`M ${itemX+itemW/2-5} ${itemY-9} L ${itemX+itemW/2} ${itemY-3} L ${itemX+itemW/2+5} ${itemY-9}`} fill="none" stroke="#2563eb" strokeWidth="2"/>
    <text x={innerX} y={shelfTop-10} fontSize="10" fill="#52525b">SHELF · {shelf.width} × {shelf.height} × {shelf.depth} mm</text>
    <text x={innerX+bayW/2} y={itemY+itemH+13} fontSize="9" fill="#52525b" textAnchor="middle">Pick slot {slot+1}</text>
    <rect x="495" y={floorY-9} width="150" height="9" fill="#71717a"/><rect x="520" y={destY} width="92" height={Math.max(15,itemH*.72)} rx="3" fill={placeDestination==="platform"?"#0ea5e9":"#0ea5e9"} fillOpacity={placeDestination==="platform"?".85":".15"} stroke="#0369a1"/>
    {placeDestination==="shelf"?<rect x={innerX+4} y={placeBayY+4} width={bayW-8} height={bayH-8} rx="3" fill="#0ea5e9" fillOpacity=".12" stroke="#0284c7" strokeWidth="2" strokeDasharray="5 3"/>:null}
    <path d={`M ${itemX+itemW/2} ${itemY-18} C 350 65, 440 80, ${placeDestination==="shelf"?innerX+bayW/2:destX+46} ${placeDestination==="shelf"?placeBayY+bayH/2:destY-22}`} fill="none" stroke="#2563eb" strokeWidth="2" strokeDasharray="5 4"/>
    <text x="570" y={floorY+17} textAnchor="middle" fontSize="10" fill="#52525b">{placeDestination==="shelf"?`PLACE · SHELF BAY ${placeSlot+1}`:"PLACE PLATFORM"}</text>
    <text x="570" y={floorY+30} textAnchor="middle" fontSize="9" fill="#71717a">{placeDestination==="shelf"?"Open-front insertion":"Platform destination"}</text>
    <text x="24" y="298" fontSize="9" fill={fit?"#15803d":"#b91c1c"}>{!fitRequired?"SHELF FIT CHECK: NOT REQUIRED":fit?"DIMENSION CHECK: FITS WITH CURRENT CLEARANCE":"DIMENSION CHECK: OBJECT DOES NOT FIT THIS BAY"}</text>
    <text x="656" y="298" fontSize="9" fill="#71717a" textAnchor="end">Approach {approach}°</text>
  </svg>;
}

export function PickPlacePage() {
  const location=useLocation();
  const navigate=useNavigate();
  const model = useRobotStore(s=>s.model);
  const modelLoading = useRobotStore(s=>s.modelLoading);
  const modelError = useRobotStore(s=>s.modelError);
  const bridgeState = useRobotStore(s=>s.bridgeState);
  const simulation = useRobotStore(s=>s.simulation);
  const capabilities = useRobotStore(s=>s.capabilities);
  const scene = useRobotStore(s=>s.scene);
  const sceneError = useRobotStore(s=>s.sceneError);
  const plan = useRobotStore(s=>s.plan);
  const [viewPreferences,setViewPreferences]=useState<VisualizerViewPreferences>(loadVisualizerViewPreferences);
  const emptyDisabledObjects=useMemo<Record<string, any>>(()=>({}),[]);
  const [source,setSource]=useState<InputSource>("synthetic");
  const [objectKind,setObjectKind]=useState<ObjectKind>("coke");
  const [custom,setCustom]=useState<Workpiece>({kind:"custom",label:"Custom object",shape:"box",dimensions:{x:100,y:80,z:80},massKg:.5,massSource:"User supplied"});
  const [shelfId,setShelfId]=useState<string>("standard");
  const [shelfGeometry,setShelfGeometry]=useState({width:600,height:900,depth:400,levels:3,board:18});
  const [slot,setSlot]=useState(1);
  const [placeSlot,setPlaceSlot]=useState(0);
  const [placeDestination,setPlaceDestination]=useState<"platform"|"shelf">("platform");
  const [pickSource,setPickSource]=useState<"shelf"|"free">("free");
  const [clearance,setClearance]=useState(12);
  const [gripperAngle,setGripperAngle]=useState(0);
  const [gripperRoll,setGripperRoll]=useState(90);
  const [approachAngle,setApproachAngle]=useState(-90);
  const [approachDistance,setApproachDistance]=useState(150);
  const [placeApproachDistance,setPlaceApproachDistance]=useState(50);
  const [tcpOffset,setTcpOffset]=useState({x:0,y:0,z:0});
  const [platform,setPlatform]=useState({x:500,y:0,z:-20});
  const [platformDimensions,setPlatformDimensions]=useState({x:120,y:120,z:20});
  const [gripperClearanceMm,setGripperClearanceMm]=useState(6);
  const [plannerSettings,setPlannerSettings]=useState({
    plannerId:"",allowedPlanningTime:8,numPlanningAttempts:8,
    maxVelocityScaling:0.15,maxAccelerationScaling:0.15,
    positionToleranceMm:3,orientationToleranceDeg:2.9,liftDistanceMm:80,
  });
  const [maxPayloadKg,setMaxPayloadKg]=useState(5);
  const [shelfPose,setShelfPose]=useState({x:150,y:-600,z:0,yaw:0});
  const [shelfPickOffset,setShelfPickOffset]=useState({x:0,y:150});
  const [pick,setPick]=useState({x:150,y:-450,z:691});
  const [density,setDensity]=useState(650);
  const [planState,setPlanState]=useState<"idle"|"ready"|"blocked">("idle");
  const [planResult,setPlanResult]=useState<any>(null);
  const [activeStage,setActiveStage]=useState(0);
  const [workflowMode,setWorkflowMode]=useState<WorkflowMode>("platform_to_platform");
  const [executionBusy,setExecutionBusy]=useState<"pick"|"place"|null>(null);
  const [simulationConfirmed,setSimulationConfirmed]=useState(false);
  const [pickExecution,setPickExecution]=useState<any>(null);
  const [placeExecution,setPlaceExecution]=useState<any>(null);
  const [verification,setVerification]=useState<any>(null);
  const [verificationBusy,setVerificationBusy]=useState(false);
  const [preparedSceneKey,setPreparedSceneKey]=useState<string|null>(null);
  const [preparedSceneObjectIds,setPreparedSceneObjectIds]=useState<string[]>([]);
  const [plannedConfigKey,setPlannedConfigKey]=useState<string|null>(null);
  const [sceneStatus,setSceneStatus]=useState<string|null>(null);
  const [graspStrategy,setGraspStrategy]=useState("pgea_finger_wrap");
  const [toolMassKg,setToolMassKg]=useState(1.3);
  const [toolCom,setToolCom]=useState({x:-86.1,y:0,z:-92.6});
  const [controllerToolVerified,setControllerToolVerified]=useState(false);
  const [payloadBusy,setPayloadBusy]=useState(false);
  const [sceneBusy,setSceneBusy]=useState(false);
  const [defaultsBusy,setDefaultsBusy]=useState(false);
  const [defaultsStatus,setDefaultsStatus]=useState<string|null>(null);
  const [defaultsError,setDefaultsError]=useState(false);
  const [payloadStatus,setPayloadStatus]=useState<string|null>(null);
  const [visionFrame,setVisionFrame]=useState<string|null>(null);
  const [objectYaw,setObjectYaw]=useState(90);
  const {request}=useRobotSocket();
  useEffect(()=>saveVisualizerViewPreferences(viewPreferences),[viewPreferences]);
  useEffect(()=>{
    const transfer=(location.state as any)?.visionTransfer;
    if(!transfer||!transfer.object)return;
    const item=transfer.object as Record<string,any>;
    const dimensions=Array.isArray(item.dimensions_m)?item.dimensions_m:[0.1,0.1,0.1];
    const center=Array.isArray(item.position_m)?item.position_m:(Array.isArray(item.centroid_m)?item.centroid_m:[0,0,0]);
    const label=String(item.class_name??item.label??item.id??"Vision object");
    const classKey=label.toLowerCase();
    const mass=Number(item.mass_kg??(classKey.includes("coke")||classKey.includes("cola")?0.355:classKey.includes("pringles")?0.215:0.5));
    setCustom({kind:"custom",label,shape:"box",dimensions:{x:Number(dimensions[0])*1000,y:Number(dimensions[1])*1000,z:Number(dimensions[2])*1000},massKg:Number.isFinite(mass)&&mass>0?mass:0.5,massSource:"Vision 3D box · calibrated depth/TF"});
    setObjectKind("custom");setPick({x:Number(center[0])*1000,y:Number(center[1])*1000,z:Number(center[2])*1000});
    setObjectYaw(Number(item.yaw_deg??0));setVisionFrame(String(item.frame_id??model?.base_frame??"dummy_link"));
    setSource("vision");setPickSource("free");setPlanResult(null);setPlanState("idle");setPreparedSceneKey(null);setPreparedSceneObjectIds([]);setPlannedConfigKey(null);
    setPlaceDestination(transfer.suggestedDestination==="table"?"platform":"shelf");
    if(transfer.suggestedDestination==="table"&&transfer.table?.position_m){
      setPlatform({x:Number(transfer.table.position_m[0])*1000,y:Number(transfer.table.position_m[1])*1000,z:Number(transfer.table.position_m[2])*1000});
      if(Array.isArray(transfer.table.dimensions_m))setPlatformDimensions({x:Number(transfer.table.dimensions_m[0])*1000,y:Number(transfer.table.dimensions_m[1])*1000,z:Number(transfer.table.dimensions_m[2])*1000});
    }
    setActiveStage(0);
    navigate(location.pathname+location.search,{replace:true,state:null});
  },[location.key]);
  const [visionJson,setVisionJson]=useState('{\n  "frame_id": "dummy_link",\n  "objects": [{ "id": "vision_object_1", "class": "can", "shape": "cylinder", "centroid_m": [0.30, 0.00, 0.10], "dimensions_m": [0.066, 0.066, 0.122], "yaw_deg": 0, "mass_kg": 0.355 }]\n}');
  const shelf={...(SHELVES.find(item=>item.id===shelfId)??SHELVES[1]),...shelfGeometry};
  useEffect(()=>{setSlot(value=>Math.min(value,Math.max(0,shelf.levels-1)));setPlaceSlot(value=>Math.min(value,Math.max(0,shelf.levels-1)));},[shelf.levels]);
  const object=objectKind==="custom"?custom:OBJECTS[objectKind];
  const massKg=objectKind==="wood_box"?object.dimensions.x*object.dimensions.y*object.dimensions.z/1e9*density:object.massKg;
  const bayWidth=shelf.width-2*shelf.board;
  const bayHeight=(shelf.height-shelf.board*(shelf.levels+1))/shelf.levels;
  const generatedPick={x:shelfPose.x+shelfPickOffset.x,y:shelfPose.y+shelfPickOffset.y,z:shelfPose.z+shelf.board+slot*(bayHeight+shelf.board)+clearance+object.dimensions.z/2};
  const activePick=source==="synthetic"&&pickSource==="shelf"?generatedPick:pick;
  const shelfRelativeObjectYaw=(objectYaw-shelfPose.yaw)*Math.PI/180;
  const projectedShelfWidth=Math.abs(Math.cos(shelfRelativeObjectYaw))*object.dimensions.x+Math.abs(Math.sin(shelfRelativeObjectYaw))*object.dimensions.y;
  const projectedShelfDepth=Math.abs(Math.sin(shelfRelativeObjectYaw))*object.dimensions.x+Math.abs(Math.cos(shelfRelativeObjectYaw))*object.dimensions.y;
  const fit=projectedShelfWidth+2*clearance<=bayWidth&&projectedShelfDepth+2*clearance<=shelf.depth-shelf.board&&object.dimensions.z+2*clearance<=bayHeight;
  const shelfGeometryValid=shelf.width>2*shelf.board&&shelf.depth>shelf.board&&shelf.height>shelf.board*(shelf.levels+1)&&shelf.levels>=1&&shelf.levels<=8&&shelf.board>=5;
  const shelfFitRequired=pickSource==="shelf"||placeDestination==="shelf";
  const sceneFitValid=!shelfFitRequired||fit;
  const grasp=useMemo(()=>{
    const roll=gripperRoll*Math.PI/180;
    const pitch=-approachAngle*Math.PI/180;
    const yaw=(gripperAngle+objectYaw)*Math.PI/180-Math.PI/2;
    const cr=Math.cos(roll/2),sr=Math.sin(roll/2);
    const cp=Math.cos(pitch/2),sp=Math.sin(pitch/2);
    const cy=Math.cos(yaw/2),sy=Math.sin(yaw/2);
    const q={x:sr*cp*cy-cr*sp*sy,y:cr*sp*cy+sr*cp*sy,z:cr*cp*sy-sr*sp*cy,w:cr*cp*cy+sr*sp*sy};
    const rotate=(v:Dimensions)=>({
      x:(1-2*(q.y*q.y+q.z*q.z))*v.x+2*(q.x*q.y-q.z*q.w)*v.y+2*(q.x*q.z+q.y*q.w)*v.z,
      y:2*(q.x*q.y+q.z*q.w)*v.x+(1-2*(q.x*q.x+q.z*q.z))*v.y+2*(q.y*q.z-q.x*q.w)*v.z,
      z:2*(q.x*q.z-q.y*q.w)*v.x+2*(q.y*q.z+q.x*q.w)*v.y+(1-2*(q.x*q.x+q.y*q.y))*v.z,
    });
    const offsetWorld=rotate(tcpOffset);
    const approachVector=rotate({x:1,y:0,z:0});
    const tcp={x:activePick.x+offsetWorld.x,y:activePick.y+offsetWorld.y,z:activePick.z+offsetWorld.z};
    return {tcp,pregrasp:{x:tcp.x-approachVector.x*approachDistance,y:tcp.y-approachVector.y*approachDistance,z:tcp.z-approachVector.z*approachDistance},
      offsetWorld,approachVector,orientation:q,yaw:gripperAngle+objectYaw,approach:approachAngle,roll:gripperRoll,distance:approachDistance};
  },[activePick,gripperAngle,gripperRoll,objectYaw,approachAngle,approachDistance,tcpOffset]);
  const displayedTargetTcp=useMemo(()=>({x:grasp.tcp.x,y:grasp.tcp.y,z:grasp.tcp.z,rx:gripperRoll,ry:-approachAngle,rz:grasp.yaw-90}),[grasp.tcp.x,grasp.tcp.y,grasp.tcp.z,gripperRoll,approachAngle,grasp.yaw]);
  const yawRad=(gripperAngle+objectYaw)*Math.PI/180;
  const relativeJawYawRad=gripperAngle*Math.PI/180;
  const objectWidthMm=Math.abs(Math.cos(relativeJawYawRad))*object.dimensions.x+Math.abs(Math.sin(relativeJawYawRad))*object.dimensions.y;
  const gripperEnvelopeMm=graspStrategy==="pgea_finger_wrap"?85:40;
  const gripperWidthFeasible=objectWidthMm+gripperClearanceMm<=gripperEnvelopeMm;
  const totalPayloadKg=toolMassKg+massKg;
  const approachRad=approachAngle*Math.PI/180;
  const objectComOffsetMm={x:-grasp.offsetWorld.x,y:-grasp.offsetWorld.y,z:-grasp.offsetWorld.z};
  const payloadCom={
    x:(toolMassKg*toolCom.x+massKg*objectComOffsetMm.x)/totalPayloadKg,
    y:(toolMassKg*toolCom.y+massKg*objectComOffsetMm.y)/totalPayloadKg,
    z:(toolMassKg*toolCom.z+massKg*objectComOffsetMm.z)/totalPayloadKg,
  };
  const visionFrameValid=source!=="vision"||(!!visionFrame&&visionFrame===model?.base_frame);
  const placeBayDistinct=pickSource!=="shelf"||placeDestination!=="shelf"||slot!==placeSlot;
  const shelfYawForKeepout=shelfPose.yaw*Math.PI/180;
  const shelfDx=(platform.x-shelfPose.x)/1000;
  const shelfDy=(platform.y-shelfPose.y)/1000;
  const platformHalfShelfWidth=(Math.abs(Math.cos(shelfYawForKeepout))*platformDimensions.x+Math.abs(Math.sin(shelfYawForKeepout))*platformDimensions.y)/2000;
  const platformHalfShelfDepth=(Math.abs(Math.sin(shelfYawForKeepout))*platformDimensions.x+Math.abs(Math.cos(shelfYawForKeepout))*platformDimensions.y)/2000;
  const platformLocalX=Math.cos(shelfYawForKeepout)*shelfDx+Math.sin(shelfYawForKeepout)*shelfDy;
  const platformLocalY=-Math.sin(shelfYawForKeepout)*shelfDx+Math.cos(shelfYawForKeepout)*shelfDy;
  const platformSeparationX=Math.abs(platformLocalX)-shelf.width/2000-platformHalfShelfWidth;
  const platformSeparationY=Math.abs(platformLocalY)-shelf.depth/2000-platformHalfShelfDepth;
  const platformShelfClearanceMm=Math.max(platformSeparationX,platformSeparationY)*1000;
  const platformKeepoutRequired=pickSource==="shelf"&&placeDestination==="platform";
  const platformClearOfShelf=!platformKeepoutRequired||platformShelfClearanceMm>=150;
  const shelfFrontInwardX=Math.sin(shelfYawForKeepout);
  const shelfFrontInwardY=-Math.cos(shelfYawForKeepout);
  const shelfApproachAlignment=grasp.approachVector.x*shelfFrontInwardX+grasp.approachVector.y*shelfFrontInwardY;
  const shelfApproachAligned=pickSource!=="shelf"||shelfApproachAlignment>=Math.cos(35*Math.PI/180);
  const checks=[{label:"Platform outside shelf keep-out",ok:platformClearOfShelf,detail:!platformKeepoutRequired?"Not required for this workflow":`${Math.round(platformShelfClearanceMm)} mm separation · 150 mm minimum`},{label:"Object fits selected shelf bay",ok:sceneFitValid,detail:!shelfFitRequired?"Not required for free-space to platform":`Bay ${bayWidth} × ${shelf.depth} × ${Math.round(bayHeight)} mm`},{label:"Shelf source and destination differ",ok:placeBayDistinct,detail:placeDestination!=="shelf"?"Platform destination":`Pick bay ${slot+1} · destination bay ${placeSlot+1}`},{label:"Shelf opening approach",ok:(pickSource!=="shelf"&&placeDestination!=="shelf")||shelfApproachAligned,detail:shelfApproachAligned?"Gripper approaches along the open-front axis":"Adjust gripper yaw/approach to align with the shelf front"},{label:"Approach clearance",ok:approachDistance>=Math.max(40,object.dimensions.z*.25),detail:`${approachDistance} mm approach distance`},{label:"Gripper geometry",ok:gripperWidthFeasible,detail:`${objectWidthMm} mm object + ${gripperClearanceMm} mm clearance vs ${gripperEnvelopeMm} mm ${graspStrategy==="pgea_finger_wrap"?"extended-finger wrap envelope (CAD estimate)":"nominal jaw travel"}`},{label:"Vision frame aligned",ok:visionFrameValid,detail:source!=="vision"?"Synthetic / manual input":`Detection frame ${visionFrame??"missing"} · robot base ${model?.base_frame??"unknown"}`},{label:"Shelf geometry valid",ok:shelfGeometryValid,detail:shelfGeometryValid?"Positive dimensions and usable bays":"Check shelf dimensions, board thickness and level count"},{label:"Robot model available",ok:!!model,detail:model?`${model.group_name} · ${model.base_frame} → tcp`:"Waiting for bridge model"}];
  const allValid=checks.every(item=>item.ok);
  const objectReady=Boolean(model && [object.dimensions.x,object.dimensions.y,object.dimensions.z].every(value=>Number.isFinite(value)&&value>0) && visionFrameValid);
  // Only scene geometry inputs invalidate the prepared MoveIt scene. Grasp tuning is tracked separately.
  const sceneConfigKey=JSON.stringify({
    frame_id:source==="vision"?visionFrame:model?.base_frame,source,objectKind,shape:object.shape,
    dimensions:object.dimensions,centroid:activePick,objectYaw,shelfId,shelfGeometry,shelfPose,shelfPickOffset,slot,placeSlot,
    pickSource,placeDestination,clearance,platform,platformDimensions,
  });
  const liveSceneIds=useMemo(()=>new Set((scene?.world?.collision_objects??[]).map((item:any)=>item.id as string)),[scene]);
  const defaultsPresent=["ground_plane","base_pedestal","Box_0"].every(id=>liveSceneIds.has(id));
  const preparedObjectsPresent=preparedSceneObjectIds.length>0&&preparedSceneObjectIds.every(id=>liveSceneIds.has(id));
  const scenePrepared=preparedSceneKey===sceneConfigKey&&defaultsPresent&&preparedObjectsPresent;
  const planConfigKey=JSON.stringify({sceneConfigKey,massKg,density,graspStrategy,gripperAngle,gripperRoll,approachAngle,approachDistance,placeApproachDistance,tcpOffset,gripperClearanceMm,toolMassKg,toolCom,maxPayloadKg,plannerSettings});
  const planReady=scenePrepared&&plannedConfigKey===planConfigKey&&planResult?.status==="PLANNED";
  const pickDone=pickExecution?.status==="SUCCEEDED";
  const placeDone=placeExecution?.status==="SUCCEEDED";
  const executionAvailable=simulation===true&&bridgeState==="connected"&&capabilities.motion_execution_enabled===true&&capabilities.gripper_commands_enabled===true;
  const stageItems=[
    {title:"Object",description:"Select or detect the workpiece",done:objectReady,unlocked:true},
    {title:"Scene",description:"Set pick/place positions and collisions",done:scenePrepared,unlocked:objectReady},
    {title:"Grasp",description:"Set PGEA orientation, approach and TCP",done:scenePrepared&&allValid,unlocked:scenePrepared},
    {title:"Plan",description:"Validate every trajectory segment",done:planReady,unlocked:scenePrepared&&allValid},
    {title:"Pick",description:"Approach, grasp, close and withdraw",done:pickDone,unlocked:planReady},
    {title:"Place",description:"Transfer, lower, release and retreat",done:placeDone,unlocked:pickDone},
    {title:"Verify",description:"Check release and final robot state",done:verification?.confirmed===true,unlocked:placeDone},
  ];
  const goToStage=(index:number)=>{if(stageItems[index]?.unlocked)setActiveStage(index);};
  const updateObject=(patch:Partial<Workpiece>)=>{
    setCustom(current=>({...current,...object,...patch,kind:"custom",massSource:patch.massSource??(patch.massKg!==undefined?"User supplied mass":"Custom object geometry")}));
    setObjectKind("custom");
  };
  const updateObjectDimension=(axis:keyof Dimensions,value:number)=>{
    const dimensions={...object.dimensions,[axis]:value};
    const massKg=objectKind==="wood_box"?dimensions.x*dimensions.y*dimensions.z/1e9*density:object.massKg;
    updateObject({dimensions,massKg,massSource:objectKind==="wood_box"?"Calculated from current density × volume":"Previous mass estimate retained after geometry edit; verify mass"});
  };
  const resetConfiguration=()=>{
    setSource("synthetic");setObjectKind("coke");setCustom({kind:"custom",label:"Custom object",shape:"box",dimensions:{x:100,y:80,z:80},massKg:.5,massSource:"User supplied"});
    setShelfId("standard");setShelfGeometry({width:600,height:900,depth:400,levels:3,board:18});setSlot(1);setPlaceSlot(0);
    setWorkflowMode("platform_to_platform");setPlaceDestination("platform");setPickSource("free");setClearance(12);setGripperAngle(0);setGripperRoll(90);setApproachAngle(-90);setApproachDistance(150);setPlaceApproachDistance(50);setTcpOffset({x:0,y:0,z:0});
    setPlatform({x:500,y:0,z:-20});setPlatformDimensions({x:120,y:120,z:20});setShelfPose({x:450,y:-600,z:0,yaw:0});setShelfPickOffset({x:0,y:150});setPick({x:150,y:-450,z:691});
    setDensity(650);setObjectYaw(90);setGraspStrategy("pgea_finger_wrap");setGripperClearanceMm(6);
    setPlannerSettings({plannerId:"",allowedPlanningTime:8,numPlanningAttempts:6,maxVelocityScaling:.15,maxAccelerationScaling:.15,positionToleranceMm:3,orientationToleranceDeg:2.9,liftDistanceMm:80});
    setToolMassKg(1.3);setToolCom({x:-86.1,y:0,z:-92.6});setMaxPayloadKg(5);setControllerToolVerified(false);setVisionFrame(null);setDefaultsStatus(null);setDefaultsError(false);setPlanResult(null);setPlanState("idle");
    setPreparedSceneKey(null);setPreparedSceneObjectIds([]);setPlannedConfigKey(null);setSceneStatus(null);setPayloadStatus(null);setActiveStage(0);
    setExecutionBusy(null);setPickExecution(null);setPlaceExecution(null);setVerification(null);setSimulationConfirmed(false);
  };
  const applyWorkflowPreset=(mode:WorkflowMode)=>{
    setWorkflowMode(mode);setObjectKind("coke");setSource("synthetic");setVisionFrame(null);
    setPlaceDestination("platform");setClearance(12);setPlatform({x:500,y:0,z:-20});
    setPlatformDimensions({x:120,y:120,z:20});setGripperAngle(0);setTcpOffset({x:0,y:0,z:0});
    setPlannerSettings(current=>({...current,plannerId:"",allowedPlanningTime:15,numPlanningAttempts:10,
      maxVelocityScaling:.15,maxAccelerationScaling:.15,positionToleranceMm:3,orientationToleranceDeg:2.9,liftDistanceMm:30}));
    if(mode==="platform_to_platform"){
      setShelfGeometry({width:600,height:900,depth:400,levels:3,board:18});
      setPickSource("free");setPick({x:150,y:-450,z:691});setObjectYaw(90);setGripperRoll(90);setApproachAngle(-90);
      setApproachDistance(150);setPlaceApproachDistance(150);
    }else{
      setShelfGeometry({width:360,height:900,depth:400,levels:3,board:18});
      setPickSource("shelf");setShelfPose({x:450,y:-600,z:0,yaw:0});setShelfPickOffset({x:0,y:150});setSlot(1);setPlaceSlot(0);
      setPlannerSettings(current=>({...current,liftDistanceMm:120}));
      setObjectYaw(0);setGripperRoll(0);setApproachAngle(0);setApproachDistance(265);setPlaceApproachDistance(150);
      setPlannerSettings(current=>({...current,allowedPlanningTime:30,numPlanningAttempts:20}));
    }
    setPreparedSceneKey(null);setPreparedSceneObjectIds([]);setPlannedConfigKey(null);
    setPlanResult(null);setPlanState("idle");setSceneStatus(null);setPickExecution(null);
    setPlaceExecution(null);setVerification(null);setSimulationConfirmed(false);setActiveStage(0);
  };
  const loadDefaultConstraints=async()=>{
    if(!model||bridgeState!=="connected"||defaultsBusy)return;
    setDefaultsBusy(true);setDefaultsStatus(null);setDefaultsError(false);
    try{
      const result=await request("scene.load_defaults",{frame_id:model.base_frame});
      const added=Array.isArray(result.loaded)?result.loaded.length:0;
      const present=Array.isArray(result.already_present)?result.already_present.length:0;
      setDefaultsStatus(`${added} added · ${present} already present · ${result.frame_id}`);
      const latest=await request("moveit.get_scene");
      useRobotStore.getState().setScene(latest);
    }catch(error){setDefaultsError(true);setDefaultsStatus(error instanceof Error?error.message:String(error));}
    finally{setDefaultsBusy(false);}
  };
  const prepareScene=async()=>{
    if(!model||!objectReady||bridgeState!=="connected"||sceneBusy)return;
    const kind=objectKind==="coke"?"coca_cola_330ml":objectKind==="pringles"?"pringles_165g":objectKind==="wood_box"?"wood_box":"custom";
    const objectPayload:Record<string,unknown>={
      id:"pick_place_object",kind,label:object.label,shape:object.shape,
      dimensions_m:[object.dimensions.x,object.dimensions.y,object.dimensions.z].map(v=>v/1000),
      centroid_m:[activePick.x,activePick.y,activePick.z].map(v=>v/1000),
      rpy_rad:[0,0,objectYaw*Math.PI/180],density_kg_m3:density,
    };
    if(objectKind!=="wood_box")objectPayload.mass_kg=massKg;
    setSceneBusy(true);setSceneStatus(null);
    try{
      const defaults=await request("scene.load_defaults",{frame_id:model.base_frame});
      const result=await request("pick_place.prepare",{
        frame_id:source==="vision"?visionFrame:model.base_frame,group_name:model.group_name,object:objectPayload,
        shelf:{width_m:shelf.width/1000,height_m:shelf.height/1000,depth_m:shelf.depth/1000,levels:shelf.levels,board_m:shelf.board/1000},
        shelf_pose:{position_m:[shelfPose.x,shelfPose.y,shelfPose.z].map(v=>v/1000),yaw_rad:shelfPose.yaw*Math.PI/180},
        level:slot,place_level:placeSlot,pick_source:pickSource,place_destination:placeDestination,clearance_m:clearance/1000,
        platform:{id:"place_platform",position_m:[platform.x,platform.y,platform.z].map(v=>v/1000),dimensions_m:[platformDimensions.x,platformDimensions.y,platformDimensions.z].map(value=>value/1000)},
        gripper_yaw_deg:gripperAngle,gripper_roll_deg:gripperRoll,approach_angle_deg:approachAngle,grasp_strategy:graspStrategy,gripper_clearance_m:gripperClearanceMm/1000,
        approach_distance_m:approachDistance/1000,place_approach_distance_m:placeApproachDistance/1000,tcp_offset_m:[tcpOffset.x,tcpOffset.y,tcpOffset.z].map(value=>value/1000),lift_distance_m:plannerSettings.liftDistanceMm/1000,
      });
      const preparedIds=Array.isArray(result.scene_object_ids)?result.scene_object_ids as string[]:[];
      const latestScene=await request("moveit.get_scene");
      useRobotStore.getState().setScene(latestScene);
      setPreparedSceneObjectIds(preparedIds);setPreparedSceneKey(sceneConfigKey);setPlannedConfigKey(null);setPlanResult(null);setPlanState("idle");
      setPickExecution(null);setPlaceExecution(null);setVerification(null);setSimulationConfirmed(false);
      setDefaultsError(false);setDefaultsStatus(`${defaults.loaded?.length??0} baseline objects restored · ${defaults.already_present?.length??0} already present`);
      setSceneStatus(`Scene prepared · ${preparedIds.length} task objects · ${result.frame_id}`);
      setActiveStage(2);
    }catch(error){setSceneStatus(error instanceof Error?error.message:String(error));}
    finally{setSceneBusy(false);}
  };
  const runPlan=async()=>{
    if(!model||!scenePrepared||!allValid)return;
    setPlanState("idle");setPlannedConfigKey(null);setPlanResult({status:"PLANNING"});
    setPickExecution(null);setPlaceExecution(null);setVerification(null);setSimulationConfirmed(false);
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
        level:slot,place_level:placeSlot,pick_source:pickSource,place_destination:placeDestination,clearance_m:clearance/1000,
        platform:{id:"place_platform",position_m:[platform.x,platform.y,platform.z].map(v=>v/1000),dimensions_m:[platformDimensions.x,platformDimensions.y,platformDimensions.z].map(value=>value/1000)},
        gripper_yaw_deg:gripperAngle,gripper_roll_deg:gripperRoll,approach_angle_deg:approachAngle,grasp_strategy:graspStrategy,gripper_clearance_m:gripperClearanceMm/1000,
        approach_distance_m:approachDistance/1000,place_approach_distance_m:placeApproachDistance/1000,tcp_offset_m:[tcpOffset.x,tcpOffset.y,tcpOffset.z].map(value=>value/1000),lift_distance_m:plannerSettings.liftDistanceMm/1000,
        pipeline_id:"ompl",planner_id:plannerSettings.plannerId,allowed_planning_time:plannerSettings.allowedPlanningTime,
        num_planning_attempts:plannerSettings.numPlanningAttempts,max_velocity_scaling:plannerSettings.maxVelocityScaling,
        max_acceleration_scaling:plannerSettings.maxAccelerationScaling,
        position_tolerance_m:plannerSettings.positionToleranceMm/1000,orientation_tolerance_rad:plannerSettings.orientationToleranceDeg*Math.PI/180,
        tool_mass_kg:toolMassKg,tool_com_m:[toolCom.x/1000,toolCom.y/1000,toolCom.z/1000],object_com_m:[objectComOffsetMm.x/1000,objectComOffsetMm.y/1000,objectComOffsetMm.z/1000],max_payload_kg:maxPayloadKg,
      };
      const result=await request("pick_place.plan",taskPayload);
      setPlanResult(result);setPlanState(result.status==="PLANNED"?"ready":"blocked");
      if(result.status==="PLANNED"){setPlannedConfigKey(planConfigKey);setActiveStage(4);}
    }catch(error){setPlanResult({status:"FAILED",error:error instanceof Error?error.message:String(error)});setPlanState("blocked");}
  };
  const executePhase=async(phase:"pick"|"place")=>{
    if(!planReady||!executionAvailable||!simulationConfirmed||executionBusy)return;
    if(phase==="place"&&!pickDone)return;
    const platformToPlatform=workflowMode==="platform_to_platform";
    const required=phase==="pick"
      ?(platformToPlatform?["pregrasp","grasp","withdraw"]:["pregrasp","grasp","withdraw","lift"])
      :(platformToPlatform
        ?["transit_raise","transit_clear","transit_lower","transit_align","preplace","place","retreat_clear","retreat_lift","retreat"].filter(name=>(planResult?.plans??[]).some((item:any)=>item.name===name))
        :["preplace","place","retreat"]);
    const plans=(planResult?.plans??[]).filter((item:any)=>required.includes(item.name)&&item.plan_id).map((item:any)=>({name:item.name,plan_id:item.plan_id}));
    if(plans.length!==required.length){setVerification({error:"Missing planned segments: "+required.filter(name=>!plans.some((item:any)=>item.name===name)).join(", ")});return;}
    setExecutionBusy(phase);setVerification(null);
    if(phase==="pick")setPickExecution({status:"RUNNING",completed:[]});
    else setPlaceExecution({status:"RUNNING",completed:[]});
    try{
      const result=await request("pick_place.execute_phase",{phase,workflow:workflowMode,plans});
      if(phase==="pick"){setPickExecution(result);if(result.status==="SUCCEEDED")setActiveStage(5);}
      else{setPlaceExecution(result);if(result.status==="SUCCEEDED")setActiveStage(6);}
    }catch(error){
      const failed={phase,status:"FAILED",error:error instanceof Error?error.message:String(error)};
      if(phase==="pick")setPickExecution(failed);else setPlaceExecution(failed);
    }finally{setExecutionBusy(null);}
  };
  const refreshVerification=async()=>{
    if(!executionAvailable||verificationBusy)return;
    setVerificationBusy(true);
    try{
      const [snapshot,gripperStatus]=await Promise.all([request("robot.get_snapshot",{}),request("gripper.get_status",{})]);
      setVerification((current:any)=>({snapshot,gripperStatus,confirmed:current?.confirmed===true}));
    }catch(error){setVerification((current:any)=>({...current,error:error instanceof Error?error.message:String(error)}));}
    finally{setVerificationBusy(false);}
  };
  const confirmPlacement=(confirmed:boolean)=>setVerification((current:any)=>({...current,confirmed}));
  const parseVision=()=>{try{
    const data=JSON.parse(visionJson),first=data.objects?.[0];
    if(!first)throw new Error("No objects in vision payload");
    if(typeof data.frame_id!=="string"||!data.frame_id)throw new Error("Vision payload must include frame_id");
    const dims=first.dimensions_m??[.1,.1,.1];
    if(!Array.isArray(dims)||dims.length!==3||!dims.every((value:number)=>Number.isFinite(value)&&value>0))throw new Error("dimensions_m must contain three positive metre values");
    const centroid=first.centroid_m??[0,0,0];
    if(!Array.isArray(centroid)||centroid.length!==3||!centroid.every((value:number)=>Number.isFinite(value)))throw new Error("centroid_m must contain three finite metre values");
    const mass=Number(first.mass_kg??.5);if(!Number.isFinite(mass)||mass<=0)throw new Error("mass_kg must be positive");
    const shape=first.shape==="cylinder"?"cylinder":first.shape==="sphere"?"sphere":"box";
    const detectedClass=String(first.class??first.class_name??first.kind??"").toLowerCase();
    setCustom({kind:"custom",label:String(first.class??first.id??"Vision object"),shape,dimensions:{x:dims[0]*1000,y:dims[1]*1000,z:dims[2]*1000},massKg:mass,massSource:"Vision detection"});
    setObjectKind("custom");setGraspStrategy(detectedClass.includes("coke")||detectedClass.includes("cola")||detectedClass.includes("pringles")?"pgea_finger_wrap":"parallel_jaw");
    setPick({x:centroid[0]*1000,y:centroid[1]*1000,z:centroid[2]*1000});
    setObjectYaw(Number(first.yaw_deg??((first.rpy_rad?.[2]??0)*180/Math.PI)));setVisionFrame(data.frame_id);setSource("vision");setPickSource("free");setPlanState("idle");
  }catch(error){window.alert(error instanceof Error?error.message:"Invalid vision payload");}};
  const copyGrasp=()=>{void navigator.clipboard?.writeText(JSON.stringify({frame_id:model?.base_frame??"dummy_link",link_name:"tcp",tcp_target_m:[grasp.tcp.x/1000,grasp.tcp.y/1000,grasp.tcp.z/1000],gripper_yaw_deg:gripperAngle,gripper_yaw_absolute_deg:grasp.yaw,approach_angle_deg:grasp.approach,approach_distance_m:grasp.distance/1000,tcp_offset_m:[tcpOffset.x,tcpOffset.y,tcpOffset.z].map(value=>value/1000),object_yaw_deg:objectYaw,object_mass_kg:massKg,object_dimensions_m:[object.dimensions.x,object.dimensions.y,object.dimensions.z].map(v=>v/1000)},null,2)).catch(()=>{});};
  const applyPayload=async()=>{
    if(bridgeState!=="connected"||!controllerToolVerified||payloadBusy||totalPayloadKg>maxPayloadKg)return;
    setPayloadBusy(true);setPayloadStatus(null);
    try{
      const result=await request("robot.command",{command:"set-payload",payload:{load:Number(totalPayloadKg.toFixed(4)),x:Number(payloadCom.x.toFixed(2)),y:Number(payloadCom.y.toFixed(2)),z:Number(payloadCom.z.toFixed(2))}});
      setPayloadStatus(result?.res===0?`SetPayload accepted · ${JSON.stringify(result)}`:`Controller rejected SetPayload · ${JSON.stringify(result)}`);
    }catch(error){setPayloadStatus(error instanceof Error?error.message:String(error));}
    finally{setPayloadBusy(false);}
  };
  return <div className="space-y-5">
    <div className="flex flex-wrap items-start justify-between gap-3"><div><div className="eyebrow">MOTION WORKSPACE / TASK PLANNING</div><h1 className="page-title">Pick & place planner</h1><p className="mt-1 max-w-2xl text-sm text-zinc-500">Follow the stages in order. Each stage unlocks after the previous action succeeds.</p></div><div className="flex items-center gap-2"><span className={`rounded-full px-2.5 py-1 text-[10px] font-semibold ${simulation?"bg-amber-50 text-amber-700":"bg-zinc-100 text-zinc-600"}`}>{simulation?"SIMULATION":"PLAN ONLY"}</span><span className="rounded-full bg-zinc-100 px-2.5 py-1 text-[10px] text-zinc-600">{bridgeState}</span></div></div>
    <section className="panel space-y-3 p-4">
      <div className="flex flex-wrap items-center justify-between gap-2">
        <div><div className="text-sm font-semibold">Choose pick-and-place workflow</div><p className="mt-0.5 text-[11px] text-zinc-500">Choose one of the two supported task patterns. Object details and motion tuning stay under Advanced configuration.</p></div>
        <span className="rounded-full bg-zinc-100 px-2.5 py-1 text-[10px] text-zinc-600">{object.label}</span>
      </div>
      <div className="grid gap-2 sm:grid-cols-2">
        <button type="button" onClick={()=>applyWorkflowPreset("platform_to_platform")} aria-pressed={workflowMode==="platform_to_platform"}
          className={workflowMode==="platform_to_platform"?"rounded-lg border border-zinc-900 bg-zinc-900 p-3 text-left text-white":"rounded-lg border border-zinc-200 bg-white p-3 text-left text-zinc-800 hover:border-zinc-400"}>
          <span className="block text-xs font-semibold">Platform → Platform</span>
          <span className={workflowMode==="platform_to_platform"?"mt-1 block text-[10px] text-zinc-300":"mt-1 block text-[10px] text-zinc-500"}>Pick from a free-space position and place onto a destination platform.</span>
        </button>
        <button type="button" onClick={()=>applyWorkflowPreset("shelf_to_platform")} aria-pressed={workflowMode==="shelf_to_platform"}
          className={workflowMode==="shelf_to_platform"?"rounded-lg border border-zinc-900 bg-zinc-900 p-3 text-left text-white":"rounded-lg border border-zinc-200 bg-white p-3 text-left text-zinc-800 hover:border-zinc-400"}>
          <span className="block text-xs font-semibold">Shelf → Platform</span>
          <span className={workflowMode==="shelf_to_platform"?"mt-1 block text-[10px] text-zinc-300":"mt-1 block text-[10px] text-zinc-500"}>Pick through the open front, withdraw, transfer outside the rack and place at base-link height.</span>
        </button>
      </div>
      <div className="flex flex-wrap items-center justify-between gap-3 border-t border-zinc-100 pt-3">
        <div className="min-w-0 text-[10px] text-zinc-500">
          <div>Pick: {pickSource==="shelf"?"Shelf bay "+(slot+1):"Free-space position"} · Place: Platform</div>
          <div>Frame: {model?.base_frame??"waiting for robot model"} · Scene: {scenePrepared?"ready":"not prepared"} · Plan: {planResult?.status??"not generated"}</div>
          {sceneStatus&&<div className="mt-1 break-words text-amber-700">{sceneStatus}</div>}
        </div>
        <div className="flex flex-wrap gap-2">
          <Button size="sm" variant="outline" disabled={!objectReady||sceneBusy||bridgeState!=="connected"||!shelfGeometryValid||!sceneFitValid} onClick={prepareScene}>
            <Layers3 size={13}/>{sceneBusy?"Preparing…":scenePrepared?"Refresh scene":"Prepare scene"}
          </Button>
          <Button size="sm" disabled={!scenePrepared||!allValid||!model||bridgeState!=="connected"||planResult?.status==="PLANNING"} onClick={runPlan}>
            <Target size={13}/>{planResult?.status==="PLANNING"?"Planning…":planReady?"Re-plan":"Generate plan"}
          </Button>
        </div>
      </div>
      {planResult&&<div className={planReady?"rounded-md border border-emerald-200 bg-emerald-50 px-3 py-2 text-[10px] text-emerald-800":"rounded-md border border-amber-200 bg-amber-50 px-3 py-2 text-[10px] text-amber-800"}>
        <div className="font-semibold">{planResult.status}{planResult.failed_stage?" · stopped at "+planResult.failed_stage:""}</div>
        {planResult.error&&<div className="mt-1 break-words">{planResult.error}</div>}
        {Array.isArray(planResult.plans)&&<div className="mt-1">{planResult.plans.filter((item:any)=>item.status==="SUCCEEDED").length}/{planResult.plans.length} segments planned</div>}
      </div>}
    </section>
    <section className="panel overflow-hidden pick-place-viewport">
      <div className="panel-header"><div><div className="eyebrow">3D / MOVEIT PLANNING SCENE</div><h2 className="mt-1 text-sm font-semibold">Robot, workpiece and destination platform</h2></div><div className="text-[10px] text-zinc-400">{scene?.world?.collision_objects?.length??0} world · {scene?.robot_state?.attached_collision_objects?.length??0} attached</div></div>
      <RobotViewport model={model} scene={scene} plan={plan} targetTcp={displayedTargetTcp}
        loading={modelLoading} error={modelError??sceneError}
        viewPreferences={viewPreferences} onViewPreferencesChange={setViewPreferences}
        objectLabels={viewPreferences.objectLabels} disabledObjects={emptyDisabledObjects} hiddenObjects={viewPreferences.hiddenObjects}/>
      <div className="flex flex-wrap items-center justify-between gap-2 border-t border-zinc-100 px-4 py-2 text-[10px] text-zinc-500">
        <div className="min-w-0"><div>MoveIt scene data · {model?.base_frame??"waiting for base frame"}</div><div className={scenePrepared?"text-emerald-700":"text-amber-700"}>{scenePrepared?sceneStatus??"Scene ready for current settings":sceneStatus?`Last preparation: ${sceneStatus}`:"Scene not prepared for the current settings"}</div></div>
        <span>{activeStage===1?"Scene setup is active below":"Follow the stages to prepare and validate this scene"}</span>
      </div>
    </section>
    <details className="panel overflow-hidden group">
      <summary className="flex cursor-pointer list-none items-center justify-between gap-3 px-4 py-3">
        <span><span className="block text-xs font-semibold text-zinc-800">Advanced configuration</span><span className="mt-0.5 block text-[10px] text-zinc-500">Object geometry, source, shelf/platform positions, grasp, TCP, planner, payload and simulation stages.</span></span><ChevronDown size={14} className="transition-transform group-open:rotate-180"/>
      </summary>
      <div className="space-y-4 border-t border-zinc-100 p-3 sm:p-4">
    <section className="panel p-3 sm:p-4">
      <div className="grid grid-cols-2 gap-2 md:grid-cols-4 xl:grid-cols-7">
        {stageItems.map((stage,index)=>{
          const current=activeStage===index;
          const locked=!stage.unlocked;
          return <button key={stage.title} type="button" disabled={locked} onClick={()=>goToStage(index)} aria-current={current?"step":undefined}
            className={`rounded-lg border p-3 text-left transition-colors ${current?"border-zinc-900 bg-zinc-900 text-white":stage.done?"border-emerald-200 bg-emerald-50 text-emerald-900":"border-zinc-200 bg-white text-zinc-700"} ${locked?"cursor-not-allowed opacity-45":"hover:border-zinc-400"}`}>
            <div className="flex items-center justify-between gap-2"><span className="text-[10px] font-semibold uppercase tracking-wider">Step {index+1}</span><span className="text-[10px] font-semibold">{stage.done?"Complete":locked?"Locked":current?"Current":"Ready"}</span></div>
            <div className="mt-1 text-sm font-semibold">{stage.title}</div>
            <div className={`mt-1 text-[10px] leading-4 ${current?"text-zinc-300":stage.done?"text-emerald-800":"text-zinc-500"}`}>{stage.description}</div>
          </button>;
        })}
      </div>
      <div className="mt-3 flex flex-wrap items-center justify-between gap-2 border-t border-zinc-100 pt-3 text-[11px]">
        <span className="text-zinc-500">{stageItems[activeStage]?.description}. {activeStage===3?"Planning does not move the robot.":activeStage>=4?"Execution is simulation-only and requires explicit confirmation.":""}</span>
        <span className="font-medium text-zinc-700">{stageItems.filter(stage=>stage.done).length} / 7 stages complete</span>
      </div>
    </section>
    <div className="grid gap-4">
      <div className="space-y-4 min-w-0">
        <details className="panel overflow-hidden group">
          <summary className="flex cursor-pointer list-none items-center justify-between gap-3 px-4 py-3">
            <span><span className="block text-xs font-semibold text-zinc-800">Configuration preview</span><span className="mt-0.5 block text-[10px] text-zinc-500">{object.label} · {shelf.width} × {shelf.height} × {shelf.depth} mm shelf · {!shelfFitRequired?"shelf fit not required":fit?"fits current bay":"does not fit current bay"}</span></span>
            <ChevronDown size={14} className="transition-transform group-open:rotate-180"/>
          </summary>
          <div className="flex justify-end border-t border-zinc-100 px-3 py-2"><Button size="sm" variant="outline" onClick={resetConfiguration}>Reset all settings</Button></div>
          <div className="h-[230px] border-t border-zinc-100 p-2 sm:h-[270px]"><ShelfPreview shelf={shelf} object={object} slot={slot} placeSlot={placeSlot} placeDestination={placeDestination} clearance={clearance} platform={platform} approach={approachAngle} objectYaw={objectYaw} shelfYaw={shelfPose.yaw} fitRequired={shelfFitRequired}/></div>
          <div className="grid grid-cols-2 gap-3 border-t border-zinc-100 px-4 py-2.5 text-[11px] sm:grid-cols-4"><div><div className="text-zinc-400">Object payload</div><div className="mt-0.5 font-mono font-semibold">{number(massKg,3)} kg</div></div><div><div className="text-zinc-400">Selected bay</div><div className="mt-0.5 font-mono">{slot+1} / {shelf.levels}</div></div><div><div className="text-zinc-400">Destination</div><div className="mt-0.5 font-medium">{placeDestination==="shelf"?`Shelf bay ${placeSlot+1}`:"Platform"}</div></div><div><div className="text-zinc-400">Fit check</div><div className={`mt-0.5 font-semibold ${fit?"text-emerald-700":"text-red-700"}`}>{fit?"Fits":"Does not fit"}</div></div></div>
        </details>
        {activeStage===0&&<section className="panel overflow-hidden">
          <div className="panel-header"><div><div className="text-sm font-semibold">1 · Object and input</div><div className="mt-0.5 text-[10px] text-zinc-500">Select a preset, then edit any physical dimension or mass in the table.</div></div><span className="text-[10px] text-zinc-500">{object.label}</span></div>
          <div className="grid gap-3 p-4 xl:grid-cols-2">
            <div className="space-y-3">
              <label className="block"><span className="mb-1 block text-[10px] font-medium uppercase tracking-wider text-zinc-500">Object preset</span><select value={objectKind} onChange={event=>{const next=event.currentTarget.value as ObjectKind;setObjectKind(next);setSource("synthetic");setVisionFrame(null);if(next==="wood_box")setGraspStrategy("parallel_jaw");else if(next==="coke"||next==="pringles")setGraspStrategy("pgea_finger_wrap");setPlanResult(null);setPlannedConfigKey(null);}} className="h-9 w-full rounded-md border border-zinc-200 bg-white px-2 text-xs"><option value="coke">Coca-Cola can · 330 ml</option><option value="pringles">Pringles tube · 165 g</option><option value="wood_box">Wooden box · density model</option><option value="custom">Custom / vision object</option></select></label>
              <ConfigTable title="Object properties" description="All dimensions and mass are sent to the scene and plan request.">
                <ConfigNumberRow label="Diameter / length X" value={object.dimensions.x} unit="mm" min={1} max={2000} onChange={value=>updateObjectDimension("x",value)}/>
                <ConfigNumberRow label="Depth / width Y" value={object.dimensions.y} unit="mm" min={1} max={2000} onChange={value=>updateObjectDimension("y",value)}/>
                <ConfigNumberRow label="Height Z" value={object.dimensions.z} unit="mm" min={1} max={2000} onChange={value=>updateObjectDimension("z",value)}/>
                <ConfigSelectRow label="Collision shape" value={object.shape} options={[{value:"box",label:"Box"},{value:"cylinder",label:"Cylinder"},{value:"sphere",label:"Sphere"}]} onChange={value=>updateObject({shape:value as Workpiece["shape"]})}/>
                {objectKind==="wood_box"?<ConfigNumberRow label="Wood density" value={density} unit="kg/m³" min={100} max={1400} onChange={value=>setDensity(value)}/>:<ConfigNumberRow label="Object mass" value={massKg} unit="kg" min={0.001} max={100} step={0.001} onChange={value=>updateObject({massKg:value})}/>}
              </ConfigTable>
              <p className="text-[10px] leading-4 text-zinc-500">Current mass: {number(massKg,3)} kg · {objectKind==="wood_box"?"calculated from volume × density":object.massSource}. Editing a preset converts it to a custom object so values are not discarded.</p>
            </div>
            <div className="space-y-3">
              <ConfigTable title="Input source" description="Vision detections preserve their measured dimensions and centroid. Manual poses are entered in Step 3.">
                <tr><th className="px-3 py-2 font-medium text-zinc-700">Object source</th><td colSpan={2} className="px-2 py-1.5"><div className="flex flex-wrap gap-1">{(["synthetic","manual","vision"] as const).map(value=><button key={value} type="button" onClick={()=>{if(value==="manual"){if(source==="synthetic")setPick(activePick);setPickSource("free");}setSource(value);}} className={`rounded-md border px-2.5 py-1.5 text-[10px] font-medium capitalize ${source===value?"border-zinc-900 bg-zinc-900 text-white":"border-zinc-200 text-zinc-600 hover:bg-zinc-50"}`}>{value}</button>)}</div></td></tr>
                <tr><th className="px-3 py-2 font-medium text-zinc-700">Reference frame</th><td colSpan={2} className="px-2 py-1.5 font-mono text-[11px] text-zinc-600">{source==="vision"?visionFrame??"not loaded":model?.base_frame??"waiting for robot model"}</td></tr>
              </ConfigTable>
              {source==="vision"&&<div className="space-y-2 rounded-lg border border-zinc-200 p-3"><div className="text-xs font-semibold">Vision detection JSON</div><textarea aria-label="Vision detection JSON" value={visionJson} onChange={event=>setVisionJson(event.currentTarget.value)} rows={5} spellCheck={false} className="w-full rounded-md border border-zinc-200 p-2 font-mono text-[10px] leading-4"/><Button size="sm" variant="outline" onClick={parseVision}><ScanLine size={13}/> Load detection</Button><p className="text-[10px] leading-4 text-zinc-500">Expected: frame_id and objects[0] with centroid_m, dimensions_m, shape and optional mass_kg / yaw_deg. Frame must match {model?.base_frame??"the MoveIt base frame"}.</p></div>}
              {source==="manual"&&<div className="rounded-md bg-zinc-50 p-3 text-[10px] leading-4 text-zinc-600">Manual mode uses the object centroid entered in Step 3. No camera transform is applied automatically.</div>}
            </div>
          </div>
          <div className="flex items-center justify-between border-t border-zinc-100 px-4 py-3"><span className="text-[10px] text-zinc-500">{objectReady?`${number(massKg,3)} kg · ${model?.base_frame}`:"Robot model or valid object geometry is not ready."}</span><Button size="sm" disabled={!objectReady} onClick={()=>setActiveStage(1)}>Continue to scene <ChevronDown size={13}/></Button></div>
        </section>}
        {activeStage===1&&<section className="panel overflow-hidden">
          <div className="panel-header"><div><div className="text-sm font-semibold">2 · Collision scene and placement</div><div className="mt-0.5 text-[10px] text-zinc-500">These values define the geometry inserted into the MoveIt planning scene.</div></div><span className="text-[10px] text-zinc-500">{model?.base_frame??"No base frame"}</span></div>
          <div className="space-y-3 p-4">
            <div className="flex flex-wrap items-center justify-between gap-3 rounded-lg border border-zinc-200 px-3 py-2.5"><div><div className="text-xs font-semibold">Baseline collision constraints</div><p className="mt-0.5 text-[10px] text-zinc-500">Ground plane · CR5 pedestal · Box_0. Safe to restore after startup or scene clear.</p>{defaultsStatus&&<p className={`mt-1 text-[10px] ${defaultsError?"text-red-700":"text-emerald-700"}`}>{defaultsStatus}</p>}</div><Button size="sm" variant="outline" disabled={!model||bridgeState!=="connected"||defaultsBusy} onClick={loadDefaultConstraints}><Layers3 size={13}/>{defaultsBusy?"Loading…":"Load 3 default obstacles"}</Button></div>
            <div className="grid gap-3 xl:grid-cols-2">
              <div className="space-y-3">
                <ConfigTable title="Shelf dimensions" description="Choose a preset, then override every dimension. Overrides are used by the preview and MoveIt collision geometry.">
                  <ConfigSelectRow label="Preset" value={shelfId} options={SHELVES.map(item=>({value:item.id,label:item.label}))} onChange={value=>{const preset=SHELVES.find(item=>item.id===value)??SHELVES[1];setShelfId(value);setShelfGeometry({width:preset.width,height:preset.height,depth:preset.depth,levels:preset.levels,board:preset.board});}}/>
                  <ConfigNumberRow label="Outside width" value={shelf.width} unit="mm" min={100} max={3000} onChange={value=>setShelfGeometry(current=>({...current,width:value}))}/>
                  <ConfigNumberRow label="Outside height" value={shelf.height} unit="mm" min={100} max={3000} onChange={value=>setShelfGeometry(current=>({...current,height:value}))}/>
                  <ConfigNumberRow label="Outside depth" value={shelf.depth} unit="mm" min={100} max={2000} onChange={value=>setShelfGeometry(current=>({...current,depth:value}))}/>
                  <ConfigNumberRow label="Number of bays" value={shelf.levels} unit="levels" min={1} max={8} step={1} onChange={value=>setShelfGeometry(current=>({...current,levels:Math.max(1,Math.min(8,Math.round(value)))}))}/>
                  <ConfigNumberRow label="Board thickness" value={shelf.board} unit="mm" min={5} max={100} onChange={value=>setShelfGeometry(current=>({...current,board:value}))}/>
                </ConfigTable>
                <ConfigTable title="Shelf pose" description={`Position and yaw in ${model?.base_frame??"the MoveIt base frame"}`}>
                  <ConfigNumberRow label="X" value={shelfPose.x} unit="mm" onChange={value=>setShelfPose(current=>({...current,x:value}))}/>
                  <ConfigNumberRow label="Y" value={shelfPose.y} unit="mm" onChange={value=>setShelfPose(current=>({...current,y:value}))}/>
                  <ConfigNumberRow label="Z" value={shelfPose.z} unit="mm" onChange={value=>setShelfPose(current=>({...current,z:value}))}/>
                  <ConfigNumberRow label="Yaw" value={shelfPose.yaw} unit="deg" min={-180} max={180} onChange={value=>setShelfPose(current=>({...current,yaw:value}))}/>
                  <ConfigNumberRow label="Bay pick offset X" value={shelfPickOffset.x} unit="mm" min={-1000} max={1000} onChange={value=>setShelfPickOffset(current=>({...current,x:value}))}/>
                  <ConfigNumberRow label="Bay pick offset Y" value={shelfPickOffset.y} unit="mm" min={-1000} max={1000} onChange={value=>setShelfPickOffset(current=>({...current,y:value}))}/>
                </ConfigTable>
              </div>
              <div className="space-y-3">
                <ConfigTable title="Pick and place locations">
                  <ConfigSelectRow label="Pick source" value={pickSource} options={[{value:"shelf",label:"Shelf bay"},{value:"free",label:"Free-space / manual / vision pose"}]} onChange={value=>setPickSource(value as "shelf"|"free")}/>
                  {pickSource==="shelf"&&<ConfigNumberRow label="Pick bay" value={slot+1} unit={`/ ${shelf.levels}`} min={1} max={shelf.levels} step={1} onChange={value=>setSlot(Math.max(0,Math.min(shelf.levels-1,Math.round(value)-1)))}/>}
                  <ConfigSelectRow label="Destination" value={placeDestination} options={[{value:"shelf",label:"Shelf bay"},{value:"platform",label:"Separate platform"}]} onChange={value=>setPlaceDestination(value as "shelf"|"platform")}/>
                  {placeDestination==="shelf"&&<ConfigNumberRow label="Destination bay" value={placeSlot+1} unit={`/ ${shelf.levels}`} min={1} max={shelf.levels} step={1} onChange={value=>setPlaceSlot(Math.max(0,Math.min(shelf.levels-1,Math.round(value)-1)))}/>}
                  <ConfigNumberRow label="Clearance margin" value={clearance} unit="mm" min={0} max={100} step={1} onChange={setClearance}/>
                </ConfigTable>
                {pickSource==="free"&&<div className="rounded-md bg-zinc-50 p-3 text-[10px] leading-4 text-zinc-600">The planner will use the manual or vision centroid entered in Step 3. Ensure the selected frame is the MoveIt base frame.</div>}
                {placeDestination==="platform"&&<>
                  <ConfigTable title="Platform position" description="X/Y are the platform centre. The preset top surface is aligned to base_link Z=0; the base Z field includes platform thickness.">
                    <ConfigNumberRow label="Center X" value={platform.x} unit="mm" onChange={value=>setPlatform(current=>({...current,x:value}))}/>
                    <ConfigNumberRow label="Center Y" value={platform.y} unit="mm" onChange={value=>setPlatform(current=>({...current,y:value}))}/>
                    <ConfigNumberRow label="Base Z" value={platform.z} unit="mm" onChange={value=>setPlatform(current=>({...current,z:value}))}/>
                  </ConfigTable>
                  <ConfigTable title="Platform collision dimensions">
                    <ConfigNumberRow label="Width X" value={platformDimensions.x} unit="mm" min={10} max={2000} onChange={value=>setPlatformDimensions(current=>({...current,x:value}))}/>
                    <ConfigNumberRow label="Depth Y" value={platformDimensions.y} unit="mm" min={10} max={2000} onChange={value=>setPlatformDimensions(current=>({...current,y:value}))}/>
                    <ConfigNumberRow label="Thickness Z" value={platformDimensions.z} unit="mm" min={5} max={500} onChange={value=>setPlatformDimensions(current=>({...current,z:value}))}/>
                  </ConfigTable>
                </>}
              </div>
            </div>
          </div>
          <div className="flex flex-wrap items-center justify-between gap-3 border-t border-zinc-100 px-4 py-3"><span className="text-[10px] text-zinc-500">{scenePrepared?sceneStatus??"Scene prepared":"Prepare defaults, shelf and object collisions before grasp validation."}</span><Button size="sm" disabled={!objectReady||sceneBusy||bridgeState!=="connected"||!shelfGeometryValid||!sceneFitValid} onClick={prepareScene}><Layers3 size={13}/>{sceneBusy?"Preparing scene…":scenePrepared?"Refresh collision scene":"Prepare scene & continue"}</Button></div>
        </section>}
      </div>
      <aside className="space-y-4">{activeStage===2&&<>
<section className="panel overflow-hidden"><div className="panel-header"><div><div className="text-sm font-semibold">3 · Grasp and TCP configuration</div><div className="mt-0.5 text-[10px] text-zinc-500">Pose inputs are expressed in {model?.base_frame??"the MoveIt base frame"}; offsets are applied in the grasp TCP orientation.</div></div><span className="text-[10px] text-zinc-500">{graspStrategy==="pgea_finger_wrap"?"PGEA finger wrap":"Parallel jaw"}</span></div>
  <div className="grid gap-3 p-4 xl:grid-cols-2">
    <div className="space-y-3">
      <ConfigTable title="Object centroid and orientation" description={source==="synthetic"&&pickSource==="shelf"?"Generated from the selected shelf bay. Edit any coordinate to switch to a manual pose.":"Manual / vision pose. For vision, the frame is validated against the MoveIt base frame."}>
        <ConfigNumberRow label="Centroid X" value={activePick.x} unit="mm" onChange={value=>{setPick(current=>({...current,x:value}));setSource("manual");setPickSource("free");}}/>
        <ConfigNumberRow label="Centroid Y" value={activePick.y} unit="mm" onChange={value=>{setPick(current=>({...current,y:value}));setSource("manual");setPickSource("free");}}/>
        <ConfigNumberRow label="Centroid Z" value={activePick.z} unit="mm" onChange={value=>{setPick(current=>({...current,z:value}));setSource("manual");setPickSource("free");}}/>
        <ConfigNumberRow label="Object yaw" value={objectYaw} unit="deg" min={-180} max={180} onChange={setObjectYaw}/>
      </ConfigTable>
      <ConfigTable title="Grasp geometry" description="These settings affect the generated pre-grasp, grasp and withdrawal poses.">
        <ConfigSelectRow label="Grasp strategy" value={graspStrategy} options={[{value:"pgea_finger_wrap",label:"PGEA extended-finger wrap (estimated 85 mm)"},{value:"parallel_jaw",label:"Parallel jaw (40 mm nominal)"}]} onChange={setGraspStrategy}/>
        <ConfigNumberRow label="Gripper yaw" value={gripperAngle} unit="deg" min={-180} max={180} onChange={setGripperAngle}/>
        <ConfigNumberRow label="Gripper roll" value={gripperRoll} unit="deg" min={-180} max={180} onChange={setGripperRoll}/>
        <ConfigNumberRow label="Approach tilt" value={approachAngle} unit="deg" min={-90} max={90} onChange={setApproachAngle}/>
        <ConfigNumberRow label="Pick approach distance" value={approachDistance} unit="mm" min={20} max={500} onChange={setApproachDistance}/>
        <ConfigNumberRow label="Place approach distance" value={placeApproachDistance} unit="mm" min={10} max={300} onChange={setPlaceApproachDistance}/>
        <ConfigNumberRow label="Finger clearance" value={gripperClearanceMm} unit="mm" min={0} max={30} step={0.5} onChange={setGripperClearanceMm}/>
      </ConfigTable>
    </div>
    <div className="space-y-3">
      <ConfigTable title="TCP offset · relative to grasp orientation" description="XYZ offset from the detected object centroid to the selected tcp link target.">
        <ConfigNumberRow label="Offset X" value={tcpOffset.x} unit="mm" min={-200} max={200} step={0.5} onChange={value=>setTcpOffset(current=>({...current,x:value}))}/>
        <ConfigNumberRow label="Offset Y" value={tcpOffset.y} unit="mm" min={-200} max={200} step={0.5} onChange={value=>setTcpOffset(current=>({...current,y:value}))}/>
        <ConfigNumberRow label="Offset Z" value={tcpOffset.z} unit="mm" min={-200} max={200} step={0.5} onChange={value=>setTcpOffset(current=>({...current,z:value}))}/>
      </ConfigTable>
      <div className="rounded-lg border border-zinc-200 p-3"><div className="flex items-center justify-between gap-2"><span className="text-xs font-semibold">Derived grasp TCP · tcp</span><span className="text-[10px] text-zinc-400">{model?.base_frame??"base frame"}</span></div><div className="mt-2 grid grid-cols-3 gap-2 font-mono text-xs">{Object.entries(grasp.tcp).map(([axis,value])=><div key={axis}><div className="text-[10px] text-zinc-400">{axis.toUpperCase()}</div><div>{number(value)} mm</div></div>)}</div><div className="mt-2 flex flex-wrap justify-between gap-2 border-t border-zinc-100 pt-2 text-[10px] text-zinc-500"><span>Yaw {number(grasp.yaw)}°</span><span>Tilt {number(grasp.approach)}°</span><span>Approach {number(grasp.distance)} mm</span></div><Button size="sm" variant="outline" className="mt-3 w-full" onClick={copyGrasp}><ClipboardCopy size={13}/> Copy grasp target JSON</Button></div>
      <details className="rounded-lg border border-zinc-200 p-3"><summary className="cursor-pointer text-[11px] font-medium text-zinc-700">Gripper model assumptions</summary><p className="mt-2 text-[10px] leading-4 text-zinc-500">The 40 mm figure is nominal actuator travel. Custom finger CAD adds about 45 mm of lateral span, giving a preliminary 85 mm wrap envelope. This is a geometry estimate, not a rated grasp width or proof of retention/contact force.</p></details>
    </div>
  </div>
</section>
      <section className="panel"><div className="panel-header"><div className="flex items-center gap-2"><ShieldCheck size={16}/><span className="text-sm font-semibold">Pre-plan validation</span></div><span className="text-[10px] text-zinc-400">{checks.filter(c=>c.ok).length}/{checks.length}</span></div><div className="space-y-3 p-4">{checks.map(c=><div key={c.label} className="flex items-start gap-2"><span className={`mt-0.5 h-2 w-2 rounded-full ${c.ok?"bg-emerald-500":c.label==="Gripper width"?"bg-amber-500":"bg-red-500"}`}/><div className="min-w-0"><div className="text-xs font-medium">{c.label}</div><div className="text-[10px] text-zinc-500">{c.detail}</div></div></div>)}<details className="rounded-md border border-zinc-200 p-3"><summary className="cursor-pointer text-[10px] font-medium text-zinc-700">Read grasp-model limitations</summary><p className="mt-2 text-[10px] leading-4 text-zinc-500">The PGEA's 40 mm figure is nominal actuator travel. Custom finger CAD adds about 45 mm of lateral span, giving an estimated 85 mm envelope. This is not a rated grasp width or proof of contact force, friction or retention. MoveIt path planning is not contact simulation; validate the grasp in Gazebo and with guarded hardware tests.</p></details><Button className="w-full" disabled={!scenePrepared||!allValid} onClick={()=>setActiveStage(3)}>Continue to plan generation <ChevronDown size={14}/></Button>
        {!scenePrepared?<div className="flex flex-wrap items-center justify-between gap-2 rounded-md border border-amber-200 bg-amber-50 px-3 py-2"><p className="text-[10px] text-amber-800">Scene inputs changed. Finish your pose edits, then refresh the collision scene before planning.</p><Button size="sm" variant="outline" onClick={()=>setActiveStage(1)}>Return to Step 2</Button></div>:null}
        </div></section></>}
{activeStage===3&&<>
      <section className="panel"><div className="panel-header"><div className="flex items-center gap-2"><Target size={16}/><span className="text-sm font-semibold">Step 4 · Generate MoveIt plan</span></div><span className="text-[10px] text-zinc-400">PLAN ONLY</span></div>
        <div className="space-y-3 p-4"><p className="text-[11px] leading-5 text-zinc-600">Plan only. These settings are sent to every waypoint request. No robot or Gazebo motion is executed.</p>
          <ConfigTable title="MoveIt planning settings" description="The CR5A pick/place pose pipeline currently uses OMPL. Planner IDs are loaded from cr5_group configuration.">
            <ConfigSelectRow label="OMPL planner" value={plannerSettings.plannerId} options={[{value:"",label:"Use CR5A MoveIt configured default"},{value:"RRTConnect",label:"RRTConnect · fast feasible path"},{value:"RRTstar",label:"RRTstar · path optimization"},{value:"PRMstar",label:"PRMstar · roadmap"}]} onChange={value=>setPlannerSettings(current=>({...current,plannerId:value as ""|"RRTConnect"|"RRTstar"|"PRMstar"}))}/>
            <ConfigNumberRow label="Allowed planning time" value={plannerSettings.allowedPlanningTime} unit="s" min={0.1} max={60} step={0.5} onChange={value=>setPlannerSettings(current=>({...current,allowedPlanningTime:Math.max(.1,Math.min(60,value))}))}/>
            <ConfigNumberRow label="Planning attempts" value={plannerSettings.numPlanningAttempts} unit="tries" min={1} max={20} step={1} onChange={value=>setPlannerSettings(current=>({...current,numPlanningAttempts:Math.max(1,Math.min(20,Math.round(value)))}))}/>
            <ConfigNumberRow label="Velocity scaling" value={plannerSettings.maxVelocityScaling} unit="0–1" min={0.01} max={1} step={0.01} onChange={value=>setPlannerSettings(current=>({...current,maxVelocityScaling:Math.max(.01,Math.min(1,value))}))}/>
            <ConfigNumberRow label="Acceleration scaling" value={plannerSettings.maxAccelerationScaling} unit="0–1" min={0.01} max={1} step={0.01} onChange={value=>setPlannerSettings(current=>({...current,maxAccelerationScaling:Math.max(.01,Math.min(1,value))}))}/>
            <ConfigNumberRow label="Position tolerance" value={plannerSettings.positionToleranceMm} unit="mm" min={0.1} max={50} step={0.1} onChange={value=>setPlannerSettings(current=>({...current,positionToleranceMm:Math.max(.1,Math.min(50,value))}))}/>
            <ConfigNumberRow label="Orientation tolerance" value={plannerSettings.orientationToleranceDeg} unit="deg" min={0.1} max={30} step={0.1} onChange={value=>setPlannerSettings(current=>({...current,orientationToleranceDeg:Math.max(.1,Math.min(30,value))}))}/>
            <ConfigNumberRow label="Additional lift distance" value={plannerSettings.liftDistanceMm} unit="mm" min={0} max={500} step={5} onChange={value=>setPlannerSettings(current=>({...current,liftDistanceMm:Math.max(0,Math.min(500,value))}))}/>
          </ConfigTable>
          <div className="rounded-md bg-zinc-50 p-3 text-[10px] leading-5 text-zinc-600"><div className="font-semibold text-zinc-800">Plan inputs</div><div>Object: {object.label}</div><div>Pick: {pickSource==="shelf"?`Shelf bay ${slot+1}`:"Free-space / vision pose"}</div><div>Place: {placeDestination==="shelf"?`Shelf bay ${placeSlot+1}`:"Platform"}</div><div>Target TCP: {number(grasp.tcp.x)} / {number(grasp.tcp.y)} / {number(grasp.tcp.z)} mm</div><div>Validation: {checks.filter(item=>item.ok).length}/{checks.length} passed</div></div>
          <Button className="w-full" disabled={!scenePrepared||!allValid||!model||bridgeState!=="connected"||planResult?.status==="PLANNING"} onClick={runPlan}><Target size={14}/>{planResult?.status==="PLANNING"?"Planning task…":planReady?"Re-plan task":"Generate pick & place plan"}</Button>
          {!scenePrepared?<p className="text-[10px] text-amber-700">Return to Step 2 and prepare the current collision scene.</p>:!allValid?<p className="text-[10px] text-amber-700">Return to Step 3 and resolve the failed validation checks.</p>:null}
          {planResult?<div className="space-y-2 rounded-md border border-zinc-200 p-3"><div className="flex items-center justify-between"><span className="text-[10px] font-semibold uppercase tracking-wider">MoveIt task result</span><span className={`rounded px-2 py-1 text-[9px] font-semibold ${planResult.status==="PLANNED"?"bg-emerald-50 text-emerald-700":planResult.status==="PLANNING"?"bg-blue-50 text-blue-700":"bg-red-50 text-red-700"}`}>{planResult.status}</span></div>{planResult.error?<p className="text-[10px] text-red-700">{planResult.error}</p>:null}{planResult.failed_stage?<p className="text-[10px] text-red-700">Failed at: {planResult.failed_stage} · MoveIt error {planResult.error_code??"—"}</p>:null}{Array.isArray(planResult.plans)?planResult.plans.map((stage:any)=><div key={stage.name} className="flex justify-between text-[10px]"><span className="capitalize">{stage.name}</span><span className={stage.status==="SUCCEEDED"?"text-emerald-700":"text-red-700"}>{stage.status} {stage.planning_time?`· ${number(stage.planning_time,2)} s`:""}</span></div>):null}{planResult.planning_time_total!==undefined?<div className="border-t border-zinc-100 pt-2 text-[10px]">Total planning time <b>{number(planResult.planning_time_total,2)} s</b></div>:null}{planResult.payload_model?<div className="space-y-1 border-t border-zinc-100 pt-2 text-[10px]"><div className="font-semibold">Payload-aware MoveIt timing</div><div className="flex justify-between"><span>Combined payload</span><b>{number(planResult.payload_model.total_mass_kg,3)} kg</b></div><div className="flex justify-between"><span>Velocity scaling</span><b>{number(planResult.payload_model.max_velocity_scaling*100,1)}%</b></div><div className="flex justify-between"><span>Acceleration scaling</span><b>{number(planResult.payload_model.max_acceleration_scaling*100,1)}%</b></div><div className="text-zinc-400">{planResult.payload_model.heuristic}</div></div>:null}</div>:null}
          {planReady?<div className="rounded-md border border-emerald-200 bg-emerald-50 px-3 py-2 text-[10px] text-emerald-800">All returned planning stages are marked planned. Execution remains a separate, guarded operation.</div>:planResult?.status==="PLANNED"?<div className="rounded-md border border-amber-200 bg-amber-50 px-3 py-2 text-[10px] text-amber-800">Inputs changed after this plan was generated. Generate a new plan before using these results.</div>:null}
        </div>
      </section>
<details className="panel overflow-hidden"><summary className="flex cursor-pointer list-none items-center justify-between gap-3 px-4 py-3"><span><span className="block text-xs font-semibold">Advanced · Controller payload</span><span className="mt-0.5 block text-[10px] text-zinc-500">Tool mass, COM and explicit SetPayload action · not required for planning</span></span><ChevronDown size={14}/></summary>
  <div className="space-y-3 border-t border-zinc-100 p-4">
    <ConfigTable title="Tool and payload limits">
      <ConfigNumberRow label="Tool assembly mass" value={toolMassKg} unit="kg" min={0} max={4.5} step={0.01} onChange={value=>{setToolMassKg(value);setControllerToolVerified(false);}}/>
      <ConfigNumberRow label="Maximum combined payload" value={maxPayloadKg} unit="kg" min={0.1} max={5} step={0.1} onChange={value=>setMaxPayloadKg(Math.max(.1,Math.min(5,value)))}/>
      <tr><th className="px-3 py-2 font-medium text-zinc-700">Object mass</th><td className="px-2 py-2 font-mono">{number(massKg,3)}</td><td className="px-3 text-right text-[10px] text-zinc-400">kg</td></tr>
      <tr><th className="px-3 py-2 font-medium text-zinc-700">Combined mass</th><td className="px-2 py-2 font-mono font-semibold">{number(totalPayloadKg,3)}</td><td className="px-3 text-right text-[10px] text-zinc-400">kg</td></tr>
    </ConfigTable>
    <ConfigTable title="Tool COM relative to TCP" description="Edit the tool assembly COM. The combined tool + object COM is calculated below.">
      <ConfigNumberRow label="Tool COM X" value={toolCom.x} unit="mm" min={-500} max={500} step={0.1} onChange={value=>{setToolCom(current=>({...current,x:value}));setControllerToolVerified(false);}}/>
      <ConfigNumberRow label="Tool COM Y" value={toolCom.y} unit="mm" min={-500} max={500} step={0.1} onChange={value=>{setToolCom(current=>({...current,y:value}));setControllerToolVerified(false);}}/>
      <ConfigNumberRow label="Tool COM Z" value={toolCom.z} unit="mm" min={-500} max={500} step={0.1} onChange={value=>{setToolCom(current=>({...current,z:value}));setControllerToolVerified(false);}}/>
    </ConfigTable>
    <div className="rounded-md bg-zinc-50 p-3"><div className="text-[10px] font-semibold uppercase tracking-wider text-zinc-500">Combined COM · calculated</div><div className="mt-2 grid grid-cols-3 gap-2 font-mono text-xs">{Object.entries(payloadCom).map(([axis,value])=><div key={axis}><span className="text-zinc-400">{axis.toUpperCase()}</span><div>{number(value,1)} mm</div></div>)}</div></div>
    <p className="text-[10px] leading-4 text-zinc-500">The default 1.30 kg tool value is an estimate from the selected URDF inertial masses (connector + PGEA body + two fingers). Verify the installed tool and controller Tool0 frame before applying. This payload model is not a torque or stability certification.</p>
    <label className="flex items-start gap-2 text-[10px] text-amber-800"><input type="checkbox" checked={controllerToolVerified} onChange={event=>setControllerToolVerified(event.currentTarget.checked)} className="mt-0.5"/><span>I verified the controller Tool0 frame and total tool mass against the mounted hardware.</span></label>
    <Button size="sm" className="w-full" disabled={bridgeState!=="connected"||!controllerToolVerified||payloadBusy||totalPayloadKg>maxPayloadKg} onClick={applyPayload}><ShieldCheck size={13}/>{payloadBusy?"Applying controller payload…":"Apply payload with SetPayload"}</Button>
    {totalPayloadKg>maxPayloadKg&&<p className="text-[10px] text-red-700">Combined payload exceeds the selected {number(maxPayloadKg,1)} kg limit (maximum configurable CR5 limit is 5 kg).</p>}
    {payloadStatus&&<p className="break-words text-[10px] text-zinc-600">{payloadStatus}</p>}
    <p className="text-[10px] text-zinc-400">The bridge may still reject this command when robot.allow_commands is disabled. It is never applied automatically during planning.</p>
  </div>
</details></>}
      {activeStage===4&&<section className="panel overflow-hidden">
        <div className="panel-header"><div><div className="text-sm font-semibold">5 · Pick</div><div className="mt-0.5 text-[10px] text-zinc-500">Approach → grasp pose → close PGEA → withdraw → lift.</div></div><span className="text-[10px] font-semibold text-zinc-500">{pickExecution?.status??"READY"}</span></div>
        <div className="space-y-3 p-4">
          <div className="rounded-lg border border-amber-200 bg-amber-50 p-3 text-[11px] leading-5 text-amber-900"><div className="font-semibold">Simulation execution only</div>The bridge must report simulation mode and explicitly enable both motion and gripper commands. The hardware bridge rejects this endpoint.</div>
          <div className="grid grid-cols-2 gap-2 rounded-lg bg-zinc-50 p-3 text-[11px]">
            <div><div className="text-zinc-400">Simulation</div><b>{simulation===true?"Active":"Not active"}</b></div>
            <div><div className="text-zinc-400">Bridge</div><b>{bridgeState}</b></div>
            <div><div className="text-zinc-400">MoveIt execution</div><b>{capabilities.motion_execution_enabled?"Enabled":"Disabled"}</b></div>
            <div><div className="text-zinc-400">Gripper commands</div><b>{capabilities.gripper_commands_enabled?"Enabled":"Disabled"}</b></div>
          </div>
          <div className="space-y-2 rounded-lg border border-zinc-200 p-3">
            {["pregrasp","grasp","close_gripper","withdraw","lift"].map((name,index)=>{
              const item=(pickExecution?.completed??[]).find((entry:any)=>entry.step===name);
              return <div key={name} className="flex items-center justify-between gap-3 border-b border-zinc-100 py-1.5 last:border-0"><span className="text-xs font-medium">{index+1}. {name.replace(/_/g," ")}</span><span className={item?.status==="SUCCEEDED"?"text-[10px] font-semibold text-emerald-700":item?.status==="FAILED"?"text-[10px] font-semibold text-red-700":"text-[10px] font-semibold text-zinc-400"}>{item?.status??"Pending"}</span></div>;
            })}
          </div>
          {pickExecution?.error&&<div className="rounded-md border border-red-200 bg-red-50 p-3 text-[10px] text-red-800">Pick stopped: {pickExecution.error}. {pickExecution.completed?.length?"Partial motion was executed. Inspect and re-home the simulator before replanning; do not restart this phase from the old plan.":"No confirmed phase step completed."}</div>}
          <label className="flex items-start gap-2 rounded-md border border-zinc-200 p-3 text-[10px] leading-4 text-zinc-700"><input type="checkbox" checked={simulationConfirmed} onChange={event=>setSimulationConfirmed(event.currentTarget.checked)} className="mt-0.5"/><span>I confirm this is the CR5A simulation bridge and approve the planned pick motion.</span></label>
          <Button className="w-full" disabled={!planReady||!executionAvailable||!simulationConfirmed||!!executionBusy||pickExecution?.status==="FAILED"||pickDone} onClick={()=>executePhase("pick")}>{executionBusy==="pick"?"Picking…":pickDone?"Pick complete":"Run simulated pick"}</Button>
          {!executionAvailable&&<p className="text-[10px] text-zinc-500">Execution is locked until simulation mode, the bridge connection, MoveIt execution and gripper command capability are all confirmed.</p>}
        </div>
      </section>}
      {activeStage===5&&<section className="panel overflow-hidden">
        <div className="panel-header"><div><div className="text-sm font-semibold">6 · Place</div><div className="mt-0.5 text-[10px] text-zinc-500">Transfer → pre-place → lower → open PGEA → retreat.</div></div><span className="text-[10px] font-semibold text-zinc-500">{placeExecution?.status??"READY"}</span></div>
        <div className="space-y-3 p-4">
          <div className="rounded-lg border border-zinc-200 p-3 text-[11px] leading-5 text-zinc-600">The pick phase has {pickExecution?.completed?.length??0} reported completed actions. Placement uses the planned platform or shelf target from Step 2.</div>
          <div className="space-y-2 rounded-lg border border-zinc-200 p-3">
            {["preplace","place","open_gripper","retreat"].map((name,index)=>{
              const item=(placeExecution?.completed??[]).find((entry:any)=>entry.step===name);
              return <div key={name} className="flex items-center justify-between gap-3 border-b border-zinc-100 py-1.5 last:border-0"><span className="text-xs font-medium">{index+1}. {name.replace(/_/g," ")}</span><span className={item?.status==="SUCCEEDED"?"text-[10px] font-semibold text-emerald-700":item?.status==="FAILED"?"text-[10px] font-semibold text-red-700":"text-[10px] font-semibold text-zinc-400"}>{item?.status??"Pending"}</span></div>;
            })}
          </div>
          {placeExecution?.error&&<div className="rounded-md border border-red-200 bg-red-50 p-3 text-[10px] text-red-800">Place stopped: {placeExecution.error}. {placeExecution.completed?.length?"Partial motion was executed. Inspect and re-home the simulator before replanning; do not restart this phase from the old plan.":""}</div>}
          <Button className="w-full" disabled={!planReady||!pickDone||!executionAvailable||!simulationConfirmed||!!executionBusy||placeExecution?.status==="FAILED"||placeDone} onClick={()=>executePhase("place")}>{executionBusy==="place"?"Placing…":placeDone?"Place complete":"Run simulated place and release"}</Button>
          <p className="text-[10px] text-zinc-500">This phase is available only after the pick phase succeeds and still requires the simulation confirmation from Step 5.</p>
        </div>
      </section>}
      {activeStage===6&&<section className="panel overflow-hidden">
        <div className="panel-header"><div><div className="text-sm font-semibold">7 · Verify</div><div className="mt-0.5 text-[10px] text-zinc-500">Read the robot and gripper state, then confirm the physical scene.</div></div><span className="text-[10px] font-semibold text-zinc-500">{verification?.confirmed?"CONFIRMED":"REVIEW"}</span></div>
        <div className="space-y-3 p-4">
          <div className="grid grid-cols-2 gap-2 rounded-lg bg-zinc-50 p-3 text-[11px]"><div><div className="text-zinc-400">Pick phase</div><b>{pickExecution?.status??"Not run"}</b></div><div><div className="text-zinc-400">Place phase</div><b>{placeExecution?.status??"Not run"}</b></div></div>
          <Button size="sm" variant="outline" className="w-full" disabled={!executionAvailable||verificationBusy} onClick={refreshVerification}>{verificationBusy?"Refreshing…":"Refresh robot and gripper state"}</Button>
          {verification?.error&&<div className="text-[10px] text-red-700">{verification.error}</div>}
          {verification?.snapshot&&<div className="rounded-lg border border-zinc-200 p-3 text-[10px]"><div className="font-semibold text-zinc-800">Robot snapshot</div><pre className="mt-2 max-h-40 overflow-auto whitespace-pre-wrap break-words text-[9px] text-zinc-600">{JSON.stringify({connected:verification.snapshot.connected,enabled:verification.snapshot.enabled,simulation:verification.snapshot.simulation,joints:verification.snapshot.joints},null,2)}</pre><div className="mt-2 font-semibold text-zinc-800">Gripper status</div><pre className="mt-1 max-h-28 overflow-auto whitespace-pre-wrap break-words text-[9px] text-zinc-600">{JSON.stringify(verification.gripperStatus,null,2)}</pre></div>}
          <label className="flex items-start gap-2 rounded-md border border-zinc-200 p-3 text-[10px] leading-4 text-zinc-700"><input type="checkbox" checked={verification?.confirmed===true} onChange={event=>confirmPlacement(event.currentTarget.checked)} className="mt-0.5"/><span>I verified that the object is resting on the selected destination and the gripper has cleared it.</span></label>
          {verification?.confirmed?<div className="rounded-md border border-emerald-200 bg-emerald-50 p-3 text-[11px] text-emerald-800">Pick, place and operator verification are complete.</div>:<p className="text-[10px] text-zinc-500">Telemetry is supporting evidence only. Confirm placement after checking the 3D scene / simulator; do not infer successful placement from a command response alone.</p>}
        </div>
      </section>}
      </aside>
      {activeStage>=2&&<div className="flex flex-wrap items-center justify-between gap-2 rounded-lg border border-zinc-200 bg-white px-4 py-3">
        <Button size="sm" variant="outline" onClick={()=>setActiveStage(Math.max(0,activeStage-1))}>Back</Button>
        <span className="text-[10px] text-zinc-500">Step {activeStage+1} of 7 · {stageItems[activeStage]?.title}</span>
        {activeStage===2?<Button size="sm" disabled={!scenePrepared||!allValid} onClick={()=>setActiveStage(3)}>Continue to plan <ChevronDown size={13}/></Button>
        :activeStage===3?<Button size="sm" disabled={!planReady} onClick={()=>setActiveStage(4)}>Continue to pick <ChevronDown size={13}/></Button>
        :activeStage===4?<Button size="sm" disabled={!pickDone} onClick={()=>setActiveStage(5)}>Continue to place <ChevronDown size={13}/></Button>
        :activeStage===5?<Button size="sm" disabled={!placeDone} onClick={()=>setActiveStage(6)}>Continue to verify <ChevronDown size={13}/></Button>
        :<Button size="sm" variant="outline" onClick={()=>setActiveStage(5)}>Back to place</Button>}
      </div>}
      </div>
      </div>
    </details>
  </div>;
}
