// Alternative compact workbench layout preserved for UI comparison.
import {useMemo,useState} from "react";
import {ArrowDownToLine,ArrowUpFromLine,Box,CheckCircle2,ClipboardPaste,Layers3,Play,RefreshCw,Settings2} from "lucide-react";
import {Button} from "@/components/ui/button";
import {useRobotSocket} from "@/lib/socket";
import {useRobotStore} from "@/store/robot-store";

type Vec3={x:number;y:number;z:number};
type ObjectKind="coca_cola_can"|"pringles_can"|"wood_box";
type InputSource="synthetic"|"vision";
type SceneItem={id:string;shape:"box"|"cylinder";position_m:[number,number,number];dimensions_m?:[number,number,number];height_m?:number;radius_m?:number;role:string};

const OBJECTS:Record<ObjectKind,{label:string;shape:"cylinder"|"box";dimensions:[number,number,number];massKg:number;massBasis:string;graspWidth:number}>={
  coca_cola_can:{label:"Coca-Cola 330 ml can",shape:"cylinder",dimensions:[0.066,0.066,0.122],massKg:0.343,massBasis:"330 g drink + 13 g empty can (estimate)",graspWidth:0.066},
  pringles_can:{label:"Pringles 165 g tube",shape:"cylinder",dimensions:[0.076,0.076,0.255],massKg:0.212,massBasis:"165 g contents + approx. 47 g tube/lid (estimate)",graspWidth:0.076},
  wood_box:{label:"Wood box",shape:"box",dimensions:[0.080,0.060,0.120],massKg:0.0,massBasis:"Calculated from pine density × volume",graspWidth:0.060}
};
const SHELF_PRESETS={compact:{label:"Compact",width:0.42,depth:0.32,height:0.38},standard:{label:"Standard",width:0.60,depth:0.42,height:0.50},wide:{label:"Wide",width:0.80,depth:0.55,height:0.60}} as const;
const num=(v:number)=>Number(v.toFixed(3));
const quatFromRpy=(roll:number,pitch:number,yaw:number):[number,number,number,number]=>{
  const r=roll*Math.PI/360,p=pitch*Math.PI/360,y=yaw*Math.PI/360;
  const cr=Math.cos(r),sr=Math.sin(r),cp=Math.cos(p),sp=Math.sin(p),cy=Math.cos(y),sy=Math.sin(y);
  return [sr*cp*cy-cr*sp*sy,cr*sp*cy+sr*cp*sy,cr*cp*sy-sr*sp*cy,cr*cp*cy+sr*sp*sy];
};
function Field({label,value,onChange,step="0.01",unit}:{label:string;value:number;onChange:(v:number)=>void;step?:string;unit?:string}){
  return <label className="block min-w-0"><span className="mb-1 block text-[10px] font-medium uppercase tracking-wider text-zinc-500">{label}</span><div className="flex h-9 items-center rounded-md border border-zinc-200 bg-white"><input type="number" step={step} value={value} onChange={e=>onChange(Number(e.target.value))} className="min-w-0 flex-1 bg-transparent px-2.5 font-mono text-sm outline-none"/><span className="px-2 text-[10px] text-zinc-400">{unit}</span></div></label>;
}

export function PickPlaceAlternativePage(){
  const {request}=useRobotSocket();
  const model=useRobotStore(s=>s.model);
  const bridgeState=useRobotStore(s=>s.bridgeState);
  const jointState=useRobotStore(s=>s.joints);
  const [source,setSource]=useState<InputSource>("synthetic");
  const [objectKind,setObjectKind]=useState<ObjectKind>("coca_cola_can");
  const [shelfPreset,setShelfPreset]=useState<keyof typeof SHELF_PRESETS>("standard");
  const [shelf,setShelf]=useState({width:0.60,depth:0.42,height:0.50,wall:0.012,clearance:0.012});
  const [platform,setPlatform]=useState({x:0.55,y:0.0,z:0.025,width:0.30,depth:0.24,height:0.05});
  const [object,setObject]=useState({x:0.0,y:0.0,z:0.085,yaw:0});
  const [woodDensity,setWoodDensity]=useState(510);
  const [massOverride,setMassOverride]=useState<number|null>(null);
  const [grasp,setGrasp]=useState({yaw:0,approachPitch:0,approachRoll:0,approachDistance:0.10,liftDistance:0.10,prePlaceDistance:0.08,gripperClearance:0.006});
  const [gripperMaxOpening,setGripperMaxOpening]=useState(0.040);
  const [visionJson,setVisionJson]=useState("");
  const [sceneIds,setSceneIds]=useState<string[]>([]);
  const [busy,setBusy]=useState<string|null>(null);
  const [notice,setNotice]=useState<{kind:"success"|"error"|"info";message:string}|null>(null);
  const [lastPlan,setLastPlan]=useState<any>(null);
  const selected=OBJECTS[objectKind];
  const objectDims=selected.dimensions;
  const objectMass=massOverride??(objectKind==="wood_box"?objectDims[0]*objectDims[1]*objectDims[2]*woodDensity:selected.massKg);
  const graspWidth=selected.graspWidth;
  const gripFits=graspWidth+grasp.gripperClearance<=gripperMaxOpening;
  const frame=model?.base_frame??"dummy_link";
  const shelfInner={width:shelf.width-2*shelf.wall,depth:shelf.depth-shelf.wall,height:shelf.height-shelf.wall};
  const objectFits=shelfInner.width>=objectDims[0]+2*shelf.clearance&&shelfInner.depth>=objectDims[1]+shelf.clearance&&shelfInner.height>=objectDims[2]+shelf.clearance;
  const payloadForce=objectMass*9.80665;
  const payloadTorque=payloadForce*0.10;
  const items=useMemo<SceneItem[]>(()=>{
    const w=shelf.width,d=shelf.depth,h=shelf.height,t=shelf.wall;
    const ox=-w/2,oy=-d/2;
    const base:SceneItem[]=[
      {id:"pp_shelf_floor",shape:"box",position_m:[ox+w/2,oy+d/2,-t/2],dimensions_m:[w,d,t],role:"shelf"},
      {id:"pp_shelf_left",shape:"box",position_m:[ox+t/2,oy+d/2,h/2],dimensions_m:[t,d,h],role:"shelf"},
      {id:"pp_shelf_right",shape:"box",position_m:[ox+w-t/2,oy+d/2,h/2],dimensions_m:[t,d,h],role:"shelf"},
      {id:"pp_shelf_back",shape:"box",position_m:[ox+w/2,oy+d-t/2,h/2],dimensions_m:[w,t,h],role:"shelf"},
      {id:"pp_platform",shape:"box",position_m:[platform.x,platform.y,platform.z-platform.height/2],dimensions_m:[platform.width,platform.depth,platform.height],role:"platform"}
    ];
    const obj:SceneItem=selected.shape==="cylinder"
      ?{id:"pp_pick_object",shape:"cylinder",position_m:[object.x,object.y,object.z],height_m:objectDims[2],radius_m:objectDims[0]/2,role:"object"}
      :{id:"pp_pick_object",shape:"box",position_m:[object.x,object.y,object.z],dimensions_m:[...objectDims],role:"object"};
    return [...base,obj];
  },[shelf,platform,object,selected,objectDims]);
  const applyScene=async()=>{
    if(source==="vision"){
      try{const parsed=JSON.parse(visionJson);if(!Array.isArray(parsed.objects)||parsed.objects.length===0)throw new Error("Expected {objects:[...]}");}
      catch(e){setNotice({kind:"error",message:e instanceof Error?e.message:"Invalid vision JSON"});return;}
      setNotice({kind:"info",message:"Vision JSON parsed. Vision-to-scene normalization is being implemented in the next planner phase."});return;
    }
    setBusy("scene");setNotice(null);
    try{
      const created:string[]=[];
      for(const item of items){
        const payload:any={id:item.id,shape:item.shape,frame_id:frame,position_m:item.position_m,orientation_xyzw:[0,0,0,1]};
        if(item.shape==="box")payload.dimensions_m=item.dimensions_m;
        else {payload.height_m=item.height_m;payload.radius_m=item.radius_m;}
        const result=await request("scene.add",payload);
        if(result?.id)created.push(result.id);
      }
      setSceneIds(created);setNotice({kind:"success",message:`Added ${created.length} generated collision objects to MoveIt (${frame}).`});
    }catch(e){setNotice({kind:"error",message:e instanceof Error?e.message:String(e)});}
    finally{setBusy(null);}
  };
  const removeGeneratedScene=async()=>{
    if(!sceneIds.length)return;setBusy("remove");
    try{for(const id of sceneIds)await request("scene.remove",{id});setSceneIds([]);setLastPlan(null);setNotice({kind:"success",message:"Generated scene objects removed."});}
    catch(e){setNotice({kind:"error",message:e instanceof Error?e.message:String(e)});}finally{setBusy(null);}
  };
  const planPose=async(label:string,position:[number,number,number],rpy:[number,number,number])=>{
    if(bridgeState!=="connected"||!model){setNotice({kind:"error",message:"Bridge and MoveIt robot model must be connected before planning."});return;}
    if(!jointState?.position?.length){setNotice({kind:"error",message:"Current joint telemetry is required for planning."});return;}
    setBusy("plan");setNotice(null);
    try{
      const result=await request("motion.plan",{group_name:model.group_name,target:{type:"pose",frame_id:frame,link_name:model.end_effector_link,position_m:position,orientation_xyzw:quatFromRpy(...rpy),position_tolerance_m:0.003,orientation_tolerance_rad:0.03},allowed_planning_time:8,max_velocity_scaling:0.12,max_acceleration_scaling:0.12});
      setLastPlan(result);setNotice({kind:result.status==="SUCCEEDED"?"success":"error",message:`${label}: ${result.status??"UNKNOWN"}${result.error_code!==undefined?` (MoveIt ${result.error_code})`:""}.`});
    }catch(e){setNotice({kind:"error",message:e instanceof Error?e.message:String(e)});}finally{setBusy(null);}
  };
  const planGrasp=()=>{
    const approachYaw=object.yaw+grasp.yaw;
    const q=quatFromRpy(grasp.approachRoll,grasp.approachPitch,approachYaw);
    const [qx,qy,qz,qw]=q;
    const direction=[2*(qx*qz+qw*qy),2*(qy*qz-qw*qx),1-2*(qx*qx+qy*qy)];
    const target:[number,number,number]=[object.x,object.y,object.z];
    const pre:[number,number,number]=[target[0]-direction[0]*grasp.approachDistance,target[1]-direction[1]*grasp.approachDistance,target[2]-direction[2]*grasp.approachDistance];
    return planPose("Pre-grasp approach",pre,[grasp.approachRoll,grasp.approachPitch,approachYaw]);
  };
  const planPlace=()=>{
    const x=platform.x,y=platform.y,z=platform.z+platform.height/2+objectDims[2]/2+shelf.clearance;
    return planPose("Place target",[x,y,z],[0,0,grasp.yaw]);
  };
  const loadPreset=(key:keyof typeof SHELF_PRESETS)=>{setShelfPreset(key);const p=SHELF_PRESETS[key];setShelf(v=>({...v,width:p.width,depth:p.depth,height:p.height}));};
  const loadVision=()=>{try{const parsed=JSON.parse(visionJson);const first=parsed.objects?.[0];if(!first)throw new Error("No objects found");const p=first.centroid_m??first.position_m;if(!Array.isArray(p)||p.length!==3)throw new Error("First object needs centroid_m or position_m [x,y,z]");setObject(v=>({...v,x:p[0],y:p[1],z:p[2],yaw:first.yaw_deg??0}));setNotice({kind:"success",message:"First vision detection loaded into the object target fields."});}catch(e){setNotice({kind:"error",message:e instanceof Error?e.message:String(e)});}};

  return <div className="space-y-5">
    <div className="flex flex-wrap items-end justify-between gap-3"><div><div className="eyebrow">Manipulation workbench / experimental</div><h1 className="page-title">Pick & Place Planner</h1><p className="mt-1 text-sm text-zinc-500">Generate test scenes, derive grasp poses and plan collision-aware object transfers.</p></div><div className="flex items-center gap-2 rounded-md border border-zinc-200 bg-white px-3 py-2 text-xs"><span className={`h-2 w-2 rounded-full ${bridgeState==="connected"?"bg-emerald-500":"bg-zinc-300"}`}/>{bridgeState==="connected"?"Bridge connected":"Bridge disconnected"}<span className="border-l border-zinc-200 pl-2 text-zinc-400">Frame: {frame}</span></div></div>
    {notice&&<div className={`notice ${notice.kind==="success"?"notice-success":notice.kind==="error"?"notice-error":""}`}>{notice.message}</div>}
    <div className="grid gap-4 xl:grid-cols-[minmax(0,1.55fr)_minmax(340px,0.85fr)]">
      <section className="panel overflow-hidden"><div className="panel-header"><div><div className="eyebrow">01 / Input source</div><h2 className="mt-1 text-sm font-semibold">Scene and object input</h2></div><div className="flex rounded-md bg-zinc-100 p-1"><button onClick={()=>setSource("synthetic")} className={`rounded px-3 py-1.5 text-xs ${source==="synthetic"?"bg-white shadow-sm":"text-zinc-500"}`}>Synthetic</button><button onClick={()=>setSource("vision")} className={`rounded px-3 py-1.5 text-xs ${source==="vision"?"bg-white shadow-sm":"text-zinc-500"}`}>Vision JSON</button></div></div>
        <div className="space-y-5 p-5">{source==="synthetic"?<>
          <div><label className="mb-2 block text-xs font-medium text-zinc-600">Object to pick</label><div className="grid gap-2 sm:grid-cols-3">{(Object.keys(OBJECTS) as ObjectKind[]).map(key=><button key={key} onClick={()=>{setObjectKind(key);setMassOverride(null);}} className={`rounded-lg border p-3 text-left ${objectKind===key?"border-zinc-900 bg-zinc-50":"border-zinc-200 hover:border-zinc-400"}`}><div className="mb-2 flex h-8 w-8 items-center justify-center rounded-md bg-zinc-100"><Box size={17}/></div><div className="text-xs font-semibold">{OBJECTS[key].label}</div><div className="mt-1 text-[10px] text-zinc-500">{OBJECTS[key].shape} · {(OBJECTS[key].dimensions[2]*1000).toFixed(0)} mm high</div></button>)}</div></div>
          <div className="grid gap-3 sm:grid-cols-2"><div><label className="mb-1 block text-[10px] font-medium uppercase tracking-wider text-zinc-500">Shelf preset</label><select value={shelfPreset} onChange={e=>loadPreset(e.target.value as keyof typeof SHELF_PRESETS)} className="h-9 w-full rounded-md border border-zinc-200 bg-white px-2 text-sm">{Object.entries(SHELF_PRESETS).map(([k,v])=><option key={k} value={k}>{v.label} · {Math.round(v.width*1000)} × {Math.round(v.depth*1000)} × {Math.round(v.height*1000)} mm</option>)}</select></div><div><label className="mb-1 block text-[10px] font-medium uppercase tracking-wider text-zinc-500">Wood density</label><div className="flex h-9 items-center rounded-md border border-zinc-200"><input type="number" value={woodDensity} onChange={e=>setWoodDensity(Number(e.target.value))} className="w-full bg-transparent px-2 text-sm"/><span className="px-2 text-[10px] text-zinc-400">kg/m³</span></div></div></div>
          <div><div className="mb-2 flex items-center justify-between"><span className="text-xs font-semibold">Shelf interior</span><span className="text-[10px] text-zinc-400">millimetres</span></div><div className="grid grid-cols-2 gap-3 sm:grid-cols-3"><Field label="Width" value={shelf.width*1000} unit="mm" onChange={v=>setShelf(s=>({...s,width:v/1000}))}/><Field label="Depth" value={shelf.depth*1000} unit="mm" onChange={v=>setShelf(s=>({...s,depth:v/1000}))}/><Field label="Height" value={shelf.height*1000} unit="mm" onChange={v=>setShelf(s=>({...s,height:v/1000}))}/><Field label="Wall thickness" value={shelf.wall*1000} unit="mm" onChange={v=>setShelf(s=>({...s,wall:v/1000}))}/><Field label="Placement clearance" value={shelf.clearance*1000} unit="mm" onChange={v=>setShelf(s=>({...s,clearance:v/1000}))}/></div></div>
          <div><div className="mb-2 text-xs font-semibold">Object centroid in base frame</div><div className="grid grid-cols-2 gap-3 sm:grid-cols-4"><Field label="X" value={object.x*1000} unit="mm" onChange={v=>setObject(o=>({...o,x:v/1000}))}/><Field label="Y" value={object.y*1000} unit="mm" onChange={v=>setObject(o=>({...o,y:v/1000}))}/><Field label="Z" value={object.z*1000} unit="mm" onChange={v=>setObject(o=>({...o,z:v/1000}))}/><Field label="Object yaw" value={object.yaw} unit="deg" step="1" onChange={v=>setObject(o=>({...o,yaw:v}))}/></div></div>
        </>:<div className="space-y-3"><p className="text-xs leading-5 text-zinc-500">Paste normalized detections from the vision module. Accepted object fields: <code>centroid_m</code> or <code>position_m</code>, optional <code>dimensions_m</code>, <code>yaw_deg</code>, and <code>class_name</code>.</p><textarea value={visionJson} onChange={e=>setVisionJson(e.target.value)} rows={9} placeholder={'{"frame_id":"dummy_link","objects":[{"class_name":"can","centroid_m":[0.4,0.0,0.2],"dimensions_m":[0.066,0.066,0.122],"yaw_deg":0}]}'} className="w-full rounded-md border border-zinc-200 p-3 font-mono text-xs outline-none focus:border-zinc-500"/><div className="flex gap-2"><Button variant="outline" size="sm" onClick={loadVision}><ClipboardPaste size={13}/>Load first detection</Button><span className="self-center text-[10px] text-zinc-400">Adapter contract: frame-aware metres and radians/degrees</span></div></div>}
          <div className="flex flex-wrap gap-2 border-t border-zinc-100 pt-4"><Button onClick={applyScene} disabled={busy!==null||source!=="synthetic"}><Layers3 size={14}/>{busy==="scene"?"Applying…":"Generate MoveIt scene"}</Button><Button variant="outline" onClick={removeGeneratedScene} disabled={busy!==null||!sceneIds.length}><RefreshCw size={13}/>Remove generated scene</Button><span className="self-center text-[10px] text-zinc-400">{sceneIds.length?`${sceneIds.length} generated objects tracked`:"Scene is not yet applied"}</span></div>
        </div></section>
      <div className="space-y-4">
        <section className="panel"><div className="panel-header"><div><div className="eyebrow">02 / Payload model</div><h2 className="mt-1 text-sm font-semibold">Mass and gripper feasibility</h2></div><Settings2 size={16} className="text-zinc-400"/></div><div className="space-y-3 p-5"><div className="flex items-end justify-between"><div><div className="text-[10px] uppercase tracking-wider text-zinc-400">Estimated object mass</div><div className="mt-1 font-mono text-2xl font-semibold">{(massOverride??objectMass).toFixed(3)} <span className="text-xs font-normal text-zinc-400">kg</span></div></div><div className="text-right text-[10px] text-zinc-400">{(payloadForce).toFixed(2)} N<br/>static weight</div></div><div className="text-[10px] leading-4 text-zinc-500">{selected.massBasis}{objectKind==="wood_box"?` · ${woodDensity} kg/m³`:""}</div><Field label="Mass override" value={massOverride??objectMass} unit="kg" step="0.001" onChange={v=>setMassOverride(v)}/><button onClick={()=>setMassOverride(null)} className="text-[10px] text-zinc-500 underline">Use estimated mass</button><div className="grid grid-cols-2 gap-2 border-t border-zinc-100 pt-3"><div><div className="text-[10px] text-zinc-400">Payload force</div><div className="font-mono text-sm">{payloadForce.toFixed(2)} N</div></div><div><div className="text-[10px] text-zinc-400">Moment @ 100 mm</div><div className="font-mono text-sm">{payloadTorque.toFixed(3)} Nm</div></div></div><Field label="Configured max opening" value={gripperMaxOpening*1000} unit="mm" onChange={v=>setGripperMaxOpening(v/1000)}/><div className={`rounded-md border p-3 text-xs ${gripFits?"border-emerald-200 bg-emerald-50 text-emerald-800":"border-amber-200 bg-amber-50 text-amber-900"}`}><div className="font-semibold">{gripFits?"Nominal jaw clearance fits":"Grasp width exceeds PGEA opening"}</div><div className="mt-1 text-[10px] leading-4">Object width {graspWidth*1000} mm + clearance {grasp.gripperClearance*1000} mm; configured opening {gripperMaxOpening*1000} mm. PGEA-100-40-W has 40 mm nominal total travel; override only for alternate tooling, not the physical gripper.</div></div></div></section>
        <section className="panel"><div className="panel-header"><div><div className="eyebrow">03 / Grasp parameters</div><h2 className="mt-1 text-sm font-semibold">TCP and approach geometry</h2></div><ArrowUpFromLine size={16} className="text-zinc-400"/></div><div className="grid grid-cols-2 gap-3 p-5"><Field label="Gripper yaw" value={grasp.yaw} unit="deg" step="1" onChange={v=>setGrasp(g=>({...g,yaw:v}))}/><Field label="Approach pitch" value={grasp.approachPitch} unit="deg" step="1" onChange={v=>setGrasp(g=>({...g,approachPitch:v}))}/><Field label="Approach roll" value={grasp.approachRoll} unit="deg" step="1" onChange={v=>setGrasp(g=>({...g,approachRoll:v}))}/><Field label="Approach distance" value={grasp.approachDistance*1000} unit="mm" onChange={v=>setGrasp(g=>({...g,approachDistance:v/1000}))}/><Field label="Lift distance" value={grasp.liftDistance*1000} unit="mm" onChange={v=>setGrasp(g=>({...g,liftDistance:v/1000}))}/><Field label="Jaw clearance" value={grasp.gripperClearance*1000} unit="mm" onChange={v=>setGrasp(g=>({...g,gripperClearance:v/1000}))}/><div className="col-span-2 flex flex-wrap gap-2 pt-1"><Button size="sm" onClick={planGrasp} disabled={busy!==null}><Play size={13}/>Plan pre-grasp</Button><Button size="sm" variant="outline" onClick={planPlace} disabled={busy!==null}><ArrowDownToLine size={13}/>Plan place target</Button></div><p className="col-span-2 text-[10px] leading-4 text-zinc-400">Centroid is the grasp TCP target. Pre-grasp is offset opposite the rotated TCP approach axis. Plan is preview-only; no motion is executed.</p></div></section>
      </div>
    </div>
    <div className="grid gap-4 md:grid-cols-3"><div className="panel p-4"><div className="eyebrow">Shelf placement check</div><div className={`mt-2 text-sm font-semibold ${objectFits?"text-emerald-700":"text-amber-700"}`}>{objectFits?"Object fits shelf opening":"Object does not fit shelf opening"}</div><div className="mt-1 text-[10px] text-zinc-500">Interior {Math.round(shelfInner.width*1000)} × {Math.round(shelfInner.depth*1000)} × {Math.round(shelfInner.height*1000)} mm; object {objectDims.map(v=>Math.round(v*1000)).join(" × ")} mm</div></div><div className="panel p-4"><div className="eyebrow">MoveIt state</div><div className="mt-2 text-sm font-semibold">{lastPlan?.status??"No plan requested"}</div><div className="mt-1 text-[10px] text-zinc-500">{lastPlan?.plan_id??"Plan-only requests use the active MoveIt group and planning scene."}</div></div><div className="panel p-4"><div className="eyebrow">Current robot model</div><div className="mt-2 text-sm font-semibold">{model?.group_name??"Waiting for model"}</div><div className="mt-1 text-[10px] text-zinc-500">TCP {model?.end_effector_link??"—"} · base {frame}</div></div></div>
    <div className="rounded-lg border border-amber-200 bg-amber-50 px-4 py-3 text-[11px] leading-5 text-amber-900"><strong>Payload safety:</strong> MoveIt’s geometric collision planner does not natively consume arbitrary attached-object mass as a dynamics/load parameter. This UI calculates static force and a reference moment; the later integration phase must apply conservative velocity/acceleration scaling and keep the robot controller’s payload/load configuration separate. Never assume the displayed mass has configured Dobot gravity compensation.</div>
  </div>;
}
