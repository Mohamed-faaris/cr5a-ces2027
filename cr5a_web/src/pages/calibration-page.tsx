import {useCallback,useEffect,useMemo,useState,type ReactNode} from "react";
import {Activity,AlertCircle,Camera,CheckCircle2,Circle,Crosshair,FileImage,Focus,Grid3X3,Info,RefreshCw,Save,Scan,ShieldCheck,Sparkles,Target} from "lucide-react";
import {Link} from "react-router-dom";
import {Button} from "@/components/ui/button";
import {StatusBadge} from "@/components/status-badge";
import {deriveRobotStatus,hasActiveError} from "@/lib/status";
import {useRobotSocket} from "@/lib/socket";
import {useRobotStore} from "@/store/robot-store";

const CAMERA_INFO_TOPIC="/cr5/wrist_camera/color/camera_info";
const SETTINGS_KEY="cr5a.calibration.settings.v1";
type TargetKind="apriltag"|"aruco"|"checkerboard"|"gridboard";
type Settings={
  target:TargetKind; tagFamily:string; dictionary:string; tagSize:number;
  rows:number; columns:number; spacing:number; cornersX:number; cornersY:number;
  squareSize:number; gridFamily:"aruco"|"apriltag"|"charuco";
  samples:number; candidates:number; velocity:number; acceleration:number;
  settleVelocity:number; maxReprojection:number; sessionPath:string;
};
const defaults:Settings={
  target:"apriltag",tagFamily:"tag36h11",dictionary:"DICT_5X5_100",
  tagSize:40,rows:5,columns:5,spacing:10,cornersX:9,cornersY:6,
  squareSize:24,gridFamily:"aruco",samples:25,candidates:60,
  velocity:0.1,acceleration:0.1,settleVelocity:0.01,maxReprojection:0.8,
  sessionPath:"~/cr5a_data/calibration/session001"
};
function loadSettings():Settings{
  try{const value=window.localStorage.getItem(SETTINGS_KEY);return value?{...defaults,...JSON.parse(value)}:defaults;}
  catch{return defaults;}
}
function Field({label,value,unit,step=1,min=0,max,onChange}:{label:string;value:number;unit?:string;step?:number;min?:number;max?:number;onChange:(value:number)=>void}){
  return <label className="block min-w-0">
    <span className="mb-1.5 block text-[11px] font-medium text-zinc-500">{label}</span>
    <span className="flex h-9 items-center rounded-md border border-zinc-200 bg-white focus-within:border-zinc-500">
      <input type="number" value={value} min={min} max={max} step={step} onChange={event=>onChange(Number(event.target.value))}
        className="min-w-0 flex-1 bg-transparent px-2.5 font-mono text-sm tabular-nums outline-none"/>
      {unit&&<span className="px-2 text-[10px] text-zinc-400">{unit}</span>}
    </span>
  </label>;
}
function SelectField({label,value,options,onChange}:{label:string;value:string;options:string[];onChange:(value:string)=>void}){
  return <label className="block min-w-0">
    <span className="mb-1.5 block text-[11px] font-medium text-zinc-500">{label}</span>
    <select value={value} onChange={event=>onChange(event.target.value)} className="h-9 w-full rounded-md border border-zinc-200 bg-white px-2.5 text-sm outline-none focus:border-zinc-500">
      {options.map(option=><option key={option}>{option}</option>)}
    </select>
  </label>;
}
function Section({title,description,children}:{title:string;description?:string;children:ReactNode}){
  return <section className="space-y-3"><div><h3 className="text-sm font-semibold text-zinc-900">{title}</h3>{description&&<p className="mt-0.5 text-xs leading-5 text-zinc-500">{description}</p>}</div>{children}</section>;
}
function Readiness({label,detail,state}:{label:string;detail:string;state:"ready"|"waiting"|"error"}){
  const Icon=state==="ready"?CheckCircle2:state==="error"?AlertCircle:Circle;
  const color=state==="ready"?"text-emerald-700":state==="error"?"text-red-700":"text-zinc-400";
  return <div className="flex items-start gap-2.5 py-2"><Icon size={15} className={"mt-0.5 shrink-0 "+color}/><div className="min-w-0 flex-1"><div className="text-xs font-medium text-zinc-800">{label}</div><div className="mt-0.5 break-words text-[11px] leading-4 text-zinc-500">{detail}</div></div><span className={"text-[10px] font-medium "+color}>{state==="ready"?"Ready":state==="error"?"Check":"Waiting"}</span></div>;
}
const targetCards=[
  {id:"apriltag" as const,label:"AprilTag",detail:"Single tag or known tag grid",Icon:Scan},
  {id:"aruco" as const,label:"ArUco",detail:"Single marker or GridBoard",Icon:Target},
  {id:"checkerboard" as const,label:"Checkerboard",detail:"Known inner-corner pattern",Icon:Grid3X3},
  {id:"gridboard" as const,label:"Grid / ChArUco",detail:"ArUco, AprilTag or ChArUco grid",Icon:Focus}
];

export function CalibrationPage(){
  const {request,subscribeChannels}=useRobotSocket();
  const bridgeState=useRobotStore(s=>s.bridgeState);
  const simulation=useRobotStore(s=>s.simulation);
  const robotStatus=useRobotStore(s=>s.robotStatus);
  const robotMode=useRobotStore(s=>s.robotMode);
  const errors=useRobotStore(s=>s.errors);
  const lastCommand=useRobotStore(s=>s.lastCommand);
  const lastJointChangeAt=useRobotStore(s=>s.lastJointChangeAt);
  const joints=useRobotStore(s=>s.joints);
  const tcp=useRobotStore(s=>s.tcp);
  const timestamps=useRobotStore(s=>s.timestamps);
  const model=useRobotStore(s=>s.model);
  const modelLoading=useRobotStore(s=>s.modelLoading);
  const modelError=useRobotStore(s=>s.modelError);
  const scene=useRobotStore(s=>s.scene);
  const sceneLoading=useRobotStore(s=>s.sceneLoading);
  const sceneError=useRobotStore(s=>s.sceneError);
  const cameraMessage=useRobotStore(s=>s.topicMessages[CAMERA_INFO_TOPIC]);
  const cameraError=useRobotStore(s=>s.topicSubscriptionErrors[CAMERA_INFO_TOPIC]);
  const setModel=useRobotStore(s=>s.setModel);
  const setScene=useRobotStore(s=>s.setScene);
  const notice=useRobotStore(s=>s.notice);
  const setNotice=useRobotStore(s=>s.setNotice);
  const [settings,setSettings]=useState<Settings>(loadSettings);
  const [refreshing,setRefreshing]=useState(false);
  const [saved,setSaved]=useState(false);

  useEffect(()=>subscribeChannels(["topic:"+CAMERA_INFO_TOPIC]),[subscribeChannels]);

  const cameraInfo=(cameraMessage?.data??null) as Record<string,unknown>|null;
  const width=Number(cameraInfo?.width??0);
  const height=Number(cameraInfo?.height??0);
  const cameraReady=Boolean(cameraInfo&&width>0&&height>0);
  const robotConnected=robotStatus?.is_connected===true;
  const robotStatusView=useMemo(()=>deriveRobotStatus({
    bridgeState,connected:robotStatus?.is_connected,enabled:robotStatus?.is_enable,
    mode:robotMode,errors,lastCommand,lastJointChangeAt
  }),[bridgeState,robotStatus,robotMode,errors,lastCommand,lastJointChangeAt]);
  const activeError=hasActiveError(errors);
  const jointsCount=joints?.name?.length??0;
  const validJoints=(joints?.position??[]).filter(value=>typeof value==="number"&&Number.isFinite(value)).length;
  const worldCount=scene?.world?.collision_objects?.length??0;
  const attachedCount=scene?.robot_state?.attached_collision_objects?.length??0;

  const update=<K extends keyof Settings,>(key:K,value:Settings[K])=>{
    setSettings(current=>({...current,[key]:value}));
    setSaved(false);
  };
  const refresh=useCallback(async()=>{
    setRefreshing(true);
    const results=await Promise.allSettled([
      request("robot.get_snapshot").then(value=>useRobotStore.getState().mergeSnapshot(value)),
      request("robot.get_model").then(value=>setModel(value)),
      request("moveit.get_scene").then(value=>setScene(value))
    ]);
    const failed=results.filter(result=>result.status==="rejected").length;
    setNotice(failed
      ?{tone:"error",message:failed+" readiness request(s) failed. Check the bridge and MoveIt startup."}
      :{tone:"success",message:"Robot, model and planning scene refreshed."});
    setRefreshing(false);
  },[request,setModel,setScene,setNotice]);
  const save=()=>{
    try{
      window.localStorage.setItem(SETTINGS_KEY,JSON.stringify(settings,null,2));
      setSaved(true);
      setNotice({tone:"success",message:"Calibration setup saved in this browser only; no robot command was sent."});
    }catch(error){
      setNotice({tone:"error",message:"Could not save setup: "+(error instanceof Error?error.message:String(error))});
    }
  };

  return <div className="space-y-6">
    <div className="flex flex-wrap items-end justify-between gap-3">
      <div><div className="eyebrow">ROBOT / CALIBRATION</div><h1 className="page-title">Calibration</h1><p className="mt-1 max-w-2xl text-sm text-zinc-500">D415 wrist-camera setup and automatic eye-in-hand calibration configuration.</p></div>
      <div className="flex items-center gap-2">
        {simulation&&<span className="rounded-md border border-amber-200 bg-amber-50 px-2 py-1 text-[10px] font-semibold text-amber-800">SIMULATION</span>}
        <StatusBadge state={robotStatusView.key}/>
        <Button variant="outline" size="sm" onClick={refresh} disabled={refreshing||bridgeState!=="connected"}><RefreshCw size={13} className={refreshing?"animate-spin":""}/> Refresh checks</Button>
      </div>
    </div>
    {notice&&<div className={notice.tone==="error"?"notice notice-error":notice.tone==="success"?"notice notice-success":"notice"}>{notice.message}</div>}
    <div className="notice border-amber-200 bg-amber-50 text-amber-900"><div className="flex items-start gap-2"><Info size={15} className="mt-0.5 shrink-0"/><div><div className="text-xs font-semibold">Calibration engine not implemented yet</div><p className="mt-1 text-xs leading-5 text-amber-800">This page provides target setup, live readiness and saved browser configuration. Live image transport, target detection, collision insertion, automatic viewpoint generation, synchronized capture, solving and activation still require the ROS calibration backend. Start is intentionally disabled until those operations exist.</p></div></div></div>

    <div className="grid items-start gap-5 xl:grid-cols-[minmax(0,1fr)_350px]">
      <div className="space-y-5">
        <section className="panel">
          <div className="panel-header"><div className="flex items-center gap-2"><Camera size={16} className="text-zinc-500"/><span className="text-sm font-semibold">Camera and mode</span></div><span className="text-[10px] font-semibold tracking-wider text-zinc-400">FIXED</span></div>
          <div className="grid gap-3 p-4 sm:grid-cols-2">
            <div className="rounded-lg border border-zinc-200 bg-zinc-50 p-3">
              <div className="flex items-center gap-2 text-xs font-semibold text-zinc-800"><Camera size={14}/> Intel RealSense D415</div>
              <p className="mt-2 text-[11px] leading-5 text-zinc-500">Wrist-mounted RGB-D camera. CameraInfo is dynamically subscribed through the shared bridge WebSocket.</p>
              <div className="mt-3 flex justify-between text-[11px]"><span className="text-zinc-500">CameraInfo</span><span className={cameraReady?"font-semibold text-emerald-700":"font-semibold text-zinc-500"}>{cameraReady?"Receiving":"Not confirmed"}</span></div>
              <div className="mt-1 break-all font-mono text-[10px] text-zinc-400">{CAMERA_INFO_TOPIC}</div>
              {cameraReady&&<div className="mt-2 border-t border-zinc-200 pt-2 text-xs text-zinc-600">Resolution <strong className="font-mono">{width} × {height}</strong></div>}
              {cameraError&&<div className="mt-2 text-[10px] text-red-700">{cameraError}</div>}
            </div>
            <div className="rounded-lg border border-zinc-200 bg-zinc-50 p-3">
              <div className="flex items-center gap-2 text-xs font-semibold text-zinc-800"><Crosshair size={14}/> Eye-in-hand</div>
              <p className="mt-2 text-[11px] leading-5 text-zinc-500">Camera remains fixed to the wrist. The calibration board is stationary while the CR5A moves through automatically generated viewpoints.</p>
              <div className="mt-3 grid grid-cols-2 gap-2 border-t border-zinc-200 pt-2"><div><div className="text-[10px] text-zinc-400">MoveIt base</div><div className="font-mono text-xs">{model?.base_frame??"—"}</div></div><div><div className="text-[10px] text-zinc-400">End effector</div><div className="font-mono text-xs">{model?.end_effector_link??"—"}</div></div></div>
            </div>
          </div>
          <div className="border-t border-zinc-100 px-4 py-3 text-[11px] leading-5 text-zinc-500">CameraInfo confirms calibration metadata only. A compressed RGB image stream and detector overlay are needed for the live board preview.</div>
        </section>

        <section className="panel">
          <div className="panel-header"><div><div className="text-sm font-semibold">Calibration target</div><div className="mt-0.5 text-xs text-zinc-500">Choose the printed board that remains fixed in the workspace.</div></div><Target size={16} className="text-zinc-400"/></div>
          <div className="grid gap-2 p-4 sm:grid-cols-2">
            {targetCards.map(({id,label,detail,Icon})=>{
              const selected=settings.target===id;
              return <button key={id} type="button" onClick={()=>update("target",id)} className={"rounded-lg border p-3 text-left transition "+(selected?"border-zinc-900 bg-zinc-50 ring-1 ring-zinc-900":"border-zinc-200 bg-white hover:border-zinc-400")}>
                <div className="flex items-center justify-between"><span className="flex items-center gap-2 text-xs font-semibold text-zinc-900"><Icon size={14}/>{label}</span><span className={"h-3.5 w-3.5 rounded-full border "+(selected?"border-zinc-900 bg-zinc-900 ring-2 ring-zinc-200":"border-zinc-300")}/></div><p className="mt-1.5 text-[11px] text-zinc-500">{detail}</p>
              </button>;
            })}
          </div>
          <div className="border-t border-zinc-100 p-4">
            {settings.target==="apriltag"&&<Section title="AprilTag grid" description="Set values to the physical print, including the black tag border size used by the detector.">
              <div className="grid gap-3 sm:grid-cols-2">
                <SelectField label="Tag family" value={settings.tagFamily} options={["tag36h11","tag25h9","tag16h5"]} onChange={value=>update("tagFamily",value)}/>
                <Field label="Tag size" value={settings.tagSize} unit="mm" step={0.5} onChange={value=>update("tagSize",value)}/>
                <Field label="Rows" value={settings.rows} min={1} max={20} onChange={value=>update("rows",value)}/>
                <Field label="Columns" value={settings.columns} min={1} max={20} onChange={value=>update("columns",value)}/>
                <Field label="Gap between tags" value={settings.spacing} unit="mm" step={0.5} onChange={value=>update("spacing",value)}/>
              </div>
            </Section>}
            {settings.target==="aruco"&&<Section title="ArUco GridBoard" description="Dictionary and geometry must match the printed board.">
              <div className="grid gap-3 sm:grid-cols-2">
                <SelectField label="Dictionary" value={settings.dictionary} options={["DICT_4X4_50","DICT_4X4_100","DICT_5X5_100","DICT_5X5_250","DICT_6X6_100","DICT_6X6_250","DICT_7X7_100"]} onChange={value=>update("dictionary",value)}/>
                <Field label="Marker size" value={settings.tagSize} unit="mm" step={0.5} onChange={value=>update("tagSize",value)}/>
                <Field label="Rows" value={settings.rows} min={1} max={20} onChange={value=>update("rows",value)}/>
                <Field label="Columns" value={settings.columns} min={1} max={20} onChange={value=>update("columns",value)}/>
                <Field label="Marker separation" value={settings.spacing} unit="mm" step={0.5} onChange={value=>update("spacing",value)}/>
              </div>
            </Section>}
            {settings.target==="checkerboard"&&<Section title="Checkerboard geometry" description="Counts are inner corners, not the number of printed squares.">
              <div className="grid gap-3 sm:grid-cols-3">
                <Field label="Inner corners X" value={settings.cornersX} min={2} max={30} onChange={value=>update("cornersX",value)}/>
                <Field label="Inner corners Y" value={settings.cornersY} min={2} max={30} onChange={value=>update("cornersY",value)}/>
                <Field label="Square size" value={settings.squareSize} unit="mm" step={0.5} onChange={value=>update("squareSize",value)}/>
              </div>
            </Section>}
            {settings.target==="gridboard"&&<Section title="Grid board geometry" description="Select the board family and its printed layout.">
              <div className="grid gap-3 sm:grid-cols-2">
                <SelectField label="Board family" value={settings.gridFamily} options={["aruco","apriltag","charuco"]} onChange={value=>update("gridFamily",value as Settings["gridFamily"])}/>
                {settings.gridFamily==="charuco"&&<SelectField label="Dictionary" value={settings.dictionary} options={["DICT_4X4_50","DICT_5X5_100","DICT_6X6_250"]} onChange={value=>update("dictionary",value)}/>}
                <Field label={settings.gridFamily==="charuco"?"Squares X":"Rows"} value={settings.rows} min={2} max={30} onChange={value=>update("rows",value)}/>
                <Field label={settings.gridFamily==="charuco"?"Squares Y":"Columns"} value={settings.columns} min={2} max={30} onChange={value=>update("columns",value)}/>
                <Field label={settings.gridFamily==="charuco"?"Square size":"Marker size"} value={settings.squareSize} unit="mm" step={0.5} onChange={value=>update("squareSize",value)}/>
                <Field label="Marker spacing / size" value={settings.spacing} unit="mm" step={0.5} onChange={value=>update("spacing",value)}/>
              </div>
            </Section>}
            <div className="mt-4 rounded-md bg-zinc-50 px-3 py-2.5 text-[11px] leading-5 text-zinc-500">The board must have known physical dimensions. It will be detected from one complete frame before its collision geometry is added to MoveIt.</div>
          </div>
        </section>

        <section className="panel">
          <div className="panel-header"><div><div className="text-sm font-semibold">Automatic viewpoint collection</div><div className="mt-0.5 text-xs text-zinc-500">Generate diverse poses automatically; no manual pose list.</div></div><Sparkles size={16} className="text-zinc-400"/></div>
          <div className="grid gap-3 p-4 sm:grid-cols-2 lg:grid-cols-3">
            <Field label="Accepted samples" value={settings.samples} min={8} max={100} onChange={value=>update("samples",value)}/>
            <Field label="Candidate limit" value={settings.candidates} min={10} max={300} onChange={value=>update("candidates",value)}/>
            <Field label="Max reprojection error" value={settings.maxReprojection} unit="px" step={0.1} onChange={value=>update("maxReprojection",value)}/>
            <Field label="Velocity scaling" value={settings.velocity} step={0.01} min={0.01} max={1} onChange={value=>update("velocity",value)}/>
            <Field label="Acceleration scaling" value={settings.acceleration} step={0.01} min={0.01} max={1} onChange={value=>update("acceleration",value)}/>
            <Field label="Settle threshold" value={settings.settleVelocity} unit="rad/s" step={0.001} onChange={value=>update("settleVelocity",value)}/>
          </div>
          <div className="mx-4 mb-4 rounded-md border border-blue-100 bg-blue-50 px-3 py-2.5 text-[11px] leading-5 text-blue-900">Capture only after execution completes and live joint feedback settles. Store requested pose and measured pose separately; reject blurred, partial-board, high-error or redundant samples.</div>
        </section>
      </div>

      <aside className="space-y-5">
        <section className="panel">
          <div className="panel-header"><div className="text-sm font-semibold">System readiness</div><Activity size={15} className="text-zinc-400"/></div>
          <div className="divide-y divide-zinc-100 px-4">
            <Readiness label="ROS bridge" detail={bridgeState==="connected"?"Application WebSocket connected":"Bridge: "+bridgeState} state={bridgeState==="connected"?"ready":bridgeState==="disconnected"?"error":"waiting"}/>
            <Readiness label="CR5A controller" detail={robotConnected?(simulation?"Simulation telemetry":"Controller connected"):"Waiting for RobotStatus"} state={robotConnected?"ready":bridgeState==="connected"?"waiting":"error"}/>
            <Readiness label="Joint feedback" detail={jointsCount+" joints · "+validJoints+" valid · "+(timestamps.joints??"timestamp unavailable")} state={validJoints>=6?"ready":"waiting"}/>
            <Readiness label="MoveIt robot model" detail={model?model.base_frame+" → "+model.end_effector_link:modelError??(modelLoading?"Loading model":"Not loaded")} state={model?"ready":modelError?"error":"waiting"}/>
            <Readiness label="MoveIt scene" detail={scene?worldCount+" world · "+attachedCount+" attached":sceneError??(sceneLoading?"Loading scene":"Not loaded")} state={scene?"ready":sceneError?"error":"waiting"}/>
            <Readiness label="D415 CameraInfo" detail={cameraReady?width+" × "+height+" intrinsics received":cameraError??"Waiting for "+CAMERA_INFO_TOPIC} state={cameraReady?"ready":cameraError?"error":"waiting"}/>
            <Readiness label="Controller errors" detail={activeError?"Error data present; inspect Diagnostics":"No active error reported"} state={activeError?"error":robotConnected?"ready":"waiting"}/>
          </div>
          <div className="border-t border-zinc-100 px-4 py-3"><Link to="/diagnostics" className="text-xs font-medium text-zinc-700 underline decoration-zinc-300 underline-offset-4 hover:text-zinc-950">Open diagnostics and jog controls</Link></div>
        </section>

        <section className="panel overflow-hidden">
          <div className="panel-header"><div><div className="text-sm font-semibold">Initial positioning</div><div className="mt-0.5 text-xs text-zinc-500">One manual setup step</div></div><Focus size={15} className="text-zinc-400"/></div>
          <div className="space-y-3 p-4">
            <div className="flex aspect-video items-center justify-center rounded-lg border border-dashed border-zinc-300 bg-zinc-950 text-center">
              <div className="px-5"><FileImage size={25} className="mx-auto mb-2 text-zinc-600"/><div className="text-xs font-medium text-zinc-300">D415 image preview unavailable</div><div className="mt-1 text-[10px] leading-4 text-zinc-500">Current bridge integration monitors CameraInfo only. A compressed-image relay is required for the live frame and board overlay.</div></div>
            </div>
            <ol className="space-y-2 text-[11px] leading-5 text-zinc-600"><li>01 · Jog the CR5A until the entire board is visible.</li><li>02 · Keep the board fixed and the camera unobstructed.</li><li>03 · Automatic detection will estimate the board pose before scene insertion.</li></ol>
            <div className="rounded-md bg-zinc-50 px-3 py-2 text-[10px] leading-4 text-zinc-500">A rough Link6-to-camera transform from bracket CAD or measurement is needed for initial collision placement.</div>
          </div>
        </section>

        <section className="panel">
          <div className="panel-header"><div className="text-sm font-semibold">Session storage</div><ShieldCheck size={15} className="text-zinc-400"/></div>
          <div className="space-y-3 p-4">
            <label className="block"><span className="mb-1.5 block text-[11px] font-medium text-zinc-500">Persistent session directory</span><input value={settings.sessionPath} onChange={event=>update("sessionPath",event.target.value)} className="h-9 w-full rounded-md border border-zinc-200 px-2.5 font-mono text-xs outline-none focus:border-zinc-500"/></label>
            <div className="rounded-md bg-zinc-50 p-3"><div className="text-[10px] font-semibold uppercase tracking-wider text-zinc-500">Measurement file</div><div className="mt-1 break-all font-mono text-[10px] leading-5 text-zinc-600">samples/pose001/image-pose001-actual-measurement.png</div><div className="mt-1 text-[10px] leading-4 text-zinc-400">Save original image, overlay, depth, requested pose, actual pose, joints and target observation.</div></div>
            <Button variant="outline" className="w-full" onClick={save}><Save size={14}/>{saved?"Setup saved":"Save setup locally"}</Button>
            <p className="text-[10px] leading-4 text-zinc-400">Saves the settings in this browser only. The ROS backend will create session folders and image files.</p>
          </div>
        </section>

        <section className="panel">
          <div className="panel-header"><div className="text-sm font-semibold">Calibration workflow</div><span className="text-[10px] font-medium text-zinc-400">NOT STARTED</span></div>
          <div className="space-y-3 p-4">
            {[
              ["Detect complete board","Image transport and detector pending"],
              ["Add board to collision scene","Needs board pose and approximate camera extrinsic"],
              ["Generate safe viewpoints","MoveIt viewpoint generator pending"],
              ["Capture actual pose and image","Synchronized sample collector pending"],
              ["Solve and validate transform","Hand-eye solver pending"],
              ["Save and activate","Requires passing validation and approval"]
            ].map(([title,detail],index)=><div key={title} className="flex gap-2.5"><span className="flex h-5 w-5 shrink-0 items-center justify-center rounded-full border border-zinc-200 text-[10px] font-medium text-zinc-400">{index+1}</span><div><div className="text-xs font-medium text-zinc-700">{title}</div><div className="mt-0.5 text-[10px] leading-4 text-zinc-400">{detail}</div></div></div>)}
            <Button className="w-full" disabled title="Calibration backend has not been implemented yet"><Sparkles size={14}/> Start automatic calibration</Button>
            <div className="text-center text-[10px] text-zinc-400">Start, pause, capture, solve and save controls will be enabled after the backend is implemented.</div>
          </div>
        </section>
      </aside>
    </div>
    <div className="flex flex-wrap items-center justify-between gap-3 border-t border-zinc-200 pt-4 text-[11px] text-zinc-400"><span>Joint data: {validJoints} / {jointsCount} valid · TCP: {tcp?"available":"waiting"}</span><span>CameraInfo received: {cameraMessage?.timestamp??"not yet"}</span></div>
  </div>;
}
