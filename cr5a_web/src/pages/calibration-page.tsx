import {useCallback,useEffect,useMemo,useState,type ReactNode} from "react";
import {Activity,AlertCircle,Camera,CheckCircle2,Circle,Crosshair,FileImage,Focus,Grid3X3,Info,RefreshCw,Save,Scan,ShieldCheck,Sparkles,Target,Upload,Eye,EyeOff} from "lucide-react";
import {Link} from "react-router-dom";
import {Button} from "@/components/ui/button";
import {StatusBadge} from "@/components/status-badge";
import {deriveRobotStatus,hasActiveError} from "@/lib/status";
import {useRobotSocket} from "@/lib/socket";
import {useRobotStore} from "@/store/robot-store";
import {RobotViewport} from "@/components/robot/robot-viewport";
import {loadVisualizerViewPreferences,saveVisualizerViewPreferences} from "@/lib/visualizer-view-preferences";

const CAMERA_INFO_TOPIC="/cr5/wrist_camera/color/camera_info";
type GeneratedPose={
  id:string;label:string;position_m:[number,number,number];orientation_xyzw?:[number,number,number,number];
  pose_frame?:"camera"|"tcp";kind?:"sample"|"planned";tcp_position_m?:[number,number,number];joints?:Record<string,number>;
  image_data_url?:string;image_path?:string;reprojection_rms_px?:number;tag_ids?:number[];
};
function parseGeneratedPoses(value:unknown):GeneratedPose[]{
  const rows=Array.isArray(value)?value:(value&&typeof value==="object"&&Array.isArray((value as {poses?:unknown}).poses)?(value as {poses:unknown[]}).poses:[]);
  if(!rows.length)throw new Error("Expected a non-empty array or an object with a poses array.");
  return rows.map((row,index)=>{
    if(!row||typeof row!=="object")throw new Error("Pose "+(index+1)+" is not an object.");
    const item=row as Record<string,unknown>;
    const p=item.position_m;
    if(!Array.isArray(p)||p.length!==3||!p.every(v=>typeof v==="number"&&Number.isFinite(v)))throw new Error("Pose "+(index+1)+" needs position_m: [x, y, z] in metres.");
    const q=item.orientation_xyzw;
    if(q!==undefined&&(!Array.isArray(q)||q.length!==4||!q.every(v=>typeof v==="number"&&Number.isFinite(v))))throw new Error("Pose "+(index+1)+" orientation_xyzw must be a finite [x, y, z, w] quaternion.");
    const joints=item.joints&&typeof item.joints==="object"?item.joints as Record<string,number>:undefined;
    const image=item.image_data_url;
    return {
      id:String(item.id??("pose-"+(index+1))),label:String(item.label??("Pose "+(index+1))),
      position_m:p as [number,number,number],orientation_xyzw:q as [number,number,number,number]|undefined,
      pose_frame:item.pose_frame==="camera"?"camera":item.pose_frame==="tcp"?"tcp":undefined,
      kind:item.kind==="planned"?"planned":"sample",
      tcp_position_m:Array.isArray(item.tcp_position_m)&&item.tcp_position_m.length===3?item.tcp_position_m as [number,number,number]:undefined,
      joints,image_data_url:typeof image==="string"&&image.startsWith("data:image/")?image:undefined,
      image_path:typeof item.image_path==="string"?item.image_path:undefined,
      reprojection_rms_px:typeof item.reprojection_rms_px==="number"&&Number.isFinite(item.reprojection_rms_px)?item.reprojection_rms_px:undefined,
      tag_ids:Array.isArray(item.tag_ids)?item.tag_ids.filter((v):v is number=>typeof v==="number"&&Number.isFinite(v)):undefined
    };
  });
}
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
  target:"gridboard",tagFamily:"tag36h11",dictionary:"DICT_5X5_1000",
  tagSize:22,rows:17,columns:24,spacing:10,cornersX:9,cornersY:6,
  squareSize:30,gridFamily:"charuco",samples:25,candidates:60,
  velocity:0.1,acceleration:0.1,settleVelocity:0.01,maxReprojection:5.0,
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
  const plan=useRobotStore(s=>s.plan);
  const [generatedPoses,setGeneratedPoses]=useState<GeneratedPose[]>([]);
  const [selectedPoseId,setSelectedPoseId]=useState<string|null>(null);
  const [showAllPoses,setShowAllPoses]=useState(true);
  const [showPoseMarkers,setShowPoseMarkers]=useState(true);
  const [poseImportError,setPoseImportError]=useState<string|null>(null);
  const [datasetSummary,setDatasetSummary]=useState<Record<string,unknown>|null>(null);
  const [datasetFrames,setDatasetFrames]=useState<{base?:string;tcp?:string;camera?:string}|null>(null);
  const [datasetSource,setDatasetSource]=useState<string|null>(null);
  const [datasetWarnings,setDatasetWarnings]=useState<string[]>([]);
  const [loadingReference,setLoadingReference]=useState(false);
  const [viewPreferences,setViewPreferences]=useState(loadVisualizerViewPreferences);
  useEffect(()=>{saveVisualizerViewPreferences(viewPreferences);},[viewPreferences]);
  const selectedPose=generatedPoses.find(p=>p.id===selectedPoseId)??generatedPoses[0]??null;
  const visiblePoseMarkers=showPoseMarkers?generatedPoses.filter(p=>showAllPoses||p.id===selectedPoseId).map(p=>({
    id:p.id,label:p.label,position:p.position_m,orientation:p.orientation_xyzw,frame_id:datasetFrames?.base,color:p.id!==selectedPoseId&&p.kind==="planned"?"#2563eb":undefined,selected:p.id===selectedPoseId
  })):[];

  const applyDatasetBundle=(raw:unknown)=>{
    const bundle=raw&&typeof raw==="object"&&!Array.isArray(raw)?raw as Record<string,unknown>:{};
    const sampleRows=Array.isArray(raw)?raw:Array.isArray(bundle.poses)?bundle.poses:[];
    const parsed=sampleRows.length?parseGeneratedPoses(sampleRows):[];
    const plannedRows=Array.isArray(bundle.planned_poses)?bundle.planned_poses:[];
    const planned=plannedRows.length?parseGeneratedPoses({poses:plannedRows}).map(p=>({...p,kind:"planned" as const})):[];
    const combined=[...parsed.map(p=>({...p,kind:"sample" as const})),...planned];
    if(!combined.length)throw new Error("No captured samples or Cartesian planned poses were found in this JSON.");
    setGeneratedPoses(combined);
    setSelectedPoseId(combined.find(p=>p.kind==="sample")?.id??combined[0]?.id??null);
    setShowAllPoses(true);
    setDatasetSource(typeof bundle.source_dataset==="string"?bundle.source_dataset:"Imported pose set");
    setDatasetSummary(bundle.summary&&typeof bundle.summary==="object"?bundle.summary as Record<string,unknown>:null);
    setDatasetFrames(bundle.frames&&typeof bundle.frames==="object"?bundle.frames as {base?:string;tcp?:string;camera?:string}:null);
    setDatasetWarnings(Array.isArray(bundle.warnings)?bundle.warnings.filter((v):v is string=>typeof v==="string"):[]);
    setPoseImportError(null);
  };

  useEffect(()=>subscribeChannels(["topic:"+CAMERA_INFO_TOPIC]),[subscribeChannels]);

  const cameraInfo=(cameraMessage?.data??null) as Record<string,unknown>|null;
  const cameraHeader=cameraInfo?.header as {frame_id?:string}|undefined;
  const liveCameraFrame=String(cameraHeader?.frame_id??"");
  const width=Number(cameraInfo?.width??0);
  const height=Number(cameraInfo?.height??0);
  const cameraReady=Boolean(cameraInfo&&width>0&&height>0);
  const cameraFrameMismatch=Boolean(datasetFrames?.camera&&liveCameraFrame&&datasetFrames.camera!==liveCameraFrame);
  const baseFrameMismatch=Boolean(datasetFrames?.base&&model?.base_frame&&datasetFrames.base!==model.base_frame);
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
  const loadReferenceDataset=async()=>{
    setLoadingReference(true);
    try{
      const response=await fetch("/calibration/reference-dataset-002.json");
      if(!response.ok)throw new Error("Reference dataset request failed ("+response.status+").");
      applyDatasetBundle(await response.json());
    }catch(error){
      setPoseImportError(error instanceof Error?error.message:String(error));
    }finally{
      setLoadingReference(false);
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
    <div className="notice border-amber-200 bg-amber-50 text-amber-900"><div className="flex items-start gap-2"><Info size={15} className="mt-0.5 shrink-0"/><div><div className="text-xs font-semibold">Reference dataset preview · live calibration backend pending</div><p className="mt-1 text-xs leading-5 text-amber-800">This page can preview converted datasets and pose/image samples. The existing ROS 2 capture, viewpoint-generation, MoveIt planning, validation and hand-eye solver workflows remain separate CLI tools; this page does not execute robot motion or apply a calibration transform.</p></div></div></div>

    <section className="panel overflow-hidden">
      <div className="panel-header"><div className="flex items-center gap-2"><Crosshair size={15} className="text-zinc-500"/><span className="text-sm font-semibold">3D calibration workspace</span><span className="text-xs text-zinc-400">{generatedPoses.length} pose samples</span></div><span className="text-[10px] text-zinc-400">{model?.base_frame??"dummy_link"}</span></div>
      <div className="grid items-start xl:grid-cols-[minmax(0,1fr)_280px]">
        <div className="min-w-0">
          <RobotViewport model={model} jointState={joints} scene={scene} plan={plan} loading={modelLoading} error={modelError??sceneError}
            viewPreferences={viewPreferences} onViewPreferencesChange={setViewPreferences}
            objectLabels={viewPreferences.objectLabels} disabledObjects={{}} hiddenObjects={viewPreferences.hiddenObjects} poseMarkers={visiblePoseMarkers}/>
          <div className="flex flex-wrap items-center gap-4 border-t border-zinc-100 px-3 py-2 text-[10px] text-zinc-500"><span><i className="mr-1.5 inline-block h-2.5 w-2.5 rounded-full bg-teal-600"/>Captured sample</span><span><i className="mr-1.5 inline-block h-2.5 w-2.5 rounded-full bg-blue-600"/>Generated plan</span><span><i className="mr-1.5 inline-block h-2.5 w-2.5 rounded-full bg-orange-500"/>Selected pose</span><span>Drag to orbit · Scroll to zoom · Right-drag to pan</span></div>
        </div>
        <aside className="space-y-3 border-t border-zinc-100 p-3 xl:border-l xl:border-t-0">
          <div><div className="text-xs font-semibold text-zinc-800">Reference samples and viewpoints</div><p className="mt-1 text-[10px] leading-4 text-zinc-500">Import a converted legacy dataset to inspect its camera/TCP poses, orientations and captured D415 frames in the robot base frame.</p></div>
          <button type="button" onClick={loadReferenceDataset} disabled={loadingReference} className="flex h-9 w-full items-center justify-center gap-2 rounded-md border border-zinc-900 bg-zinc-900 px-3 text-xs font-medium text-white hover:bg-zinc-800 disabled:opacity-60">{loadingReference?<RefreshCw size={13} className="animate-spin"/>:<Eye size={13}/>} {loadingReference?"Loading reference dataset…":"Load bundled reference run 002 · 34 images + 6 targets"}</button>
          <label className="flex h-9 cursor-pointer items-center justify-center gap-2 rounded-md border border-zinc-300 bg-white px-3 text-xs font-medium text-zinc-700 hover:bg-zinc-50"><Upload size={13}/> Import another dataset / pose JSON<input type="file" accept=".json,application/json" className="hidden" onChange={async event=>{const file=event.target.files?.[0];if(!file)return;try{applyDatasetBundle(JSON.parse(await file.text()));}catch(error){setPoseImportError(error instanceof Error?error.message:String(error));}finally{event.target.value="";}}}/></label>
          {poseImportError&&<div className="rounded-md border border-red-200 bg-red-50 px-2.5 py-2 text-[10px] text-red-700">{poseImportError}</div>}
          {datasetSummary&&<div className="rounded-md border border-zinc-200 bg-zinc-50 p-2.5">
            <div className="text-[11px] font-semibold text-zinc-800">{datasetSource??"Imported dataset"}</div>
            <div className="mt-2 grid grid-cols-2 gap-2 text-[10px]">
              <div><div className="text-zinc-400">Samples</div><div className="font-mono font-semibold text-zinc-700">{String(datasetSummary.sample_count??generatedPoses.filter(p=>p.kind!=="planned").length)}</div></div>
              <div><div className="text-zinc-400">Planned poses</div><div className="font-mono font-semibold text-zinc-700">{String(datasetSummary.planned_pose_count??generatedPoses.filter(p=>p.kind==="planned").length)}</div></div>
              <div><div className="text-zinc-400">Board RMS</div><div className="font-mono font-semibold text-zinc-700">{typeof datasetSummary.mean_board_reprojection_rms_px==="number"?datasetSummary.mean_board_reprojection_rms_px.toFixed(2)+" px":"—"}</div></div>
              <div><div className="text-zinc-400">Closure translation</div><div className="font-mono font-semibold text-zinc-700">{typeof datasetSummary.translation_rms_mm==="number"?datasetSummary.translation_rms_mm.toFixed(1)+" mm":"—"}</div></div>
              <div><div className="text-zinc-400">Closure rotation</div><div className="font-mono font-semibold text-zinc-700">{typeof datasetSummary.rotation_rms_deg==="number"?datasetSummary.rotation_rms_deg.toFixed(2)+"°":"—"}</div></div>
            </div>
            <div className="mt-2 text-[9px] leading-4 text-zinc-500">Solver: {String(datasetSummary.algorithm??"not recorded")}. Best closure methods: translation {String(datasetSummary.best_translation_rms_algorithm??"—")}, rotation {String(datasetSummary.best_rotation_rms_algorithm??"—")}. These are consistency checks, not ground-truth accuracy.</div>
            {datasetFrames&&<div className="mt-2 border-t border-zinc-200 pt-2 text-[9px] leading-4 text-zinc-500">Frames: {datasetFrames.base??"?"} → {datasetFrames.tcp??"?"} → {datasetFrames.camera??"?"}</div>}
            {baseFrameMismatch&&<div className="mt-2 rounded border border-amber-200 bg-amber-50 p-2 text-[9px] leading-4 text-amber-800">Reference poses use {datasetFrames?.base}; the live MoveIt model uses {model?.base_frame}. The viewport applies /tf_static when available and hides the markers if no transform exists.</div>}
            {cameraFrameMismatch&&<div className="mt-2 rounded border border-amber-200 bg-amber-50 p-2 text-[9px] leading-4 text-amber-800">Imported camera frame differs from live CameraInfo ({liveCameraFrame}). The imported poses are reference data only; verify the camera mounting and frame mapping before reuse.</div>}
          </div>}
          {selectedPose&&<div className="overflow-hidden rounded-md border border-zinc-200">
            <div className="flex items-center justify-between gap-2 border-b border-zinc-100 px-2.5 py-2"><span className="text-[10px] font-semibold text-zinc-700">{selectedPose.kind==="planned"?"Generated target pose":"Captured camera frame"}</span><span className="text-[9px] text-zinc-400">{selectedPose.label}</span></div>
            {selectedPose.image_data_url?<img src={selectedPose.image_data_url} alt={"D415 capture "+selectedPose.label} className="block aspect-video w-full bg-zinc-950 object-contain"/>:<div className="flex aspect-video items-center justify-center bg-zinc-950 px-3 text-center text-[10px] text-zinc-500">{selectedPose.image_path??(selectedPose.kind==="planned"?"Planned TCP target pose — no captured image":"No embedded image thumbnail")}</div>}
            <div className="flex flex-wrap items-center justify-between gap-2 px-2.5 py-2 text-[9px] text-zinc-500">
              <span>{selectedPose.pose_frame==="camera"?"Camera optical pose":selectedPose.pose_frame==="tcp"?"TCP pose":"Imported pose"}</span>
              <span>{typeof selectedPose.reprojection_rms_px==="number"?"Board RMS "+selectedPose.reprojection_rms_px.toFixed(2)+" px":"No detection RMS"}</span>
            </div>
            {selectedPose.tag_ids&&<div className="px-2.5 pb-2 text-[9px] text-zinc-400">{selectedPose.tag_ids.length} detected markers / IDs</div>}
          </div>}
          {datasetWarnings.length>0&&<div className="rounded-md border border-amber-200 bg-amber-50 p-2 text-[9px] leading-4 text-amber-800">{datasetWarnings.slice(0,3).join(" · ")}{datasetWarnings.length>3?" · +"+(datasetWarnings.length-3)+" more":""}</div>}
          <div className="flex items-center justify-between gap-2"><button type="button" onClick={()=>setShowAllPoses(true)} disabled={!generatedPoses.length} className={"rounded-md px-2.5 py-1.5 text-[10px] font-medium "+(showAllPoses?"bg-zinc-900 text-white":"border border-zinc-200 text-zinc-600 disabled:opacity-40")}>Show all</button><button type="button" onClick={()=>setShowAllPoses(false)} disabled={!generatedPoses.length||!selectedPoseId} className={"rounded-md px-2.5 py-1.5 text-[10px] font-medium "+(!showAllPoses?"bg-zinc-900 text-white":"border border-zinc-200 text-zinc-600 disabled:opacity-40")}>Selected only</button><button type="button" onClick={()=>setShowPoseMarkers(value=>!value)} className="rounded-md border border-zinc-200 p-1.5 text-zinc-600" title={showPoseMarkers?"Hide pose markers":"Show pose markers"}>{showPoseMarkers?<Eye size={13}/>:<EyeOff size={13}/>}</button></div>
          <div className="max-h-64 space-y-1 overflow-y-auto">
            {generatedPoses.length===0?<div className="rounded-md border border-dashed border-zinc-200 px-3 py-5 text-center"><Crosshair size={18} className="mx-auto mb-2 text-zinc-300"/><div className="text-[11px] font-medium text-zinc-600">No pose samples loaded</div><div className="mt-1 text-[10px] leading-4 text-zinc-400">The web page does not generate or capture poses yet. Import a converted dataset bundle to preview reference samples.</div></div>:generatedPoses.map((pose,index)=><button key={pose.id} type="button" onClick={()=>{setSelectedPoseId(pose.id);setShowAllPoses(false);}} className={"w-full rounded-md border px-2.5 py-2 text-left "+(selectedPoseId===pose.id?"border-orange-300 bg-orange-50":"border-zinc-100 hover:bg-zinc-50")}><div className="flex items-center justify-between gap-2"><span className="truncate text-[11px] font-medium text-zinc-800">{index+1}. {pose.label}</span><span className="shrink-0 text-[9px] text-zinc-400">{pose.kind==="planned"?"planned":pose.pose_frame==="camera"?"camera":pose.pose_frame==="tcp"?"TCP":pose.joints?Object.keys(pose.joints).length+" joints":"position"}</span></div><div className="mt-1 font-mono text-[9px] tabular-nums text-zinc-500">[{pose.position_m.map(v=>v.toFixed(3)).join(", ")}] m{typeof pose.reprojection_rms_px==="number"?" · "+pose.reprojection_rms_px.toFixed(2)+" px RMS":""}</div></button>)}
          </div>
          {generatedPoses.length>0&&<div className="flex items-center justify-between border-t border-zinc-100 pt-2 text-[10px] text-zinc-400"><span>{visiblePoseMarkers.length} / {generatedPoses.length} visible</span><button type="button" onClick={()=>{setGeneratedPoses([]);setSelectedPoseId(null);setShowAllPoses(true);setDatasetSummary(null);setDatasetFrames(null);setDatasetSource(null);setDatasetWarnings([]);}} className="font-medium text-zinc-500 hover:text-red-700">Clear set</button></div>}
        </aside>
      </div>
    </section>
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
                {settings.gridFamily==="charuco"&&<SelectField label="Dictionary" value={settings.dictionary} options={["DICT_4X4_50","DICT_5X5_100","DICT_5X5_1000","DICT_6X6_250"]} onChange={value=>update("dictionary",value)}/>}
                <Field label={settings.gridFamily==="charuco"?"Squares X":"Rows"} value={settings.gridFamily==="charuco"?settings.columns:settings.rows} min={2} max={30} onChange={value=>update(settings.gridFamily==="charuco"?"columns":"rows",value)}/>
                <Field label={settings.gridFamily==="charuco"?"Squares Y":"Columns"} value={settings.gridFamily==="charuco"?settings.rows:settings.columns} min={2} max={30} onChange={value=>update(settings.gridFamily==="charuco"?"rows":"columns",value)}/>
                {settings.gridFamily==="charuco"?<>
                  <Field label="Square size" value={settings.squareSize} unit="mm" step={0.5} onChange={value=>update("squareSize",value)}/>
                  <Field label="Marker size" value={settings.tagSize} unit="mm" step={0.5} onChange={value=>update("tagSize",value)}/>
                </>:<>
                  <Field label="Marker size" value={settings.tagSize} unit="mm" step={0.5} onChange={value=>update("tagSize",value)}/>
                  <Field label="Marker pitch" value={settings.spacing} unit="mm" step={0.5} onChange={value=>update("spacing",value)}/>
                </>}
              </div>
            </Section>}
            <div className="mt-4 rounded-md bg-zinc-50 px-3 py-2.5 text-[11px] leading-5 text-zinc-500">The board must have known physical dimensions. The legacy workflow detects the board, estimates its pose, then uses it to generate camera viewpoints and validate candidate poses.</div>
          </div>
        </section>

        <section className="panel">
          <div className="panel-header"><div><div className="text-sm font-semibold">Viewpoint generation settings</div><div className="mt-0.5 text-xs text-zinc-500">Matches the reference ROS generator parameters; execution stays in the existing CLI workflow.</div></div><Sparkles size={16} className="text-zinc-400"/></div>
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
