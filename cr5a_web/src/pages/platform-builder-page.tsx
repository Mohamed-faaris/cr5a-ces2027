import {useCallback,useEffect,useMemo,useRef,useState} from "react";
import {AlertTriangle,CheckCircle2,Download,FileUp,Layers3,Plus,RefreshCw,Save,Scan,Trash2,Upload} from "lucide-react";
import {Button} from "@/components/ui/button";
import {RobotViewport} from "@/components/robot/robot-viewport";
import {useRobotSocket} from "@/lib/socket";
import type {CollisionObject} from "@/lib/types";
import {useRobotStore} from "@/store/robot-store";
import {loadVisualizerViewPreferences,saveVisualizerViewPreferences,type VisualizerViewPreferences} from "@/lib/visualizer-view-preferences";

type Mode="four_corners"|"two_corners"|"center_dimensions";
type Source="tf"|"manual";
type MarkerInput={id:string;corner:string;source_frame:string;x_mm:string;y_mm:string;z_mm:string;yaw_deg:string};
type ResolvedMarker={id:string;corner:string;frame_id:string;source_frame:string;position_m:[number,number,number];orientation_xyzw:[number,number,number,number];transform_applied?:boolean;tf?:any};
type PlatformGeometry={id:string;frame_id:string;position_m:[number,number,number];dimensions_m:[number,number,number];orientation_xyzw:[number,number,number,number];top_z_m:number;ground_z_m:number;quality:{plane_rms_m:number;height_spread_m:number;corner_fit_rms_m:number;max_marker_tilt_deg:number;marker_count:number;source_confidence:string};warnings:string[];loaded?:boolean};
type Profile={name:string;mode:Mode;source:Source;tag_family:string;tag_size_mm:number;markers:MarkerInput[];width_mm:number;depth_mm:number;ground_z_mm:number;margin_mm:number;updated_at:string};
type Capture={id:string;name:string;created_at:string;mode:Mode;geometry:PlatformGeometry;markers:ResolvedMarker[]};

const PROFILE_KEY="cr5a-platform-builder-profiles-v1";
const CAPTURE_KEY="cr5a-platform-builder-captures-v1";
const CORNERS=["front_left","front_right","back_right","back_left"];
function defaultMarkers(mode:Mode):MarkerInput[]{
  const corners=mode==="four_corners"?CORNERS:mode==="two_corners"?["front_left","back_right"]:["center"];
  return corners.map((corner,index)=>({id:String(index),corner,source_frame:`tag_${index}`,x_mm:"",y_mm:"",z_mm:"",yaw_deg:"0"}));
}
function modeCount(mode:Mode){return mode==="four_corners"?4:mode==="two_corners"?2:1;}
function isObject(value:any):value is Record<string,any>{return value!==null&&typeof value==="object"&&!Array.isArray(value);}
function readList<T>(key:string):T[]{try{const raw=localStorage.getItem(key);const parsed=raw?JSON.parse(raw):[];return Array.isArray(parsed)?parsed:[];}catch{return [];}}
function mm(value:number,digits=1){return Number.isFinite(value)?(value*1000).toFixed(digits):"—";}
function radians(degrees:number){return degrees*Math.PI/180;}
function profileFromState(name:string,mode:Mode,source:Source,tagFamily:string,tagSize:number,markers:MarkerInput[],width:number,depth:number,ground:number,margin:number):Profile{
  return {name,mode,source,tag_family:tagFamily,tag_size_mm:tagSize,markers,width_mm:width,depth_mm:depth,ground_z_mm:ground,margin_mm:margin,updated_at:new Date().toISOString()};
}

function NumberCell({label,value,unit,min=-2000,max=2000,step=1,onChange}:{label:string;value:number;unit:string;min?:number;max?:number;step?:number;onChange:(value:number)=>void}){
  return <label className="block min-w-0"><span className="mb-1 block text-[10px] font-semibold uppercase tracking-wider text-zinc-500">{label}</span><div className="flex items-center gap-1"><input type="number" value={value} min={min} max={max} step={step} onChange={event=>onChange(Number(event.currentTarget.value))} className="h-8 min-w-0 w-full rounded-md border border-zinc-200 bg-white px-2 font-mono text-xs"/><span className="text-[10px] text-zinc-400">{unit}</span></div></label>;
}

export function PlatformBuilderPage(){
  const model=useRobotStore(state=>state.model);
  const modelLoading=useRobotStore(state=>state.modelLoading);
  const modelError=useRobotStore(state=>state.modelError);
  const bridgeState=useRobotStore(state=>state.bridgeState);
  const scene=useRobotStore(state=>state.scene);
  const plan=useRobotStore(state=>state.plan);
  const {request,subscribeChannels}=useRobotSocket();
  const [mode,setMode]=useState<Mode>("four_corners");
  const [source,setSource]=useState<Source>("tf");
  const [tagFamily,setTagFamily]=useState("tag36h11");
  const [tagSize,setTagSize]=useState(40);
  const [framePrefix,setFramePrefix]=useState("tag_");
  const [detectorStatus,setDetectorStatus]=useState<any>(null);
  const [detectorBusy,setDetectorBusy]=useState(false);
  const [markers,setMarkers]=useState<MarkerInput[]>(()=>defaultMarkers("four_corners"));
  const [width,setWidth]=useState(600);
  const [depth,setDepth]=useState(400);
  const [groundZ,setGroundZ]=useState(-742.9);
  const [margin,setMargin]=useState(0);
  const [resolved,setResolved]=useState<ResolvedMarker[]>([]);
  const [geometry,setGeometry]=useState<PlatformGeometry|null>(null);
  const [busy,setBusy]=useState<"resolve"|"preview"|"load"|"remove"|null>(null);
  const [error,setError]=useState<string|null>(null);
  const [notice,setNotice]=useState<string|null>(null);
  const [profiles,setProfiles]=useState<Profile[]>(()=>readList<Profile>(PROFILE_KEY));
  const [captures,setCaptures]=useState<Capture[]>(()=>readList<Capture>(CAPTURE_KEY));
  const [profileName,setProfileName]=useState("platform-default");
  const [viewPreferences,setViewPreferences]=useState<VisualizerViewPreferences>(loadVisualizerViewPreferences);
  const profileInput=useRef<HTMLInputElement>(null);
  const captureInput=useRef<HTMLInputElement>(null);
  const baseFrame=model?.base_frame??"dummy_link";
  const objectId="platform_builder_platform";
  const activeModeLabel=mode==="four_corners"?"Four corners":mode==="two_corners"?"Two opposite corners":"Center + dimensions";
  const requiredCount=modeCount(mode);

  useEffect(()=>saveVisualizerViewPreferences(viewPreferences),[viewPreferences]);
  useEffect(()=>localStorage.setItem(PROFILE_KEY,JSON.stringify(profiles)),[profiles]);
  useEffect(()=>localStorage.setItem(CAPTURE_KEY,JSON.stringify(captures)),[captures]);
  useEffect(()=>subscribeChannels(["topic:/tf","topic:/tf_static"]),[subscribeChannels]);

  const updateMarker=(index:number,patch:Partial<MarkerInput>)=>setMarkers(current=>current.map((marker,i)=>i===index?{...marker,...patch}:marker));
  const changeMode=(next:Mode)=>{setMode(next);setMarkers(defaultMarkers(next));setResolved([]);setGeometry(null);setError(null);};

  const buildMarkerPayload=()=>markers.slice(0,requiredCount).map(marker=>{
    const result:any={id:marker.id,corner:marker.corner,source_frame:marker.source_frame||baseFrame};
    if(source==="manual"){
      const xyz=[marker.x_mm,marker.y_mm,marker.z_mm];
      if(xyz.some(value=>value.trim()===""))throw new Error(`Enter X/Y/Z for marker ${marker.id}.`);
      const values=xyz.map(Number);
      if(!values.every(Number.isFinite))throw new Error(`Marker ${marker.id} has invalid coordinates.`);
      const yaw=Number(marker.yaw_deg||0);
      if(!Number.isFinite(yaw))throw new Error(`Marker ${marker.id} has an invalid yaw.`);
      result.position_m=values.map(value=>value/1000);
      result.orientation_xyzw=[0,0,Math.sin(radians(yaw)/2),Math.cos(radians(yaw)/2)];
    }
    return result;
  });

  const refreshDetectorStatus=useCallback(async()=>{
    try{
      const result=await request("apriltag.get_status");
      setDetectorStatus(result.status??null);
    }catch(value){setDetectorStatus({state:"unavailable",error:value instanceof Error?value.message:String(value)});}
  },[request]);
  useEffect(()=>{
    if(bridgeState==="connected")void refreshDetectorStatus();
  },[bridgeState,refreshDetectorStatus]);
  const startAprilTagDetector=async()=>{
    if(detectorBusy||bridgeState!=="connected")return;
    setDetectorBusy(true);setError(null);setNotice(null);
    try{
      const ids=markers.slice(0,requiredCount).map(marker=>marker.id.trim()).filter(Boolean);
      if(ids.length!==requiredCount)throw new Error("Enter every AprilTag ID before starting detection.");
      await request("apriltag.configure",{parameters:{tag_family:tagFamily,tag_size_m:tagSize/1000,tag_ids:ids.join(","),frame_prefix:framePrefix}});
      const result=await request("apriltag.start");
      setNotice(result.message??"AprilTag detector started.");
      await new Promise(resolve=>window.setTimeout(resolve,350));
      await refreshDetectorStatus();
    }catch(value){setError(value instanceof Error?value.message:String(value));}
    finally{setDetectorBusy(false);}
  };
  const stopAprilTagDetector=async()=>{
    setDetectorBusy(true);setError(null);
    try{await request("apriltag.stop");await refreshDetectorStatus();setNotice("AprilTag detector stopped.");}
    catch(value){setError(value instanceof Error?value.message:String(value));}
    finally{setDetectorBusy(false);}
  };

  const resolveMarkers=useCallback(async()=>{
    if(!model||bridgeState!=="connected")return;
    setBusy("resolve");setError(null);setNotice(null);setGeometry(null);
    try{
      const raw=buildMarkerPayload();
      const result=await request("scene_builder.resolve_markers",{markers:raw,base_frame:baseFrame,tf_timeout_sec:0.35});
      setResolved(result.markers as ResolvedMarker[]);
      setNotice(`${result.resolved_count} marker pose(s) transformed into ${result.frame_id}.`);
    }catch(value){setResolved([]);setError(value instanceof Error?value.message:String(value));}
    finally{setBusy(null);}
  },[model,bridgeState,markers,requiredCount,source,baseFrame,request]);

  const computeGeometry=useCallback(async()=>{
    if(resolved.length!==requiredCount)return;
    setBusy("preview");setError(null);setNotice(null);
    try{
      const result=await request("scene_builder.preview",{
        mode,markers:resolved.map(({id,corner,position_m,orientation_xyzw})=>({id,corner,position_m,orientation_xyzw})),
        ground_z_m:groundZ/1000,width_m:width/1000,depth_m:depth/1000,margin_m:margin/1000,
        object_id:objectId,max_marker_tilt_deg:15,max_height_spread_m:0.035,max_corner_fit_rms_m:0.05,
      });
      setGeometry(result as PlatformGeometry);
      setNotice("Platform geometry fitted. Inspect tag orientation, top height and ground-plane clearance in 3D.");
    }catch(value){setGeometry(null);setError(value instanceof Error?value.message:String(value));}
    finally{setBusy(null);}
  },[resolved,requiredCount,mode,groundZ,width,depth,margin,request]);

  const loadCollision=async()=>{
    if(!geometry||busy)return;
    setBusy("load");setError(null);setNotice(null);
    try{
      const result=await request("scene_builder.construct",{
        mode,markers:resolved.map(({id,corner,position_m,orientation_xyzw})=>({id,corner,position_m,orientation_xyzw})),
        ground_z_m:groundZ/1000,width_m:width/1000,depth_m:depth/1000,margin_m:margin/1000,
        object_id:objectId,max_marker_tilt_deg:15,max_height_spread_m:0.035,max_corner_fit_rms_m:0.05,
      });
      setGeometry(result as PlatformGeometry);
      const nextScene=await request("moveit.get_scene");
      useRobotStore.getState().setScene(nextScene);
      setNotice(`Loaded ${result.id} into MoveIt collision scene · ${result.frame_id}.`);
    }catch(value){setError(value instanceof Error?value.message:String(value));}
    finally{setBusy(null);}
  };

  const removeCollision=async()=>{
    setBusy("remove");setError(null);
    try{
      await request("scene_builder.remove",{object_id:objectId});
      const nextScene=await request("moveit.get_scene");
      useRobotStore.getState().setScene(nextScene);
      setGeometry(current=>current?{...current,loaded:false}:null);
      setNotice("Platform collision object removed from MoveIt.");
    }catch(value){setError(value instanceof Error?value.message:String(value));}
    finally{setBusy(null);}
  };

  const captureSnapshot=()=>{
    if(!geometry||resolved.length!==requiredCount)return;
    const capture:Capture={id:`capture-${Date.now()}`,name:`Capture ${captures.length+1}`,created_at:new Date().toISOString(),mode,geometry,markers:resolved};
    setCaptures(current=>[capture,...current].slice(0,20));
    setNotice(`Saved geometry snapshot with ${geometry.quality.marker_count} marker(s).`);
  };
  const restoreCapture=(capture:Capture)=>{setMode(capture.mode);setResolved(capture.markers);setGeometry({...capture.geometry,loaded:false});setNotice(`Restored ${capture.name}; re-check the current TF/calibration before loading it.`);};

  const saveProfile=()=>{
    const name=profileName.trim();if(!name){setError("Enter a profile name before saving.");return;}
    const profile=profileFromState(name,mode,source,tagFamily,tagSize,markers,width,depth,groundZ,margin);
    setProfiles(current=>[profile,...current.filter(item=>item.name!==name)]);
    setNotice(`Saved profile “${name}” in this browser.`);setError(null);
  };
  const loadProfile=(profile:Profile)=>{
    setProfileName(profile.name);setMode(profile.mode);setSource(profile.source);setTagFamily(profile.tag_family);setTagSize(profile.tag_size_mm);
    setMarkers(profile.markers);setWidth(profile.width_mm);setDepth(profile.depth_mm);setGroundZ(profile.ground_z_mm);setMargin(profile.margin_mm);
    setResolved([]);setGeometry(null);setNotice(`Loaded profile “${profile.name}”. Resolve the markers again before adding collisions.`);
  };
  const exportJson=(name:string,data:unknown)=>{
    const blob=new Blob([JSON.stringify(data,null,2)],{type:"application/json"});
    const url=URL.createObjectURL(blob);const link=document.createElement("a");link.href=url;link.download=name;link.click();URL.revokeObjectURL(url);
  };
  const importProfile=async(file?:File)=>{
    if(!file)return;
    try{
      const parsed=JSON.parse(await file.text());
      if(!isObject(parsed)||!parsed.name||!parsed.mode||!Array.isArray(parsed.markers))throw new Error("Not a Platform Builder profile JSON.");
      const profile=parsed as Profile;
      if(!["four_corners","two_corners","center_dimensions"].includes(profile.mode))throw new Error("Unsupported profile mode.");
      setProfiles(current=>[profile,...current.filter(item=>item.name!==profile.name)]);loadProfile(profile);
    }catch(value){setError(value instanceof Error?value.message:String(value));}
    finally{if(profileInput.current)profileInput.current.value="";}
  };
  const importCapture=async(file?:File)=>{
    if(!file)return;
    try{
      const parsed=JSON.parse(await file.text());
      if(!isObject(parsed)||!Array.isArray(parsed.markers)||!parsed.mode)throw new Error("Capture JSON needs mode and markers.");
      const data=parsed as Capture;
      if(!data.geometry||!data.markers.length)throw new Error("Capture needs fitted geometry and marker poses.");
      setCaptures(current=>[{...data,id:`import-${Date.now()}`,name:data.name||file.name},...current].slice(0,20));
      setNotice(`Imported capture ${file.name}.`);
    }catch(value){setError(value instanceof Error?value.message:String(value));}
    finally{if(captureInput.current)captureInput.current.value="";}
  };

  const previewObject=useMemo(()=>geometry?({
    id:objectId,header:{frame_id:baseFrame},
    pose:{position:{x:geometry.position_m[0],y:geometry.position_m[1],z:geometry.position_m[2]},orientation:{x:geometry.orientation_xyzw[0],y:geometry.orientation_xyzw[1],z:geometry.orientation_xyzw[2],w:geometry.orientation_xyzw[3]}},
    primitives:[{type:1,dimensions:geometry.dimensions_m}],
    primitive_poses:[{position:{x:0,y:0,z:0},orientation:{x:0,y:0,z:0,w:1}}],
  }):null,[geometry,baseFrame]);
  const disabledObjects=useMemo<Record<string,CollisionObject>>(()=>previewObject?{[objectId]:previewObject as CollisionObject}:({} as Record<string,CollisionObject>),[previewObject]);
  const emptyLabels=useMemo(()=>({}),[]);
  const emptyHidden=useMemo(()=>({}),[]);
  const poseMarkers=useMemo(()=>resolved.map(marker=>({id:`tag-${marker.id}`,label:`TAG ${marker.id}`,position:marker.position_m as [number,number,number],orientation:marker.orientation_xyzw as [number,number,number,number],frame_id:baseFrame,color:"#0d9488",selected:true})),[resolved,baseFrame]);
  const allSceneObjects=scene?.world?.collision_objects??[];
  const loadedInScene=allSceneObjects.some(item=>item.id===objectId);
  const statusText=!model?"Waiting for robot model":bridgeState!=="connected"?"Bridge disconnected":resolved.length===requiredCount?"Markers resolved in base frame":"Markers not resolved";
  const recommendedCapture=[...captures].sort((a,b)=>(a.geometry.quality.plane_rms_m+a.geometry.quality.corner_fit_rms_m)-(b.geometry.quality.plane_rms_m+b.geometry.quality.corner_fit_rms_m))[0];

  return <div className="space-y-4">
    <div className="flex flex-wrap items-start justify-between gap-3"><div><div className="eyebrow">SCENE TOOLS / APRILTAG PLATFORM CONSTRUCTION</div><h1 className="page-title">Platform Builder</h1><p className="mt-1 max-w-3xl text-sm text-zinc-500">Place AprilTags on a platform, transform their poses through the calibrated camera TF chain, then build a solid collision volume down to the ground plane.</p></div><span className={`rounded-full px-2.5 py-1 text-[10px] font-semibold ${bridgeState==="connected"?"bg-emerald-50 text-emerald-700":"bg-amber-50 text-amber-700"}`}>{statusText}</span></div>

    <section className="panel"><div className="panel-header"><div><div className="flex items-center gap-2"><Scan size={16}/><span className="text-sm font-semibold">Construction mode</span></div><p className="mt-0.5 text-[10px] text-zinc-500">All AprilTag normals must face upward (+Z in the MoveIt base frame).</p></div></div>
      <div className="grid gap-2 p-3 md:grid-cols-3">
        {[{value:"four_corners",title:"Four corners",desc:"One tag on each corner. Fit a plane and rectangle from front-left, front-right, back-right and back-left."},{value:"two_corners",title:"Two opposite corners",desc:"Two diagonal tags. Their common horizontal X axis defines platform width/depth directions."},{value:"center_dimensions",title:"Center + dimensions",desc:"One center tag. Enter width and depth manually; tag orientation defines the platform yaw."}].map(item=><button key={item.value} type="button" onClick={()=>changeMode(item.value as Mode)} className={`rounded-lg border p-3 text-left ${mode===item.value?"border-zinc-900 bg-zinc-900 text-white":"border-zinc-200 bg-white text-zinc-700 hover:border-zinc-400"}`}><div className="text-xs font-semibold">{item.title}</div><div className={`mt-1 text-[10px] leading-4 ${mode===item.value?"text-zinc-300":"text-zinc-500"}`}>{item.desc}</div></button>)}
      </div>
    </section>

    <div className="grid gap-4 xl:grid-cols-[minmax(0,1fr)_390px]">
      <section className="panel overflow-hidden"><div className="panel-header"><div><div className="text-sm font-semibold">3D construction preview</div><p className="mt-0.5 text-[10px] text-zinc-500">Teal spheres/axes are resolved AprilTags. Orange translucent box is the proposed collision volume.</p></div><span className="text-[10px] text-zinc-400">{baseFrame} · metres</span></div>
        <div className="p-2"><RobotViewport model={model} scene={scene} plan={plan} poseMarkers={poseMarkers} loading={modelLoading} error={modelError} viewPreferences={viewPreferences} onViewPreferencesChange={setViewPreferences} objectLabels={emptyLabels} disabledObjects={disabledObjects} hiddenObjects={emptyHidden}/></div>
        <div className="flex flex-wrap items-center justify-between gap-2 border-t border-zinc-100 px-4 py-2"><span className="text-[10px] text-zinc-500">{geometry?`Preview box · ${geometry.dimensions_m.map(v=>mm(v)).join(" × ")} mm`:"Resolve tags and calculate geometry to preview the platform."}</span><span className="text-[10px] text-zinc-500">{loadedInScene?"Collision object loaded in MoveIt":geometry?.loaded?"Loaded":"Preview only"}</span></div>
      </section>

      <aside className="space-y-4">
        <section className="panel"><div className="panel-header"><div><div className="text-sm font-semibold">AprilTag configuration</div><p className="mt-0.5 text-[10px] text-zinc-500">Select the family/size used by your printed markers.</p></div></div>
          <div className="space-y-3 p-4"><div className="grid grid-cols-2 gap-2"><label className="block"><span className="mb-1 block text-[10px] font-semibold uppercase tracking-wider text-zinc-500">Family</span><select value={tagFamily} onChange={event=>setTagFamily(event.currentTarget.value)} className="h-8 w-full rounded-md border border-zinc-200 px-2 text-xs"><option>tag36h11</option><option>tag25h9</option><option>tag16h5</option></select></label><NumberCell label="Printed tag size" value={tagSize} unit="mm" min={10} max={200} onChange={setTagSize}/></div>
            <label className="block"><span className="mb-1 block text-[10px] font-semibold uppercase tracking-wider text-zinc-500">Pose source</span><select value={source} onChange={event=>{const next=event.currentTarget.value as Source;setSource(next);setMarkers(current=>current.map(marker=>({...marker,source_frame:next==="manual"?baseFrame:`${framePrefix}${marker.id}`})));setResolved([]);setGeometry(null);}} className="h-8 w-full rounded-md border border-zinc-200 px-2 text-xs"><option value="tf">Live AprilTag TF frames</option><option value="manual">Manual marker pose entry</option></select></label>
            <label className="block"><span className="mb-1 block text-[10px] font-semibold uppercase tracking-wider text-zinc-500">Published tag TF prefix</span><input value={framePrefix} onChange={event=>setFramePrefix(event.currentTarget.value)} className="h-8 w-full rounded-md border border-zinc-200 px-2 font-mono text-xs"/></label>
            <div className="rounded-md border border-amber-200 bg-amber-50 p-2.5 text-[10px] leading-4 text-amber-900"><b>Camera/extrinsic rule:</b> TF mode looks up `base_frame ← tag_frame`. The camera-to-base transform must already exist through the robot URDF or imported hand-eye calibration. This page does not estimate or guess camera extrinsics. AprilTag detector output is published as `tag_&lt;id&gt;` by default.</div>
            <div className="rounded-lg border border-zinc-200 p-3"><div className="flex flex-wrap items-center justify-between gap-2"><div><div className="text-xs font-semibold">Live AprilTag detector</div><div className="mt-0.5 text-[10px] text-zinc-500">RGB: /cr5/wrist_camera/color/image_raw · CameraInfo: /cr5/wrist_camera/color/camera_info</div></div><span className={`rounded px-2 py-1 text-[9px] font-semibold ${detectorStatus?.active?"bg-emerald-50 text-emerald-700":"bg-zinc-100 text-zinc-500"}`}>{detectorStatus?.state??"Unknown"}</span></div><div className="mt-2 flex flex-wrap gap-2"><Button size="sm" disabled={detectorBusy||bridgeState!=="connected"} onClick={startAprilTagDetector}><Scan size={12}/>{detectorBusy?"Starting…":"Apply IDs & start detector"}</Button><Button size="sm" variant="outline" disabled={detectorBusy} onClick={stopAprilTagDetector}>Stop</Button><Button size="sm" variant="outline" disabled={detectorBusy} onClick={refreshDetectorStatus}>Refresh status</Button></div>{detectorStatus?.message&&<div className="mt-2 text-[10px] text-zinc-600">{detectorStatus.message}</div>}{detectorStatus?.error&&<div className="mt-1 text-[10px] text-red-700">{detectorStatus.error}</div>}{detectorStatus?.active&&<div className="mt-1 font-mono text-[9px] text-emerald-700">Detected IDs: {(detectorStatus.last_ids??[]).join(", ")||"none yet"} · frames {detectorStatus.frame_count??0}</div>}</div>
          </div>
        </section>

        <section className="panel"><div className="panel-header"><div><div className="text-sm font-semibold">Marker poses · {requiredCount} required</div><p className="mt-0.5 text-[10px] text-zinc-500">Mode: {activeModeLabel}</p></div><span className="text-[10px] text-zinc-400">{resolved.length}/{requiredCount} resolved</span></div>
          <div className="divide-y divide-zinc-100">{markers.slice(0,requiredCount).map((marker,index)=><div key={`${mode}-${index}`} className="space-y-2 p-3"><div className="flex flex-wrap items-center justify-between gap-2"><div className="text-xs font-semibold">Marker {index+1} · {marker.corner.replace(/_/g," ")}</div><span className="font-mono text-[10px] text-zinc-400">ID {marker.id}</span></div>
            <div className="grid grid-cols-2 gap-2"><label className="block"><span className="mb-1 block text-[9px] font-semibold uppercase tracking-wider text-zinc-400">Tag ID</span><input value={marker.id} onChange={event=>updateMarker(index,{id:event.currentTarget.value})} className="h-8 w-full rounded border border-zinc-200 px-2 font-mono text-xs"/></label><label className="block"><span className="mb-1 block text-[9px] font-semibold uppercase tracking-wider text-zinc-400">TF child frame</span><input value={marker.source_frame} onChange={event=>updateMarker(index,{source_frame:event.currentTarget.value})} className="h-8 w-full rounded border border-zinc-200 px-2 font-mono text-xs" placeholder={`tag_${marker.id}`}/></label></div>
            {mode==="four_corners"&&<label className="block"><span className="mb-1 block text-[9px] font-semibold uppercase tracking-wider text-zinc-400">Corner assignment</span><select value={marker.corner} onChange={event=>updateMarker(index,{corner:event.currentTarget.value})} className="h-8 w-full rounded border border-zinc-200 px-2 text-xs">{CORNERS.map(corner=><option key={corner} value={corner}>{corner.replace(/_/g," ")}</option>)}</select></label>}
            {source==="manual"&&<><div className="grid grid-cols-4 gap-2">{(["x_mm","y_mm","z_mm","yaw_deg"] as const).map(key=><label key={key} className="block"><span className="mb-1 block text-[9px] font-semibold uppercase tracking-wider text-zinc-400">{key.replace("_mm","").replace("_deg","")}</span><input type="number" value={marker[key]} onChange={event=>updateMarker(index,{[key]:event.currentTarget.value})} className="h-8 w-full rounded border border-zinc-200 px-1.5 font-mono text-[11px]" placeholder="—"/></label>)}</div><div className="text-[9px] text-zinc-400">Manual XYZ is expressed in the selected source frame. Roll/pitch are fixed at zero by the “facing up” assumption; yaw is in degrees.</div></>}
            {resolved[index]&&<div className="rounded bg-emerald-50 px-2 py-1.5 font-mono text-[9px] text-emerald-800">Base: {resolved[index].position_m.map(v=>v.toFixed(4)).join(", ")} m · TF {resolved[index].transform_applied?"applied":"not required"}</div>}
          </div>)}</div>
          <div className="border-t border-zinc-100 p-3"><Button className="w-full" size="sm" disabled={!model||bridgeState!=="connected"||busy!==null} onClick={resolveMarkers}><RefreshCw size={13}/>{busy==="resolve"?"Resolving marker TF…":source==="tf"?"Capture marker poses from TF":"Resolve manual marker poses"}</Button></div>
        </section>

        {mode==="center_dimensions"&&<section className="panel"><div className="panel-header"><div className="text-sm font-semibold">Manual platform dimensions</div></div><div className="grid grid-cols-2 gap-3 p-3"><NumberCell label="Platform width" value={width} unit="mm" min={50} onChange={setWidth}/><NumberCell label="Platform depth" value={depth} unit="mm" min={50} onChange={setDepth}/></div></section>}

        <section className="panel"><div className="panel-header"><div><div className="text-sm font-semibold">Ground connection</div><p className="mt-0.5 text-[10px] text-zinc-500">The box spans from platform top height down to this floor height.</p></div></div><div className="grid grid-cols-2 gap-3 p-3"><NumberCell label="Ground plane Z" value={groundZ} unit="mm" onChange={setGroundZ}/><NumberCell label="Edge margin" value={margin} unit="mm" min={0} max={250} onChange={setMargin}/></div><div className="border-t border-zinc-100 px-3 py-2 text-[10px] text-zinc-500">Current floor setting matches the default MoveIt ground collision top at approximately −742.9 mm. Confirm this against your physical floor/robot base before loading.</div></section>
      </aside>
    </div>

    <section className="panel"><div className="panel-header"><div><div className="text-sm font-semibold">Fit, compare and load collision geometry</div><p className="mt-0.5 text-[10px] text-zinc-500">Quality values compare marker coplanarity and rectangular fit; they are not a ground-truth camera-extrinsic accuracy estimate.</p></div><div className="flex flex-wrap gap-2"><Button size="sm" variant="outline" disabled={resolved.length!==requiredCount||busy!==null} onClick={computeGeometry}><Layers3 size={13}/>{busy==="preview"?"Fitting…":"Fit plane & preview box"}</Button><Button size="sm" disabled={!geometry||busy!==null||loadedInScene} onClick={loadCollision}><CheckCircle2 size={13}/>{busy==="load"?"Loading…":"Load collision box into MoveIt"}</Button></div></div>
      {geometry&&<div className="grid gap-3 p-4 sm:grid-cols-2 lg:grid-cols-4"><div className="rounded-md bg-zinc-50 p-3"><div className="text-[9px] font-semibold uppercase tracking-wider text-zinc-400">Platform dimensions</div><div className="mt-1 font-mono text-xs font-semibold">{geometry.dimensions_m.map(v=>mm(v)).join(" × ")} mm</div><div className="mt-1 text-[9px] text-zinc-500">Width × depth × ground thickness</div></div><div className="rounded-md bg-zinc-50 p-3"><div className="text-[9px] font-semibold uppercase tracking-wider text-zinc-400">Top plane RMS</div><div className="mt-1 font-mono text-xs font-semibold">{(geometry.quality.plane_rms_m*1000).toFixed(2)} mm</div><div className="mt-1 text-[9px] text-zinc-500">Marker height consistency</div></div><div className="rounded-md bg-zinc-50 p-3"><div className="text-[9px] font-semibold uppercase tracking-wider text-zinc-400">Rectangle fit RMS</div><div className="mt-1 font-mono text-xs font-semibold">{(geometry.quality.corner_fit_rms_m*1000).toFixed(2)} mm</div><div className="mt-1 text-[9px] text-zinc-500">Four-corner consistency</div></div><div className="rounded-md bg-zinc-50 p-3"><div className="text-[9px] font-semibold uppercase tracking-wider text-zinc-400">Max normal tilt</div><div className="mt-1 font-mono text-xs font-semibold">{geometry.quality.max_marker_tilt_deg.toFixed(2)}°</div><div className="mt-1 text-[9px] text-zinc-500">Required: tags facing up</div></div></div>}
      {error&&<div className="mx-4 mb-3 flex items-start gap-2 rounded-md border border-red-200 bg-red-50 p-3 text-xs text-red-800"><AlertTriangle size={14} className="mt-0.5 shrink-0"/>{error}</div>}
      {notice&&<div className="mx-4 mb-3 rounded-md border border-emerald-200 bg-emerald-50 p-3 text-xs text-emerald-800">{notice}</div>}
      <div className="flex flex-wrap items-center justify-between gap-3 border-t border-zinc-100 p-3"><div className="text-[10px] text-zinc-500">{loadedInScene?"Platform collision is active in MoveIt.":"Preview does not affect collision checking until loaded."}</div><Button size="sm" variant="outline" disabled={!loadedInScene||busy!==null} onClick={removeCollision}><Trash2 size={12}/>Remove platform collision</Button></div>
    </section>

    <section className="grid gap-4 lg:grid-cols-2"><div className="panel"><div className="panel-header"><div><div className="text-sm font-semibold">Save / load construction profiles</div><p className="mt-0.5 text-[10px] text-zinc-500">Profiles are saved in this browser; export JSON to reuse them on another workstation.</p></div></div><div className="space-y-3 p-3"><div className="flex gap-2"><input value={profileName} onChange={event=>setProfileName(event.currentTarget.value)} className="h-8 min-w-0 flex-1 rounded border border-zinc-200 px-2 text-xs"/><Button size="sm" onClick={saveProfile}><Save size={12}/>Save</Button></div><div className="flex flex-wrap gap-2"><Button size="sm" variant="outline" onClick={()=>exportJson(`platform-${profileName||"profile"}.json`,profileFromState(profileName||"platform",mode,source,tagFamily,tagSize,markers,width,depth,groundZ,margin))}><Download size={12}/>Export profile</Button><Button size="sm" variant="outline" onClick={()=>profileInput.current?.click()}><Upload size={12}/>Import profile</Button><input ref={profileInput} type="file" accept="application/json,.json" className="hidden" onChange={event=>void importProfile(event.currentTarget.files?.[0])}/></div>
      {profiles.length===0?<div className="py-4 text-center text-[10px] text-zinc-400">No saved profiles yet.</div>:<div className="divide-y divide-zinc-100">{profiles.map(profile=><div key={profile.name} className="flex items-center justify-between gap-2 py-2"><div className="min-w-0"><div className="truncate text-xs font-medium">{profile.name}</div><div className="text-[9px] text-zinc-400">{profile.mode.replace(/_/g," ")} · {profile.tag_family} · {new Date(profile.updated_at).toLocaleString()}</div></div><div className="flex gap-1"><Button size="sm" variant="outline" onClick={()=>loadProfile(profile)}>Load</Button><Button size="sm" variant="ghost" onClick={()=>exportJson(`platform-${profile.name}.json`,profile)}><Download size={12}/></Button><Button size="sm" variant="ghost" onClick={()=>setProfiles(current=>current.filter(item=>item.name!==profile.name))}><Trash2 size={12}/></Button></div></div>)}</div>}</div></div>
      <div className="panel"><div className="panel-header"><div><div className="text-sm font-semibold">Capture comparison</div><p className="mt-0.5 text-[10px] text-zinc-500">Save repeated marker captures and compare fit errors. Best geometric consistency is ranked first.</p></div><Button size="sm" variant="outline" disabled={!geometry||resolved.length!==requiredCount} onClick={captureSnapshot}><Plus size={12}/>Save capture</Button></div><div className="p-3">{recommendedCapture&&<div className="mb-2 rounded bg-emerald-50 px-3 py-2 text-[10px] text-emerald-800">Best geometric fit: <b>{recommendedCapture.name}</b> · plane + corner RMS {(1000*(recommendedCapture.geometry.quality.plane_rms_m+recommendedCapture.geometry.quality.corner_fit_rms_m)).toFixed(2)} mm</div>}{captures.length===0?<div className="py-4 text-center text-[10px] text-zinc-400">No captures. Resolve markers, fit geometry, then save a capture.</div>:<div className="divide-y divide-zinc-100">{captures.map(capture=><div key={capture.id} className="flex items-center justify-between gap-2 py-2"><div className="min-w-0"><div className="text-xs font-medium">{capture.name}</div><div className="text-[9px] text-zinc-400">{capture.mode.replace(/_/g," ")} · plane {(capture.geometry.quality.plane_rms_m*1000).toFixed(2)} mm · corners {(capture.geometry.quality.corner_fit_rms_m*1000).toFixed(2)} mm</div></div><div className="flex gap-1"><Button size="sm" variant="outline" onClick={()=>restoreCapture(capture)}>Restore</Button><Button size="sm" variant="ghost" onClick={()=>exportJson(`${capture.name.replace(/ /g,"-")}.json`,capture)}><Download size={12}/></Button><Button size="sm" variant="ghost" onClick={()=>setCaptures(current=>current.filter(item=>item.id!==capture.id))}><Trash2 size={12}/></Button></div></div>)}</div>}<div className="mt-2 border-t border-zinc-100 pt-2"><Button size="sm" variant="outline" onClick={()=>captureInput.current?.click()}><FileUp size={12}/>Import capture JSON</Button><input ref={captureInput} type="file" accept="application/json,.json" className="hidden" onChange={event=>void importCapture(event.currentTarget.files?.[0])}/></div></div></div></section>

    <div className="rounded-lg border border-amber-200 bg-amber-50 p-3 text-[10px] leading-5 text-amber-900"><b>AprilTag / TF status:</b> {detectorStatus?.active?`Detector ${detectorStatus.state}: ${detectorStatus.message??"waiting for camera frames"}.` : detectorStatus?.state==="unavailable"?`Built-in detector is unavailable: ${detectorStatus.error??"start the CR5A Vision launch"}.` : detectorStatus?.error?`Detector error: ${detectorStatus.error}.` : "Built-in detector is stopped. Set the marker IDs above and start it to publish tag TF frames."} {detectorStatus?.camera_info_ready===false?"CameraInfo has not arrived; metric tag poses cannot be estimated yet. ":""} TF mode requires a valid camera-to-base chain from the robot URDF or hand-eye calibration. This page never guesses camera extrinsics. Manual frame poses also require a valid TF from their source frame to {baseFrame}.</div>
  </div>;
}
