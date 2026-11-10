import { useEffect, useMemo, useState, type ChangeEvent } from "react";
import { useNavigate } from "react-router-dom";
import { AlertTriangle, Box, Boxes, Camera, Check, ChevronRight, CircleStop, Eye, FileUp, Focus, Layers3, Play, ScanEye, Send, Settings2, Square, Upload } from "lucide-react";
import { Button } from "@/components/ui/button";
import { useRobotSocket } from "@/lib/socket";
import { useRobotStore } from "@/store/robot-store";

const STATUS_TOPIC="/cr5/vision/status";
const DETECTIONS_TOPIC="/cr5/vision/detections";
const PREVIEW_TOPIC="/cr5/vision/annotated/compressed";

type VisionDetection={
  id:string;class_name:string;confidence:number;shape:"box";frame_id:string;
  position_m:number[];centroid_m:number[];dimensions_m:number[];orientation_xyzw:number[];
  yaw_deg:number;valid_3d:boolean;source_backend:string;depth_points?:number;
};
type VisionStatus={state:string;active:boolean;backend:string;model_name?:string;model_family?:string;model_size?:string;image_size?:number;message:string;error?:string;base_frame:string;camera_frame:string;camera_info_ready:boolean;calibration_loaded:boolean;hand_eye_frames?:string[]|null;detections:number;frame_count:number};
const DEFAULT_TOPICS={camera:"/cr5/wrist_camera/color/image_raw",depth:"/cr5/wrist_camera/aligned_depth_to_color/image_raw",info:"/cr5/wrist_camera/color/camera_info",boxernet:"/boxernet/detections_3d"};

function parseRosJson<T>(value:any,fallback:T):T{
  const raw=value?.data;
  if(typeof raw!=="string")return fallback;
  try{return JSON.parse(raw) as T;}catch{return fallback;}
}
function NumberField({label,value,unit,onChange,min=-2000,max=2000,step=10}:{label:string;value:number;unit:string;onChange:(value:number)=>void;min?:number;max?:number;step?:number}){
  return <label className="block min-w-0"><span className="mb-1 block text-[10px] font-semibold uppercase tracking-wider text-zinc-500">{label}</span><span className="flex h-9 items-center rounded-md border border-zinc-200 bg-white"><input type="number" min={min} max={max} step={step} value={value} onChange={event=>onChange(Number(event.currentTarget.value))} className="min-w-0 flex-1 bg-transparent px-2 font-mono text-xs outline-none"/><span className="px-2 text-[10px] text-zinc-400">{unit}</span></span></label>;
}

export function VisionPage(){
  const {request,subscribeChannels}=useRobotSocket();
  const navigate=useNavigate();
  const bridgeState=useRobotStore(state=>state.bridgeState);
  const model=useRobotStore(state=>state.model);
  const statusMessage=useRobotStore(state=>state.topicMessages[STATUS_TOPIC]);
  const detectionsMessage=useRobotStore(state=>state.topicMessages[DETECTIONS_TOPIC]);
  const previewMessage=useRobotStore(state=>state.topicMessages[PREVIEW_TOPIC]);
  const statusError=useRobotStore(state=>state.topicSubscriptionErrors[STATUS_TOPIC]);
  const detectionsError=useRobotStore(state=>state.topicSubscriptionErrors[DETECTIONS_TOPIC]);
  const [backend,setBackend]=useState<"yolo_seg"|"boxernet">("yolo_seg");
  const [modelFamily,setModelFamily]=useState<"yolo26-seg"|"yoloe-26-seg"|"yolo11-seg"|"yolov8-seg"|"custom">("yoloe-26-seg");
  const [modelSize,setModelSize]=useState<"n"|"s"|"m"|"l"|"x">("n");
  const [imageSize,setImageSize]=useState(640);
  const [classPrompts,setClassPrompts]=useState("Coca-Cola can, Pringles potato chip tube, cardboard box, wooden box");
  const [weightsPath,setWeightsPath]=useState("");
  const [confidence,setConfidence]=useState(.35);
  const [imageTopic,setImageTopic]=useState(DEFAULT_TOPICS.camera);
  const [depthTopic,setDepthTopic]=useState(DEFAULT_TOPICS.depth);
  const [cameraInfoTopic,setCameraInfoTopic]=useState(DEFAULT_TOPICS.info);
  const [boxernetTopic,setBoxernetTopic]=useState(DEFAULT_TOPICS.boxernet);
  const [baseFrame,setBaseFrame]=useState("dummy_link");
  const [handEyeYaml,setHandEyeYaml]=useState("");
  const [handEyeName,setHandEyeName]=useState("");
  const [handEyeParent,setHandEyeParent]=useState("Link6");
  const [busy,setBusy]=useState<"start"|"stop"|"once"|"add"|null>(null);
  const [notice,setNotice]=useState<{kind:"success"|"error"|"info";message:string}|null>(null);
  const [selectedIds,setSelectedIds]=useState<string[]>([]);
  const [autoTable,setAutoTable]=useState(false);
  const [conveyorCenter,setConveyorCenter]=useState({x:500,y:0,z:250});
  const [conveyorSize,setConveyorSize]=useState({x:700,y:500,z:500});
  const [tablePosition,setTablePosition]=useState({x:350,y:-350,z:0});
  const [tableSize,setTableSize]=useState({x:300,y:240,z:20});
  const [sceneLoadedIds,setSceneLoadedIds]=useState<string[]>([]);

  useEffect(()=>subscribeChannels(["topic:"+STATUS_TOPIC,"topic:"+DETECTIONS_TOPIC,"topic:"+PREVIEW_TOPIC]),[subscribeChannels]);
  useEffect(()=>{if(model?.base_frame)setBaseFrame(model.base_frame);},[model?.base_frame]);

  const status=parseRosJson<VisionStatus|null>(statusMessage,null);
  const detectionsEnvelope=parseRosJson<{frame_id?:string;camera_frame?:string;objects?:VisionDetection[]}>(detectionsMessage,{});
  const detections=detectionsEnvelope.objects??[];
  const preview=previewMessage?.data as {format?:string;data_base64?:string;header?:{frame_id?:string}}|undefined;
  const selected=detections.filter(item=>selectedIds.includes(item.id));
  const previewSrc=preview?.data_base64?`data:image/jpeg;base64,${preview.data_base64}`:null;

  const detectionRouting=useMemo(()=>new Map(detections.map(item=>{
    const [x,y,z]=item.position_m;
    const inside=Math.abs((x??Infinity)*1000-conveyorCenter.x)<=conveyorSize.x/2&&
      Math.abs((y??Infinity)*1000-conveyorCenter.y)<=conveyorSize.y/2&&
      Math.abs((z??Infinity)*1000-conveyorCenter.z)<=conveyorSize.z/2;
    return [item.id,autoTable&&inside?"table":"shelf"] as const;
  })),[detections,autoTable,conveyorCenter,conveyorSize]);

  const configure=async()=>{
    const parameters:Record<string,unknown>={
      detector_backend:backend,camera_topic:imageTopic,depth_topic:depthTopic,
      camera_info_topic:cameraInfoTopic,base_frame:baseFrame,
      model_family:modelFamily,model_size:modelSize,image_size:imageSize,
      class_prompts:classPrompts,confidence_threshold:confidence,
      boxernet_topic:boxernetTopic,yolo_weights_path:weightsPath,
      hand_eye_transform_yaml:handEyeYaml,hand_eye_parent_frame:handEyeParent,
    };
    await request("vision.configure",{parameters});
  };
  const start=async()=>{
    if(bridgeState!=="connected"){setNotice({kind:"error",message:"ROS bridge is not connected."});return;}
    if(backend==="yolo_seg"&&modelFamily==="custom"&&!weightsPath.trim()){
      setNotice({kind:"error",message:"Choose a custom segmentation checkpoint path, or select a pretrained model family."});return;
    }
    if(backend==="yolo_seg"&&modelFamily==="yoloe-26-seg"&&!classPrompts.trim()){
      setNotice({kind:"error",message:"YOLOE-26 requires at least one class prompt."});return;
    }
    setBusy("start");setNotice(null);
    try{await configure();const result=await request("vision.start");setNotice({kind:"success",message:result.message??"Vision started."});}
    catch(error){setNotice({kind:"error",message:error instanceof Error?error.message:String(error)});}
    finally{setBusy(null);}
  };
  const stop=async()=>{
    setBusy("stop");
    try{const result=await request("vision.stop");setNotice({kind:"info",message:result.message??"Vision stopped."});}
    catch(error){setNotice({kind:"error",message:error instanceof Error?error.message:String(error)});}
    finally{setBusy(null);}
  };
  const detectOnce=async()=>{
    setBusy("once");
    try{const result=await request("vision.detect_once");setNotice({kind:"info",message:result.message??"Waiting for next frame."});}
    catch(error){setNotice({kind:"error",message:error instanceof Error?error.message:String(error)});}
    finally{setBusy(null);}
  };
  const loadHandEye=(event:ChangeEvent<HTMLInputElement>)=>{
    const file=event.currentTarget.files?.[0];
    if(!file)return;
    void file.text().then(text=>{
      if(!text.includes("calibrated:")||!text.includes("transform_parent_to_child:")||!text.includes("frames:")){
        throw new Error("Expected the CR5A hand_eye_transform.yaml calibration format.");
      }
      setHandEyeYaml(text);setHandEyeName(file.name);
      setNotice({kind:"success",message:`Loaded ${file.name}. It will be published as a static parent-to-camera TF when Vision starts.`});
    }).catch(error=>setNotice({kind:"error",message:error instanceof Error?error.message:String(error)}));
    event.currentTarget.value="";
  };
  const addSelectedToScene=async()=>{
    if(!selected.length){setNotice({kind:"info",message:"Select at least one 3D detection first."});return;}
    setBusy("add");setNotice(null);
    const added:string[]=[];
    try{
      for(const item of selected){
        if(!item.valid_3d)continue;
        const result=await request("scene.add",{
          id:`vision_${item.id}`,shape:"box",frame_id:item.frame_id||baseFrame,
          position_m:item.position_m,dimensions_m:item.dimensions_m,
          orientation_xyzw:item.orientation_xyzw,
        });
        added.push(result.id);
      }
      setSceneLoadedIds(current=>Array.from(new Set([...current,...added])));
      setNotice({kind:"success",message:`Added ${added.length} detected box(es) to the MoveIt planning scene.`});
    }catch(error){setNotice({kind:"error",message:`Added ${added.length} before failure: ${error instanceof Error?error.message:String(error)}`});}
    finally{setBusy(null);}
  };
  const sendToPickPlace=(item:VisionDetection)=>{
    if(!item.valid_3d){setNotice({kind:"error",message:"This detection has no calibrated 3D box. Wait for valid depth and TF."});return;}
    const destination=detectionRouting.get(item.id)??"shelf";
    navigate("/pick-place",{state:{visionTransfer:{
      object:item,
      suggestedDestination:destination,
      table:{position_m:[tablePosition.x,tablePosition.y,tablePosition.z].map(value=>value/1000),dimensions_m:[tableSize.x,tableSize.y,tableSize.z].map(value=>value/1000)},
    }}});
  };
  const toggleSelected=(id:string)=>setSelectedIds(current=>current.includes(id)?current.filter(value=>value!==id):[...current,id]);

  return <div className="space-y-5">
    <div className="flex flex-wrap items-start justify-between gap-3"><div><div className="eyebrow">PERCEPTION / RGB-D OBJECT BOXES</div><h1 className="page-title">Vision</h1><p className="mt-1 max-w-3xl text-sm text-zinc-500">Detect objects, lift segmentation masks into metric 3D boxes using aligned depth and calibrated TF, then add them to MoveIt or send one directly to Pick & Place.</p></div><div className="flex items-center gap-2"><span className="rounded-full bg-zinc-100 px-2.5 py-1 text-[10px] font-medium text-zinc-600">{bridgeState}</span><span className={`rounded-full px-2.5 py-1 text-[10px] font-semibold ${status?.active?"bg-emerald-50 text-emerald-700":status?.state==="error"?"bg-red-50 text-red-700":"bg-zinc-100 text-zinc-600"}`}>{status?.active?"DETECTING":status?.state?.toUpperCase()??"NODE WAITING"}</span></div></div>

    {notice&&<div className={`flex items-start gap-2 rounded-lg border px-3 py-2 text-xs ${notice.kind==="error"?"border-red-200 bg-red-50 text-red-800":notice.kind==="success"?"border-emerald-200 bg-emerald-50 text-emerald-800":"border-zinc-200 bg-white text-zinc-600"}`}><AlertTriangle size={14} className="mt-0.5 shrink-0"/><div>{notice.message}</div></div>}
    {(status?.error||statusError||detectionsError)&&<div className="rounded-lg border border-amber-200 bg-amber-50 px-3 py-2 text-xs text-amber-900">{status?.error??statusError??detectionsError}</div>}

    <div className="grid gap-4 xl:grid-cols-[minmax(0,1.4fr)_minmax(340px,.8fr)]">
      <section className="panel overflow-hidden">
        <div className="panel-header"><div className="flex items-center gap-2"><Camera size={16}/><div><div className="text-sm font-semibold">Live camera and segmentation</div><div className="text-[10px] text-zinc-500">Annotated image transport · compressed JPEG</div></div></div><div className="flex gap-2"><Button size="sm" variant="outline" disabled={!status?.active||busy!==null} onClick={detectOnce}><Focus size={13}/>Detect once</Button>{status?.active?<Button size="sm" variant="outline" disabled={busy!==null} onClick={stop}><CircleStop size={13}/>Stop</Button>:<Button size="sm" disabled={busy!==null||bridgeState!=="connected"} onClick={start}><Play size={13}/>{busy==="start"?"Starting…":"Start vision"}</Button>}</div></div>
        <div className="relative flex min-h-[320px] items-center justify-center bg-zinc-950 p-2 sm:min-h-[440px]">
          {previewSrc?<img src={previewSrc} alt="Vision detector annotated camera frame" className="max-h-[620px] w-full rounded object-contain"/>:<div className="max-w-md px-6 text-center text-zinc-400"><Camera size={34} className="mx-auto mb-3 opacity-50"/><div className="text-sm font-medium text-zinc-200">No camera frame received</div><p className="mt-2 text-xs leading-5">Start the RealSense wrist camera and verify the RGB, aligned-depth and CameraInfo topics. A valid optical frame and calibrated TF are required before 3D boxes can be created.</p></div>}
          {previewSrc&&<span className="absolute bottom-4 left-4 rounded bg-black/70 px-2 py-1 font-mono text-[10px] text-white">{preview?.header?.frame_id??status?.camera_frame??"camera frame"}</span>}
        </div>
        <div className="grid grid-cols-2 gap-3 border-t border-zinc-100 px-4 py-3 text-[11px] sm:grid-cols-4"><div><div className="text-zinc-400">Model</div><div className="mt-1 font-semibold">{status?.model_name?status.model_name.split("/").pop():backend==="yolo_seg"?`${modelFamily} ${modelSize.toUpperCase()}`:"BoxerNet"}</div></div><div><div className="text-zinc-400">Frames</div><div className="mt-1 font-mono font-semibold">{status?.frame_count??0}</div></div><div><div className="text-zinc-400">3D boxes</div><div className="mt-1 font-mono font-semibold">{detections.length}</div></div><div><div className="text-zinc-400">CameraInfo</div><div className={`mt-1 font-semibold ${status?.camera_info_ready?"text-emerald-700":"text-amber-700"}`}>{status?.camera_info_ready?"Ready":"Waiting"}</div></div></div>
      </section>

      <section className="panel">
        <div className="panel-header"><div className="flex items-center gap-2"><Settings2 size={16}/><span className="text-sm font-semibold">Detector and camera setup</span></div></div>
        <div className="space-y-3 p-4">
          <label className="block"><span className="mb-1 block text-[10px] font-semibold uppercase tracking-wider text-zinc-500">3D box backend</span><select value={backend} onChange={event=>setBackend(event.currentTarget.value as "yolo_seg"|"boxernet")} className="h-9 w-full rounded-md border border-zinc-200 bg-white px-2 text-xs"><option value="yolo_seg">YOLO segmentation → depth mask 3D box</option><option value="boxernet">BoxerNet → external 3D OBB adapter</option></select></label>
          {backend==="yolo_seg"?<>
            <label className="block"><span className="mb-1 block text-[10px] font-semibold uppercase tracking-wider text-zinc-500">Model family</span><select value={modelFamily} onChange={event=>{setModelFamily(event.currentTarget.value as typeof modelFamily);setWeightsPath("");}} className="h-9 w-full rounded-md border border-zinc-200 bg-white px-2 text-xs"><option value="yolo26-seg">YOLO26-seg · latest Ultralytics instance segmentation</option><option value="yoloe-26-seg">YOLOE-26-seg · open-vocabulary / text prompts</option><option value="yolo11-seg">YOLO11-seg · previous generation comparison</option><option value="yolov8-seg">YOLOv8-seg · legacy comparison</option><option value="custom">Custom segmentation checkpoint</option></select></label>
            {modelFamily!=="custom"&&<>
              <div><div className="mb-2 flex items-center justify-between"><span className="text-[10px] font-semibold uppercase tracking-wider text-zinc-500">Model size</span><span className="font-mono text-[10px] text-zinc-500">{modelSize.toUpperCase()}</span></div><div className="grid grid-cols-5 gap-1.5">{(["n","s","m","l","x"] as const).map((size,index)=>{const names=["Nano","Small","Medium","Large","X-Large"];const details=["Fastest · lowest memory","Balanced","More capacity","Higher latency","Highest capacity"];return <button key={size} type="button" onClick={()=>setModelSize(size)} aria-pressed={modelSize===size} className={`rounded-md border px-1 py-2 text-center ${modelSize===size?"border-zinc-900 bg-zinc-900 text-white":"border-zinc-200 bg-white text-zinc-700 hover:border-zinc-400"}`}><span className="block text-sm font-semibold">{size.toUpperCase()}</span><span className={`mt-0.5 block text-[9px] ${modelSize===size?"text-zinc-300":"text-zinc-400"}`}>{names[index]}</span></button>;})}</div><p className="mt-1.5 text-[10px] leading-4 text-zinc-500">N is the sensible starting point on this laptop. Larger checkpoints increase memory use and inference time. Model weights download automatically the first time they are selected.</p></div>
              <label className="block"><span className="mb-1 block text-[10px] font-semibold uppercase tracking-wider text-zinc-500">Inference resolution</span><select value={imageSize} onChange={event=>setImageSize(Number(event.currentTarget.value))} className="h-9 w-full rounded-md border border-zinc-200 bg-white px-2 text-xs">{[416,512,640,768,960,1024,1280].map(size=><option key={size} value={size}>{size} × {size}{size===640?" · default":size>=960?" · more detail, more compute":""}</option>)}</select><p className="mt-1 text-[10px] text-zinc-500">Higher resolution can help small objects but costs GPU/CPU time and memory.</p></label>
            </>}
            {modelFamily==="yoloe-26-seg"&&<label className="block"><span className="mb-1 block text-[10px] font-semibold uppercase tracking-wider text-zinc-500">Open-vocabulary class prompts</span><textarea value={classPrompts} onChange={event=>setClassPrompts(event.currentTarget.value)} rows={3} className="w-full rounded-md border border-zinc-200 p-2 font-mono text-[11px]"/><p className="mt-1 text-[10px] text-zinc-500">Comma-separated categories. YOLOE-26 can prompt for cans, packaging and box types beyond COCO's fixed class list.</p></label>}
            <div className="rounded-md border border-zinc-200 bg-zinc-50 p-3 text-[10px] leading-4 text-zinc-600"><div className="font-semibold text-zinc-800">Selected checkpoint</div><div className="mt-1 font-mono">{modelFamily==="custom"?(weightsPath||"Custom path required"):modelFamily==="yoloe-26-seg"?`yoloe-26${modelSize}-seg.pt`:`${modelFamily.startsWith("yolov8")?"yolov8":modelFamily.startsWith("yolo11")?"yolo11":"yolo26"}${modelSize}-seg.pt`}</div><div className="mt-1">{modelFamily==="yolo26-seg"?"Recommended latest family. Fixed pretrained COCO class vocabulary.":modelFamily==="yoloe-26-seg"?"Latest open-vocabulary family. Uses your text prompts for category names.":modelFamily==="yolo11-seg"?"Previous-generation reference for speed/accuracy comparison.":modelFamily==="yolov8-seg"?"Legacy comparison checkpoint.":"Load an existing Ultralytics instance-segmentation checkpoint (.pt)."}</div></div>
            <label className="block"><span className="mb-1 block text-[10px] font-semibold uppercase tracking-wider text-zinc-500">{modelFamily==="custom"?"Custom checkpoint path":"Optional local checkpoint override"}</span><input value={weightsPath} onChange={event=>setWeightsPath(event.currentTarget.value)} placeholder={modelFamily==="custom"?"/path/to/best.pt":"Leave empty to download the selected official weights"} className="h-9 w-full rounded-md border border-zinc-200 px-2 font-mono text-[11px]"/></label>
            <NumberField label="Confidence threshold" value={confidence} unit="0–1" min={0.01} max={0.99} step={0.01} onChange={setConfidence}/>
            <p className="text-[10px] leading-4 text-zinc-500">Each segmentation mask is intersected with aligned depth, deprojected using the calibrated CameraInfo intrinsics and transformed into the robot base frame through TF.</p>
          </>:<><label className="block"><span className="mb-1 block text-[10px] font-semibold uppercase tracking-wider text-zinc-500">External BoxerNet 3D output topic</span><input value={boxernetTopic} onChange={event=>setBoxernetTopic(event.currentTarget.value)} className="h-9 w-full rounded-md border border-zinc-200 px-2 font-mono text-xs"/></label><div className="rounded-md border border-amber-200 bg-amber-50 p-3 text-[10px] leading-4 text-amber-900">BoxerNet is an adapter input, not bundled inference. Connect a separately installed BoxerNet runtime that publishes metric 3D boxes to this topic. No BoxerNet code or checkpoint is present in this repository. Confirm the upstream model licence before using its weights.</div></>}
          <div className="grid grid-cols-1 gap-2"><label className="block"><span className="mb-1 block text-[10px] font-semibold uppercase tracking-wider text-zinc-500">RGB image topic</span><input value={imageTopic} onChange={event=>setImageTopic(event.currentTarget.value)} className="h-8 w-full rounded-md border border-zinc-200 px-2 font-mono text-[10px]"/></label><label className="block"><span className="mb-1 block text-[10px] font-semibold uppercase tracking-wider text-zinc-500">Aligned depth topic</span><input value={depthTopic} onChange={event=>setDepthTopic(event.currentTarget.value)} className="h-8 w-full rounded-md border border-zinc-200 px-2 font-mono text-[10px]"/></label><label className="block"><span className="mb-1 block text-[10px] font-semibold uppercase tracking-wider text-zinc-500">CameraInfo topic</span><input value={cameraInfoTopic} onChange={event=>setCameraInfoTopic(event.currentTarget.value)} className="h-8 w-full rounded-md border border-zinc-200 px-2 font-mono text-[10px]"/></label></div>
          <label className="block"><span className="mb-1 block text-[10px] font-semibold uppercase tracking-wider text-zinc-500">MoveIt base frame</span><input value={baseFrame} onChange={event=>setBaseFrame(event.currentTarget.value)} className="h-8 w-full rounded-md border border-zinc-200 px-2 font-mono text-xs"/></label>
          <div className="rounded-lg border border-zinc-200 p-3"><div className="flex items-center justify-between gap-3"><div className="min-w-0"><div className="text-xs font-semibold">Hand-eye calibration</div><div className="mt-0.5 truncate text-[10px] text-zinc-500">{handEyeName||"No calibration file imported · use existing TF if available"}</div></div><label className="flex cursor-pointer items-center gap-1 rounded-md border border-zinc-200 px-2.5 py-1.5 text-[10px] font-medium"><Upload size={12}/>Import YAML<input className="hidden" type="file" accept=".yaml,.yml,text/yaml" onChange={loadHandEye}/></label></div><div className="mt-3"><label className="mb-1 block text-[10px] font-semibold uppercase tracking-wider text-zinc-500">Tool / calibration parent frame</label><input value={handEyeParent} onChange={event=>setHandEyeParent(event.currentTarget.value)} className="h-8 w-full rounded-md border border-zinc-200 px-2 font-mono text-xs"/></div><p className="mt-2 text-[10px] leading-4 text-zinc-500">Imports the calibrated parent→camera transform (for example Link6 → D415 optical frame) and publishes it as static TF when no camera transform already exists. The live base→Link6 transform then locates detections in {baseFrame}.</p><div className="mt-2 flex items-center gap-2 text-[10px]"><span className={`h-1.5 w-1.5 rounded-full ${status?.calibration_loaded?"bg-emerald-500":status?.camera_frame?"bg-amber-500":"bg-zinc-300"}`}/>{status?.calibration_loaded?`Calibration TF: ${status.hand_eye_frames?.join(" → ")}`:status?.camera_frame?`Camera frame ${status.camera_frame}; waiting for base transform`:"No camera TF confirmed"}</div></div>
          <div className="flex flex-wrap justify-between gap-2"><Button size="sm" variant="outline" disabled={busy!==null} onClick={stop}><CircleStop size={13}/>Stop</Button><Button size="sm" disabled={busy!==null||bridgeState!=="connected"} onClick={start}><Play size={13}/>{busy==="start"?"Starting…":"Apply settings & start"}</Button></div>
        </div>
      </section>
    </div>

    <section className="panel overflow-hidden">
      <div className="panel-header"><div><div className="flex items-center gap-2"><Boxes size={16}/><span className="text-sm font-semibold">Detected 3D boxes</span></div><p className="mt-0.5 text-[10px] text-zinc-500">Metric oriented bounding boxes in the MoveIt base frame · {detectionsEnvelope.frame_id??status?.base_frame??baseFrame}</p></div><div className="flex flex-wrap gap-2"><Button size="sm" variant="outline" disabled={!selected.length||busy!==null} onClick={addSelectedToScene}><Layers3 size={13}/>{busy==="add"?"Adding…":"Add selected to MoveIt"}</Button><span className="self-center text-[10px] text-zinc-400">{selected.length} selected</span></div></div>
      {detections.length===0?<div className="px-5 py-12 text-center"><ScanEye size={28} className="mx-auto mb-2 text-zinc-300"/><div className="text-xs font-medium text-zinc-700">No 3D detections yet</div><div className="mt-1 text-[10px] text-zinc-400">Start the detector and confirm RGB, aligned depth, CameraInfo and TF are available.</div></div>:<div className="divide-y divide-zinc-100">{detections.map(item=>{
        const route=detectionRouting.get(item.id)??"shelf";
        const checked=selectedIds.includes(item.id);
        return <div key={item.id} className="grid gap-3 px-4 py-3 md:grid-cols-[minmax(0,1fr)_auto] md:items-center"><div className="flex min-w-0 items-start gap-3"><input aria-label={`Select ${item.class_name}`} type="checkbox" checked={checked} onChange={()=>toggleSelected(item.id)} className="mt-1"/><div className="min-w-0 flex-1"><div className="flex flex-wrap items-center gap-2"><span className="text-xs font-semibold text-zinc-800">{item.class_name}</span><span className="rounded bg-zinc-100 px-1.5 py-0.5 font-mono text-[9px] text-zinc-500">{(item.confidence*100).toFixed(0)}%</span>{!item.valid_3d&&<span className="text-[9px] text-amber-700">2D only</span>}{route==="table"&&<span className="rounded bg-blue-50 px-1.5 py-0.5 text-[9px] font-semibold text-blue-700">Conveyor zone → table</span>}</div><div className="mt-1 font-mono text-[10px] text-zinc-500">C: {item.position_m.map(value=>value.toFixed(3)).join(", ")} m · Size: {item.dimensions_m.map(value=>value.toFixed(3)).join(" × ")} m</div><div className="mt-0.5 text-[9px] text-zinc-400">{item.id} · {item.frame_id} · {item.depth_points?`${item.depth_points} depth points`:item.source_backend}</div></div></div><div className="flex flex-wrap gap-2"><Button size="sm" variant="outline" disabled={!item.valid_3d} onClick={()=>sendToPickPlace(item)}><Send size={12}/>Send to Pick & Place</Button></div></div>;
      })}</div>}
      {sceneLoadedIds.length>0&&<div className="border-t border-zinc-100 px-4 py-2 text-[10px] text-emerald-700">Loaded in MoveIt: {sceneLoadedIds.join(", ")}</div>}
    </section>

    <section className="panel">
      <div className="panel-header"><div><div className="text-sm font-semibold">Conveyor → table routing</div><p className="mt-0.5 text-[10px] text-zinc-500">If a detected centroid falls inside the configured conveyor zone, recommend the table as its place destination.</p></div><label className="flex items-center gap-2 text-[10px] font-medium"><input type="checkbox" checked={autoTable} onChange={event=>setAutoTable(event.currentTarget.checked)}/>Enable auto-route</label></div>
      <div className="grid gap-4 p-4 lg:grid-cols-2"><div><div className="mb-2 text-xs font-semibold">Conveyor detection zone · base-frame AABB</div><div className="grid grid-cols-3 gap-2">{(["x","y","z"] as const).map(axis=><NumberField key={axis} label={`Center ${axis.toUpperCase()}`} value={conveyorCenter[axis]} unit="mm" onChange={value=>setConveyorCenter(current=>({...current,[axis]:value}))}/>)}</div><div className="mt-2 grid grid-cols-3 gap-2">{(["x","y","z"] as const).map(axis=><NumberField key={axis} label={`Size ${axis.toUpperCase()}`} value={conveyorSize[axis]} unit="mm" min={10} onChange={value=>setConveyorSize(current=>({...current,[axis]:value}))}/>)}</div></div><div><div className="mb-2 text-xs font-semibold">Table placement target</div><div className="grid grid-cols-3 gap-2">{(["x","y","z"] as const).map(axis=><NumberField key={axis} label={`Position ${axis.toUpperCase()}`} value={tablePosition[axis]} unit="mm" onChange={value=>setTablePosition(current=>({...current,[axis]:value}))}/>)}</div><div className="mt-2 grid grid-cols-3 gap-2">{(["x","y","z"] as const).map(axis=><NumberField key={axis} label={`Table ${axis.toUpperCase()}`} value={tableSize[axis]} unit="mm" min={5} onChange={value=>setTableSize(current=>({...current,[axis]:value}))}/>)}</div><p className="mt-2 text-[10px] leading-4 text-zinc-500">Routing is a recommendation passed into the Pick & Place page. The actual target still needs to pass shelf/table collision and reachability checks.</p></div></div>
    </section>
  </div>;
}
