import {useEffect,useMemo,useState} from "react";
import {Box,CheckCircle2,CircleAlert,Crosshair,Gauge,Move3D,Play,RefreshCw,Route,ShieldCheck} from "lucide-react";
import {Button} from "@/components/ui/button";
import {Tabs,TabsContent,TabsList,TabsTrigger} from "@/components/ui/tabs";
import {RobotViewport} from "@/components/robot/robot-viewport";
import {SceneManager,collisionObjectToPayload} from "@/components/robot/scene-manager";
import type {CollisionObject} from "@/lib/types";
import {StatusBadge} from "@/components/status-badge";
import {PlannerSettings} from "@/components/planner-settings";
import {DEFAULT_PLANNER_PROFILE,plannerRequestOptions,type PlannerProfile} from "@/lib/planner-profiles";
import {formatNumber,parseJointBounds,rpyDegreesToQuaternion} from "@/lib/utils";
import {extractRobotModeCode,hasActiveError,operationStatus} from "@/lib/status";
import {useRobotSocket} from "@/lib/socket";
import {useRobotStore} from "@/store/robot-store";
import {GripperPositionCard} from "@/components/gripper/gripper-panels";
import {useGripperController} from "@/components/gripper/use-gripper-controller";

type TcpTarget={x:number;y:number;z:number;rx:number;ry:number;rz:number};
type DirectMotionStatus={phase:string;detail?:string|null;last_command?:string|null};

const DIRECT_JOINT_NAMES=["joint1","joint2","joint3","joint4","joint5","joint6"] as const;

function readNumberTuple(params:URLSearchParams,key:string,length:number):number[]|null{
  const raw=params.get(key);
  if(!raw)return null;
  const parts=raw.split(",").map((value)=>value.trim());
  if(parts.length!==length||parts.some((value)=>value===""))return null;
  const values=parts.map(Number);
  return values.every(Number.isFinite)?values:null;
}

function planErrorDetail(code?:number|null):string|null{
  if(code===undefined||code===null)return null;
  const reasons:Record<number,string>={
    [-1]:"Planning failed: no valid trajectory was found.",
    [-2]:"Invalid motion plan returned by the planner.",
    [-3]:"Plan invalidated by the current environment or robot state.",
    [-10]:"Start state is invalid (check current joints/collisions).",
    [-11]:"Start state is in collision.",
    [-12]:"Start state violates path constraints.",
    [-13]:"Goal state is invalid.",
    [-14]:"Goal state is in collision.",
    [-15]:"Goal violates path constraints.",
    [-16]:"Goal constraints are invalid.",
    [-17]:"Robot state is invalid.",
    [-18]:"Planner configuration is invalid.",
    [-21]:"Required transform is unavailable.",
    [-22]:"A required transform lookup failed.",
    [-23]:"A required transform is stale.",
    [-24]:"A required transform extrapolation failed.",
    [99999]:"MoveIt reported a generic failure."
  };
  return `MoveIt code ${code}: ${reasons[code]??"Unclassified planning result; inspect MoveIt logs."}`;
}


function NumericInput({label,value,suffix,onChange}:{label:string;value:number;suffix:string;onChange:(value:number)=>void}){
  return <label className="block">
    <span className="mb-1.5 block text-[11px] font-medium uppercase tracking-wider text-zinc-500">{label}</span>
    <div className="flex h-9 items-center rounded-md border border-zinc-200 bg-white focus-within:border-zinc-500">
      <input type="number" step="0.1" value={Number.isFinite(value)?Number(value.toFixed(3)):0}
        onChange={(event)=>onChange(Number(event.target.value))}
        className="min-w-0 flex-1 bg-transparent px-2.5 font-mono text-sm outline-none"/>
      <span className="px-2 text-[10px] text-zinc-400">{suffix}</span>
    </div>
  </label>;
}

export function VisualizerPage(){
  const gripper=useGripperController();
  const {request}=useRobotSocket();
  const bridgeState=useRobotStore((s)=>s.bridgeState);
  const robotStatus=useRobotStore((s)=>s.robotStatus);
  const robotMode=useRobotStore((s)=>s.robotMode);
  const errors=useRobotStore((s)=>s.errors);
  const joints=useRobotStore((s)=>s.joints);
  const tcp=useRobotStore((s)=>s.tcp);
  const jointLimits=useRobotStore((s)=>s.jointLimits);
  const model=useRobotStore((s)=>s.model);
  const modelLoading=useRobotStore((s)=>s.modelLoading);
  const modelError=useRobotStore((s)=>s.modelError);
  const scene=useRobotStore((s)=>s.scene);
  const sceneError=useRobotStore((s)=>s.sceneError);
  const capabilities=useRobotStore((s)=>s.capabilities);
  const plan=useRobotStore((s)=>s.plan);
  const executionStatus=useRobotStore((s)=>s.executionStatus);
  const setPlan=useRobotStore((s)=>s.setPlan);
  const setExecutionStatus=useRobotStore((s)=>s.setExecutionStatus);
  const setNotice=useRobotStore((s)=>s.setNotice);
  const notice=useRobotStore((s)=>s.notice);

  const initialParams=new URLSearchParams(window.location.search);
  const initialTcp=readNumberTuple(initialParams,"tcp",6);
  const initialJointDegrees=readNumberTuple(initialParams,"joints",6);
  const initialMovJ=readNumberTuple(initialParams,"movj",6);
  const initialMovL=readNumberTuple(initialParams,"movl",6);
  const [tab,setTab]=useState(()=>initialParams.get("tab")==="joints"?"joints":"tcp");
  const [tcpTarget,setTcpTarget]=useState<TcpTarget|null>(()=>initialTcp?{
    x:initialTcp[0],y:initialTcp[1],z:initialTcp[2],rx:initialTcp[3],ry:initialTcp[4],rz:initialTcp[5]
  }:null);
  const [tcpDirty,setTcpDirty]=useState(()=>initialTcp!==null);
  const [jointTargets,setJointTargets]=useState<Record<string,number>>(()=>initialJointDegrees
    ?Object.fromEntries(DIRECT_JOINT_NAMES.map((name,index)=>[name,initialJointDegrees[index]*Math.PI/180])):{});
  const [jointDirty,setJointDirty]=useState(()=>initialJointDegrees!==null);
  const [plannedJointSnapshot,setPlannedJointSnapshot]=useState<Record<string,number>|null>(null);
  const [busy,setBusy]=useState<string|null>(null);
  const [showObjectLabels,setShowObjectLabels]=useState(true);
  const [objectLabels,setObjectLabels]=useState<Record<string,boolean>>({});
  const [hiddenObjects,setHiddenObjects]=useState<Record<string,boolean>>({});
  const [disabledObjects,setDisabledObjects]=useState<Record<string,CollisionObject>>(()=>{
    try {
      const stored=localStorage.getItem("cr5a-visualizer-disabled-objects-v1");
      const parsed=stored?JSON.parse(stored):{};
      return parsed&&typeof parsed==="object"&&!Array.isArray(parsed)?parsed:{};
    } catch { return {}; }
  });
  const [collisionBusy,setCollisionBusy]=useState<string|null>(null);
  const toggleObjectLabel=(id:string)=>setObjectLabels((current)=>({...current,[id]:!(current[id]??true)}));
  const toggleObjectVisibility=(id:string)=>setHiddenObjects((current)=>({...current,[id]:!(current[id]??false)}));
  const removeDisabledObject=(id:string)=>setDisabledObjects((current)=>{
    const next={...current};delete next[id];return next;
  });
  const clearDisabledObjects=()=>setDisabledObjects({});
  useEffect(()=>{
    localStorage.setItem("cr5a-visualizer-disabled-objects-v1",JSON.stringify(disabledObjects));
  },[disabledObjects]);

  const toggleObjectCollision=async(object:CollisionObject)=>{
    if(!model||collisionBusy)return;
    const activeInMoveIt=worldObjects.some((item)=>item.id===object.id);
    setCollisionBusy(object.id);
    setNotice(null);
    try {
      if(activeInMoveIt){
        await request("scene.remove",{id:object.id});
        setDisabledObjects((current)=>({...current,[object.id]:object}));
      } else {
        const payload=collisionObjectToPayload(object,model.base_frame);
        if(!payload)throw new Error("Only single box, sphere, and cylinder objects can be toggled here.");
        await request("scene.add",payload as unknown as Record<string,unknown>);
        setDisabledObjects((current)=>{
          const next={...current};
          delete next[object.id];
          return next;
        });
      }
      const latest=await request("moveit.get_scene");
      useRobotStore.getState().setScene(latest);
      setPlan(null);
      setExecutionStatus(null);
      setPlannedJointSnapshot(null);
      setNotice({
        tone:"success",
        message:activeInMoveIt
          ? `Collision checking disabled for "${object.id}". The object remains visible in the viewer.`
          : `Collision checking enabled for "${object.id}" in MoveIt.`
      });
    } catch(error) {
      setNotice({tone:"error",message:error instanceof Error?error.message:String(error)});
    } finally {
      setCollisionBusy(null);
    }
  };
  const [plannerProfile,setPlannerProfile]=useState<PlannerProfile>(DEFAULT_PLANNER_PROFILE);
  const directJointNames=["joint1","joint2","joint3","joint4","joint5","joint6"];
  const [directType,setDirectType]=useState<"movj"|"movl">(()=>initialParams.get("direct")==="movl"?"movl":"movj");
  const [directJointTarget,setDirectJointTarget]=useState<number[]>(()=>initialMovJ??[0,0,0,0,0,0]);
  const [directJointDirty,setDirectJointDirty]=useState(()=>initialMovJ!==null);
  const [directPoseTarget,setDirectPoseTarget]=useState<TcpTarget>(()=>initialMovL?{
    x:initialMovL[0],y:initialMovL[1],z:initialMovL[2],rx:initialMovL[3],ry:initialMovL[4],rz:initialMovL[5]
  }:{x:0,y:0,z:0,rx:0,ry:0,rz:0});
  const [directPoseDirty,setDirectPoseDirty]=useState(()=>initialMovL!==null);
  const [directCheck,setDirectCheck]=useState<any>(null);
  const [directMotionStatus,setDirectMotionStatus]=useState<DirectMotionStatus|null>(null);

  const bounds=useMemo(
    ()=>parseJointBounds(model?.urdf,jointLimits?.joint_limits),
    [model?.urdf,jointLimits]
  );
  const jointNames=joints?.name??[];
  const actualJoints=joints?.position??[];
  const actualJointsValid=jointNames.length>0&&jointNames.every((_,index)=>
    typeof actualJoints[index]==="number"&&Number.isFinite(actualJoints[index])
  );
  const worldObjects=scene?.world?.collision_objects??[];
  const attachedObjects=scene?.robot_state?.attached_collision_objects??[];

  useEffect(()=>{
    const params=new URLSearchParams(window.location.search);
    params.set("tab",tab);
    params.set("direct",directType);
    if(tcpDirty&&tcpTarget)params.set("tcp",[tcpTarget.x,tcpTarget.y,tcpTarget.z,tcpTarget.rx,tcpTarget.ry,tcpTarget.rz].join(","));
    else params.delete("tcp");
    if(jointDirty&&DIRECT_JOINT_NAMES.every((name)=>Number.isFinite(jointTargets[name])))params.set("joints",DIRECT_JOINT_NAMES.map((name)=>String(jointTargets[name]*180/Math.PI)).join(","));
    else params.delete("joints");
    if(directJointDirty)params.set("movj",directJointTarget.join(","));
    else params.delete("movj");
    if(directPoseDirty)params.set("movl",[directPoseTarget.x,directPoseTarget.y,directPoseTarget.z,directPoseTarget.rx,directPoseTarget.ry,directPoseTarget.rz].join(","));
    else params.delete("movl");
    const query=params.toString();
    const nextUrl=window.location.pathname+(query?"?"+query:"")+window.location.hash;
    window.history.replaceState(null,"",nextUrl);
  },[tab,directType,tcpTarget,tcpDirty,jointTargets,jointDirty,directJointTarget,directJointDirty,directPoseTarget,directPoseDirty]);

  useEffect(()=>{
    if(bridgeState!=="connected")return;
    let active=true;
    void request("robot.direct.status").then((value)=>{
      if(active)setDirectMotionStatus(value);
    }).catch(()=>undefined);
    return ()=>{active=false;};
  },[bridgeState,request]);

  useEffect(()=>{
    if(tcpDirty)return;
    if(!tcp){
      setTcpTarget(null);
      return;
    }
    const values=[tcp.x,tcp.y,tcp.z,tcp.rx,tcp.ry,tcp.rz];
    if(!values.every((value)=>typeof value==="number"&&Number.isFinite(value))){
      setTcpTarget(null);
      return;
    }
    setTcpTarget({
      x:tcp.x as number,y:tcp.y as number,z:tcp.z as number,
      rx:tcp.rx as number,ry:tcp.ry as number,rz:tcp.rz as number
    });
  },[tcp,tcpDirty]);

  useEffect(()=>{
    if(jointDirty)return;
    const next:Record<string,number>={};
    jointNames.forEach((name,index)=>{
      const value=actualJoints[index];
      if(typeof value==="number"&&Number.isFinite(value))next[name]=value;
    });
    setJointTargets(next);
  },[jointNames.join("|"),actualJoints.join("|"),jointDirty]);

  useEffect(()=>{
    if(directJointDirty)return;
    const byName:Record<string,number>={};
    jointNames.forEach((name,index)=>{
      const value=actualJoints[index];
      if(typeof value==="number"&&Number.isFinite(value))byName[name]=value*180/Math.PI;
    });
    if(directJointNames.every((name)=>Number.isFinite(byName[name]))) {
      setDirectJointTarget(directJointNames.map((name)=>byName[name]));
    }
  },[jointNames.join("|"),actualJoints.join("|"),directJointDirty]);

  useEffect(()=>{
    if(plan?.status!=="SUCCEEDED"||!plannedJointSnapshot||executionStatus==="EXECUTING")return;
    const current:Record<string,number>={};
    jointNames.forEach((name,index)=>{
      const value=actualJoints[index];
      if(typeof value==="number"&&Number.isFinite(value))current[name]=value;
    });
    const names=Object.keys(plannedJointSnapshot);
    const changed=names.length===0||names.some((name)=>
      current[name]===undefined||Math.abs(current[name]-plannedJointSnapshot[name])>0.03
    );
    if(changed){
      setPlan(null);
      setExecutionStatus(null);
      setPlannedJointSnapshot(null);
      setNotice({tone:"neutral",message:"Robot joints changed after planning. The old plan was cleared; plan again."});
    }
  },[actualJoints.join("|"),jointNames.join("|"),plan?.status,plannedJointSnapshot,executionStatus]);

  const canExecute=capabilities.motion_execution_enabled===true&&bridgeState==="connected"&&
    robotStatus?.is_connected===true&&robotStatus?.is_enable===true;
  const hasPlan=plan?.status==="SUCCEEDED"&&Boolean(plan.plan_id);

  const modeCode=extractRobotModeCode(robotMode?.value);
  const telemetryFresh=(timestamp?:string)=> {
    if(!timestamp)return false;
    const age=Date.now()-Date.parse(timestamp);
    return Number.isFinite(age)&&age>=-1000&&age<=1500;
  };
  const timestamps=useRobotStore.getState().timestamps;
  const directReady=bridgeState==="connected"&&robotStatus?.is_connected===true&&
    robotStatus?.is_enable===true&&modeCode===5&&!hasActiveError(errors)&&
    actualJointsValid&&telemetryFresh(timestamps.joints)&&telemetryFresh(timestamps.status);
  const directExecutionEnabled=capabilities.direct_movement_enabled===true;
  const movlFrameConfirmed=capabilities.movl_frame_confirmed===true;
  const directMotionBlocked=directMotionStatus!==null&&directMotionStatus.phase!=="idle";

  const resetTargets=()=>{
    setPlan(null);setExecutionStatus(null);setPlannedJointSnapshot(null);
    setTcpDirty(false);setJointDirty(false);setDirectJointDirty(false);setDirectPoseDirty(false);setDirectCheck(null);
    if(tcp&&[tcp.x,tcp.y,tcp.z,tcp.rx,tcp.ry,tcp.rz].every((value)=>typeof value==="number"&&Number.isFinite(value))){
      setTcpTarget({
        x:tcp.x as number,y:tcp.y as number,z:tcp.z as number,
        rx:tcp.rx as number,ry:tcp.ry as number,rz:tcp.rz as number
      });
    }else setTcpTarget(null);
    const next:Record<string,number>={};
    jointNames.forEach((name,index)=>{
      const value=actualJoints[index];
      if(typeof value==="number"&&Number.isFinite(value))next[name]=value;
    });
    setJointTargets(next);
  };

  const planTcp=async()=>{
    if(!tcpTarget||!model)return;
    if(!actualJointsValid||![tcpTarget.x,tcpTarget.y,tcpTarget.z,tcpTarget.rx,tcpTarget.ry,tcpTarget.rz].every(Number.isFinite)){
      setNotice({tone:"error",message:"Cannot plan without finite TCP target values and complete joint telemetry."});
      return;
    }
    const startJoints:Record<string,number>={};
    jointNames.forEach((name,index)=>{startJoints[name]=actualJoints[index] as number;});
    setBusy("plan");setNotice(null);setPlan({status:"PLANNING"});setExecutionStatus(null);setPlannedJointSnapshot(null);
    try{
      const quaternion=rpyDegreesToQuaternion(tcpTarget.rx,tcpTarget.ry,tcpTarget.rz);
      const result=await request("motion.plan",{
        group_name:model.group_name,
        target:{
          type:"pose",frame_id:model.base_frame,link_name:model.end_effector_link,
          position_m:[tcpTarget.x/1000,tcpTarget.y/1000,tcpTarget.z/1000],
          orientation_xyzw:quaternion,position_tolerance_m:0.002,orientation_tolerance_rad:0.02
        },
        ...plannerRequestOptions(plannerProfile)
      });
      setPlan(result);
      setPlannedJointSnapshot(result.status==="SUCCEEDED"?startJoints:null);
      setNotice({
        tone:result.status==="SUCCEEDED"?"success":"error",
        message:result.status==="SUCCEEDED"?"Plan generated. Inspect the path before execution.":(planErrorDetail(result.error_code)??"MoveIt could not find a valid plan.")
      });
    }catch(error){
      setPlan({status:"FAILED"});
      setNotice({tone:"error",message:error instanceof Error?error.message:String(error)});
    }finally{setBusy(null);}
  };

  const planJoints=async()=>{
    if(!model||!jointNames.length)return;
    if(!actualJointsValid){
      setNotice({tone:"error",message:"Cannot plan joint targets without complete finite joint telemetry."});
      return;
    }
    if(jointNames.some((name)=>typeof jointTargets[name]!=="number"||!Number.isFinite(jointTargets[name]))){
      setNotice({tone:"error",message:"Every joint target must contain a finite value."});
      return;
    }
    const startJoints:Record<string,number>={};
    jointNames.forEach((name,index)=>{startJoints[name]=actualJoints[index] as number;});
    setBusy("plan");setNotice(null);setPlan({status:"PLANNING"});setExecutionStatus(null);setPlannedJointSnapshot(null);
    try{
      const result=await request("motion.plan",{
        group_name:model.group_name,
        target:{
          type:"joints",joint_names:jointNames,
          positions_rad:jointNames.map((name)=>jointTargets[name]??0),tolerance_rad:0.01
        },
        ...plannerRequestOptions(plannerProfile)
      });
      setPlan(result);
      setPlannedJointSnapshot(result.status==="SUCCEEDED"?startJoints:null);
      setNotice({
        tone:result.status==="SUCCEEDED"?"success":"error",
        message:result.status==="SUCCEEDED"?"Joint-space plan generated.":"MoveIt could not find a valid plan."
      });
    }catch(error){
      setPlan({status:"FAILED"});
      setNotice({tone:"error",message:error instanceof Error?error.message:String(error)});
    }finally{setBusy(null);}
  };

  const directPayload=()=>{
    if(directType==="movj")return {type:"movj",target_joints_deg:directJointTarget};
    return {
      type:"movl",
      target_pose_mm_deg:[
        directPoseTarget.x,directPoseTarget.y,directPoseTarget.z,
        directPoseTarget.rx,directPoseTarget.ry,directPoseTarget.rz
      ]
    };
  };

  const checkDirectPath=async()=>{
    if(!directReady)return;
    setBusy("direct-check");setDirectCheck(null);setNotice(null);
    try{
      const result=await request("robot.direct.check",directPayload());
      setDirectCheck(result);
      setNotice({
        tone:result.collision_free?"success":"error",
        message:result.collision_free
          ?("MoveIt checked "+result.samples_checked+" sampled states. Review the limitations before moving.")
          :(result.reason??"The requested path did not pass collision validation.")
      });
    }catch(error){
      setDirectCheck(null);
      setNotice({tone:"error",message:error instanceof Error?error.message:String(error)});
    }finally{setBusy(null);}
  };

  const refreshDirectMotionStatus=async()=>{
    try{
      const status=await request("robot.direct.status");
      setDirectMotionStatus(status);
      return status;
    }catch{
      return null;
    }
  };

  const recoverDirectMovement=async()=>{
    if(!directReady||directMotionStatus?.phase!=="recovery_required")return;
    setBusy("direct-recover");setNotice(null);
    try{
      const result=await request("robot.direct.recover",{confirmed_idle:true});
      setDirectMotionStatus(result.direct_motion??{phase:result.recovered?"idle":"recovery_required"});
      setNotice({
        tone:result.recovered?"success":"error",
        message:result.recovered
          ?"Direct movement lock cleared after controller idle and MoveIt state were verified."
          :"Direct movement remains locked. Verify the robot and controller state."
      });
    }catch(error){
      await refreshDirectMotionStatus();
      setNotice({tone:"error",message:error instanceof Error?error.message:String(error)});
    }finally{setBusy(null);}
  };

  const executeDirectMove=async()=>{
    if(!directCheck?.collision_free||!directReady||!directExecutionEnabled||directMotionBlocked)return;
    if(directType==="movl"&&!movlFrameConfirmed)return;
    setBusy("direct-execute");setNotice(null);
    try{
      const result=await request("robot.direct.execute",directPayload());
      setDirectCheck(null);
      await refreshDirectMotionStatus();
      const response=result.robot_response??{};
      const succeeded=response.res===0;
      setNotice({
        tone:succeeded?"success":"error",
        message:succeeded
          ?(directType.toUpperCase()+" completed. The bridge confirmed controller idle and stable joint telemetry.")
          :("Dobot controller returned res="+String(response.res)+": "+String(response.robot_return??"No response detail"))
      });
    }catch(error){
      await refreshDirectMotionStatus();
      setDirectCheck(null);
      setNotice({tone:"error",message:error instanceof Error?error.message:String(error)});
    }finally{setBusy(null);}
  };

  const execute=async()=>{
    if(!plan?.plan_id)return;
    setBusy("execute");setExecutionStatus("EXECUTING");
    try{
      const result=await request("motion.execute",{plan_id:plan.plan_id});
      setExecutionStatus(result.status);
      setNotice({
        tone:result.status==="SUCCEEDED"?"success":"error",
        message:result.status==="SUCCEEDED"?"Trajectory execution completed.":"Trajectory execution failed."
      });
    }catch(error){
      setExecutionStatus("FAILED");
      setNotice({tone:"error",message:error instanceof Error?error.message:String(error)});
    }finally{setBusy(null);}
  };

  return <div className="space-y-5">
    <div className="flex flex-wrap items-end justify-between gap-3">
      <div>
        <div className="eyebrow">MOTION / PLANNING SCENE</div>
        <h1 className="page-title">Scene visualizer</h1>
        <p className="mt-1 text-sm text-zinc-500">CR5 model, MoveIt collision scene and trajectory planning.</p>
      </div>
      <div className="flex gap-2">
        <Button variant="outline" size="sm" onClick={()=>{
          void navigator.clipboard.writeText(window.location.href).then(()=>{
            setNotice({tone:"success",message:"Shareable visualizer URL copied."});
          }).catch(()=>setNotice({tone:"error",message:"Clipboard access was blocked by the browser."}));
        }}>Copy URL</Button>
        <Button variant="outline" size="sm" onClick={resetTargets}><RefreshCw size={13}/> Use actual state</Button>
      </div>
    </div>

    {notice&&<div className={notice.tone==="error"?"notice notice-error":notice.tone==="success"?"notice notice-success":"notice"}>{notice.message}</div>}

    <div className="grid items-start gap-5 xl:grid-cols-[minmax(0,1fr)_360px]">
      <section className="panel overflow-hidden">
        <div className="panel-header">
          <div className="flex items-center gap-2">
            <Box size={15} className="text-zinc-500"/>
            <span className="text-sm font-semibold">Planning scene</span>
            <span className="text-xs text-zinc-400">{worldObjects.length} world · {attachedObjects.length} attached</span>
          </div>
          <span className="text-[11px] text-zinc-400">{model?.base_frame??"dummy_link"}</span>
        </div>
        <RobotViewport model={model} jointState={joints} scene={scene} plan={plan}
          gripperPositionMm={gripper.positionMm}
          loading={modelLoading} error={modelError??sceneError}
          showLabels={showObjectLabels} onToggleLabels={()=>setShowObjectLabels((value)=>!value)}
          objectLabels={objectLabels} disabledObjects={disabledObjects} hiddenObjects={hiddenObjects}/>
        <div className="flex flex-wrap items-center justify-between gap-2 border-t border-zinc-100 px-4 py-3 text-[11px] text-zinc-500">
          <span>Blue line: planned end-effector path</span>
          <span className="flex flex-wrap items-center gap-3">
            <span className="inline-flex items-center gap-1"><i className="h-2 w-2 rounded-full bg-slate-500"/> Collision enabled</span>
            <span className="inline-flex items-center gap-1"><i className="h-2 w-2 rounded-full bg-amber-500"/> Collision disabled</span>
          </span>
          <span>{plan?.status?"Plan: "+plan.status:"No planned trajectory"}</span>
        </div>
      </section>

      <aside className="space-y-4">
        <PlannerSettings onProfileChange={setPlannerProfile} disabled={busy!==null}/>
        <section className="panel">
          <div className="panel-header">
            <div>
              <div className="text-sm font-semibold">Direct movement</div>
              <div className="mt-0.5 text-xs text-zinc-500">Dobot MovJ / MovL · separate from planner profiles</div>
            </div>
            <ShieldCheck size={16} className="text-zinc-400"/>
          </div>
          <div className="space-y-3 px-4 py-4">
            <Tabs value={directType} onValueChange={(value)=>{
              setDirectType(value as "movj"|"movl");setDirectCheck(null);setNotice(null);
            }}>
              <TabsList className="grid w-full grid-cols-2">
                <TabsTrigger value="movj">MovJ · joints</TabsTrigger>
                <TabsTrigger value="movl">MovL · Cartesian</TabsTrigger>
              </TabsList>
              <TabsContent value="movj">
                <div className="grid grid-cols-3 gap-3">
                  {directJointNames.map((name,index)=><NumericInput key={name}
                    label={name.toUpperCase()} value={directJointTarget[index]??0} suffix="deg"
                    onChange={(value)=>{
                      setDirectJointDirty(true);setDirectCheck(null);
                      setDirectJointTarget((current)=>current.map((item,i)=>i===index?value:item));
                    }}/>)}
                </div>
                <p className="mt-2 text-[11px] leading-4 text-zinc-500">
                  Target joint angles in degrees. MoveIt checks interpolated joint states from the current robot position to this target.
                </p>
              </TabsContent>
              <TabsContent value="movl">
                <div className="grid grid-cols-3 gap-3">
                  {(["x","y","z"] as const).map((key)=><NumericInput key={key}
                    label={key.toUpperCase()} value={directPoseTarget[key]} suffix="mm"
                    onChange={(value)=>{setDirectPoseDirty(true);setDirectCheck(null);setDirectPoseTarget((current)=>({...current,[key]:value}));}}/>)}
                  {(["rx","ry","rz"] as const).map((key)=><NumericInput key={key}
                    label={key.toUpperCase()} value={directPoseTarget[key]} suffix="deg"
                    onChange={(value)=>{setDirectCheck(null);setDirectPoseTarget((current)=>({...current,[key]:value}));}}/>)}
                </div>
                <p className="mt-2 text-[11px] leading-4 text-zinc-500">
                  Straight Cartesian interpolation in {model?.base_frame??"the MoveIt base frame"} for {model?.end_effector_link??"the end-effector link"}. Position is mm; RPY is degrees.
                </p>
                {!movlFrameConfirmed&&<p className="mt-2 rounded-md border border-amber-200 bg-amber-50 p-2 text-[11px] leading-4 text-amber-800">
                  MovL is locked until Dobot controller user/tool coordinates are verified to match the MoveIt base frame and end-effector. Configure controller_pose_frame_confirmed in bridge.yaml only after checking the actual transform and TCP.
                </p>}
              </TabsContent>
            </Tabs>
            <div className="flex flex-wrap gap-2">
              <Button className="flex-1" variant="outline" onClick={checkDirectPath}
                disabled={!directReady||directMotionBlocked||busy!==null||(directType==="movl"&&!movlFrameConfirmed)}>
                {busy==="direct-check"?<RefreshCw size={14} className="animate-spin"/>:<ShieldCheck size={14}/>} Check path
              </Button>
              <Button className="flex-1" onClick={executeDirectMove}
                disabled={!directCheck?.collision_free||!directReady||!directExecutionEnabled||directMotionBlocked||busy!==null||(directType==="movl"&&!movlFrameConfirmed)}>
                {busy==="direct-execute"?<RefreshCw size={14} className="animate-spin"/>:<Play size={14}/>} Move
              </Button>
            </div>
            {!directReady&&<p className="text-[11px] text-zinc-400">Checking and moving require a connected, enabled, idle robot, fresh joint telemetry, and no active controller errors.</p>}
            {!directExecutionEnabled&&<p className="text-[11px] text-zinc-400">Move requires both bridge robot.allow_commands and robot.allow_direct_movement safety gates. Path checking does not enable movement.</p>}
            {directMotionStatus&&directMotionStatus.phase!=="idle"&&<div className="rounded-md border border-amber-200 bg-amber-50 p-3 text-xs text-amber-900">
              <div className="font-semibold">Direct movement: {directMotionStatus.phase.replace(/_/g," ")}</div>
              <div className="mt-1">{directMotionStatus.detail??"A previous direct movement has not been confirmed complete."}</div>
              {directMotionStatus.phase==="recovery_required"&&<>
                <p className="mt-2 leading-4">Visually verify the robot is fully stopped and the workspace is safe before unlocking. The bridge will independently verify controller idle, telemetry, errors and MoveIt validity.</p>
                <Button className="mt-3 w-full" variant="outline" onClick={recoverDirectMovement}
                  disabled={!directReady||busy!==null}>
                  {busy==="direct-recover"?<RefreshCw size={14} className="animate-spin"/>:<ShieldCheck size={14}/>} Confirm robot is idle & unlock
                </Button>
              </>}
            </div>}
            {directCheck&&<div className={directCheck.collision_free?"rounded-md border border-emerald-200 bg-emerald-50 p-3 text-xs text-emerald-800":"rounded-md border border-red-200 bg-red-50 p-3 text-xs text-red-800"}>
              <div className="font-semibold">{directCheck.collision_free?"Path check passed":"Path check failed"}</div>
              <div className="mt-1">{directCheck.reason??directCheck.message}</div>
              <div className="mt-1 font-mono text-[10px]">{directCheck.samples_checked??0} samples checked</div>
              {Array.isArray(directCheck.contacts)&&directCheck.contacts.length>0&&
                <div className="mt-2 space-y-1">{directCheck.contacts.map((contact:any,index:number)=>
                  <div key={index}>{contact.contact_body_1??contact.body_name_1??"Body"} ↔ {contact.contact_body_2??contact.body_name_2??"Body"}</div>
                )}</div>}
            </div>}
            <p className="text-[10px] leading-4 text-zinc-400">
              Sampled MoveIt pre-validation only. The Dobot controller executes its own trajectory; interpolation cannot guarantee the exact physical path or account for moving obstacles after validation.
            </p>
          </div>
        </section>
        <section className="panel">
          <div className="panel-header">
            <div>
              <div className="text-sm font-semibold">Target control</div>
              <div className="mt-0.5 text-xs text-zinc-500">Plan first · execute separately</div>
            </div>
            <Move3D size={16} className="text-zinc-400"/>
          </div>
          <div className="px-4 py-4">
            <Tabs value={tab} onValueChange={setTab}>
              <TabsList className="grid w-full grid-cols-2">
                <TabsTrigger value="tcp"><Crosshair size={13} className="mr-1.5"/> TCP coordinates</TabsTrigger>
                <TabsTrigger value="joints"><Gauge size={13} className="mr-1.5"/> Joint sliders</TabsTrigger>
              </TabsList>
              <TabsContent value="tcp">
                <div className="grid grid-cols-3 gap-3">
                  {(["x","y","z"] as const).map((key)=><NumericInput key={key} label={key.toUpperCase()}
                    value={tcpTarget?.[key]??0} suffix="mm"
                    onChange={(value)=>{
                      setPlan(null);setExecutionStatus(null);setPlannedJointSnapshot(null);
                      setTcpDirty(true);
                      setTcpTarget((current)=>({x:0,y:0,z:0,rx:0,ry:0,rz:0,...current,[key]:value}));
                    }}/>)}
                  {(["rx","ry","rz"] as const).map((key)=><NumericInput key={key} label={key.toUpperCase()}
                    value={tcpTarget?.[key]??0} suffix="deg"
                    onChange={(value)=>{
                      setPlan(null);setExecutionStatus(null);setPlannedJointSnapshot(null);
                      setTcpDirty(true);
                      setTcpTarget((current)=>({x:0,y:0,z:0,rx:0,ry:0,rz:0,...current,[key]:value}));
                    }}/>)}
                </div>
                <div className="mt-3 flex items-center justify-between text-[11px] text-zinc-400">
                  <span>Position mm · RPY degrees</span><span>Frame: {model?.base_frame??"—"}</span>
                </div>
                <p className="mt-3 text-[11px] leading-4 text-zinc-500">
                  Manual TCP targets use the MoveIt end-effector link ({model?.end_effector_link??"—"}) in {model?.base_frame??"the planning frame"}.
                  Controller ToolVectorActual values may use a different tool/user frame or TCP offset; copying them directly is not guaranteed to reproduce the same pose.
                  A failed plan can mean unreachable position/orientation, joint limits, collision, stale start state, or a missing transform. Check the MoveIt error code below.
                </p>
                <Button className="mt-4 w-full" onClick={planTcp} disabled={busy!==null||!model||!tcpTarget||!actualJointsValid}>
                  {busy==="plan"?<RefreshCw size={14} className="animate-spin"/>:<Route size={14}/>} Plan TCP target
                </Button>
              </TabsContent>
              <TabsContent value="joints">
                <div className="space-y-4">
                  {jointNames.map((name,index)=>{
                    const bound=bounds[name];
                    const rawActual=actualJoints[index];
                    const actual=typeof rawActual==="number"&&Number.isFinite(rawActual)?rawActual:null;
                    const target=jointTargets[name]??actual??0;
                    if(!bound)return <div key={name} className="text-xs text-zinc-500">{name}: position limits unavailable in URDF and MoveIt configuration.</div>;
                    return <div key={name}>
                      <div className="mb-1.5 flex items-center justify-between">
                        <span className="text-xs font-medium text-zinc-700">{name}</span>
                        <span className="font-mono text-[11px] tabular-nums text-zinc-500">
                          {formatNumber(target*180/Math.PI,1)}°
                          <span className="ml-2 text-zinc-300">actual {actual===null?"—":formatNumber(actual*180/Math.PI,1)+"°"}</span>
                        </span>
                      </div>
                      <input type="range" min={bound.lower*180/Math.PI} max={bound.upper*180/Math.PI}
                        step="0.1" value={target*180/Math.PI}
                        onChange={(event)=>{
                          setJointDirty(true);
                          const radians=Number(event.target.value)*Math.PI/180;
                          setJointTargets((current)=>({...current,[name]:radians}));
                        }} className="range-control w-full"/>
                      <div className="mt-1 flex justify-between font-mono text-[10px] text-zinc-400">
                        <span>{formatNumber(bound.lower*180/Math.PI,0)}°</span>
                        <span>{formatNumber(bound.upper*180/Math.PI,0)}°</span>
                      </div>
                    </div>;
                  })}
                  {!jointNames.length&&<div className="text-xs text-zinc-400">Waiting for joint state messages.</div>}
                  {jointNames.length>0&&!actualJointsValid&&<div className="text-xs text-amber-700">One or more joint readings are missing or invalid. Planning is disabled until all current joint positions are finite.</div>}
                </div>
                <Button className="mt-4 w-full" onClick={planJoints}
                  disabled={busy!==null||!model||!actualJointsValid||jointNames.some((name)=>!bounds[name])}>
                  {busy==="plan"?<RefreshCw size={14} className="animate-spin"/>:<Route size={14}/>} Plan joint target
                </Button>
              </TabsContent>
            </Tabs>
          </div>
        </section>

        <GripperPositionCard controller={gripper}/>

        <section className="panel">
          <div className="panel-header">
            <div className="text-sm font-semibold">Motion plan</div>
            <span className="text-[11px] text-zinc-400">{plan?.plan_id??"Not planned"}</span>
          </div>
          <div className="px-4 py-4">
            <div className="flex items-center justify-between gap-3">
              <div className="text-xs text-zinc-500">Planning result</div>
              <StatusBadge state={operationStatus(plan?.status).key} label={operationStatus(plan?.status).label} compact/>
            </div>
            <div className="mt-2 grid grid-cols-2 gap-3">
              <div className="rounded-md bg-zinc-50 p-3">
                <div className="text-[10px] uppercase tracking-wider text-zinc-400">Planning time</div>
                <div className="mt-1 font-mono text-sm">{formatNumber(plan?.planning_time,3)} s</div>
              </div>
              <div className="rounded-md bg-zinc-50 p-3">
                <div className="text-[10px] uppercase tracking-wider text-zinc-400">Trajectory points</div>
                <div className="mt-1 font-mono text-sm">{plan?.trajectory?.joint_trajectory?.points?.length??0}</div>
              </div>
            </div>
            {plan?.status==="FAILED"&&<div className="mt-3 rounded-md border border-red-200 bg-red-50 p-3 text-xs text-red-800">
              {planErrorDetail(plan.error_code)??"Planning failed without a reported MoveIt error code. Check MoveIt logs and verify target frame, orientation, reachability, and collision state."}
            </div>}
            <Button className="mt-3 w-full" variant="secondary" onClick={execute}
              disabled={!hasPlan||!canExecute||busy!==null}>
              {busy==="execute"?<RefreshCw size={14} className="animate-spin"/>:<Play size={14}/>} Execute trajectory
            </Button>
            {!canExecute&&<p className="mt-2 text-[11px] text-zinc-400">Execution is disabled by bridge safety configuration or the robot is not enabled.</p>}
            {executionStatus&&<div className="mt-3 flex items-center gap-2">
              {executionStatus==="SUCCEEDED"?<CheckCircle2 size={14} className="text-emerald-700"/>:executionStatus==="FAILED"?<CircleAlert size={14} className="text-red-700"/>:null}
              <StatusBadge state={operationStatus(executionStatus).key} label={operationStatus(executionStatus).label} compact/>
            </div>}
          </div>
        </section>

        <SceneManager
          scene={scene}
          frameId={model?.base_frame??"dummy_link"}
          modelReady={model!==null}
          connected={bridgeState==="connected"}
          request={request}
          onNotice={setNotice}
          objectLabels={objectLabels}
          onToggleObjectLabel={toggleObjectLabel}
          hiddenObjects={hiddenObjects}
          onToggleObjectVisibility={toggleObjectVisibility}
          disabledObjects={disabledObjects}
          onToggleCollision={toggleObjectCollision}
          onRemoveDisabledObject={(id)=>setDisabledObjects((current)=>{const next={...current};delete next[id];return next;})}
          onClearDisabledObjects={()=>setDisabledObjects({})}
          collisionBusy={collisionBusy}
          onSceneChanged={()=>{
            setPlan(null);
            setExecutionStatus(null);
            setPlannedJointSnapshot(null);
          }}
        />
      </aside>
    </div>
  </div>;
}
