import {useEffect,useMemo,useState} from "react";
import {Box,CheckCircle2,CircleAlert,Crosshair,Gauge,Move3D,Play,RefreshCw,Route} from "lucide-react";
import {Button} from "@/components/ui/button";
import {Tabs,TabsContent,TabsList,TabsTrigger} from "@/components/ui/tabs";
import {StatusBadge} from "@/components/status-badge";
import {RobotViewport} from "@/components/robot/robot-viewport";
import {deriveRobotStatus} from "@/lib/status";
import {formatNumber,parseJointBounds,rpyDegreesToQuaternion} from "@/lib/utils";
import {useRobotSocket} from "@/lib/socket";
import {useRobotStore} from "@/store/robot-store";

type TcpTarget={x:number;y:number;z:number;rx:number;ry:number;rz:number};

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
  const {request}=useRobotSocket();
  const bridgeState=useRobotStore((s)=>s.bridgeState);
  const robotStatus=useRobotStore((s)=>s.robotStatus);
  const robotMode=useRobotStore((s)=>s.robotMode);
  const errors=useRobotStore((s)=>s.errors);
  const lastCommand=useRobotStore((s)=>s.lastCommand);
  const lastJointChangeAt=useRobotStore((s)=>s.lastJointChangeAt);
  const joints=useRobotStore((s)=>s.joints);
  const tcp=useRobotStore((s)=>s.tcp);
  const model=useRobotStore((s)=>s.model);
  const modelLoading=useRobotStore((s)=>s.modelLoading);
  const modelError=useRobotStore((s)=>s.modelError);
  const scene=useRobotStore((s)=>s.scene);
  const sceneLoading=useRobotStore((s)=>s.sceneLoading);
  const sceneError=useRobotStore((s)=>s.sceneError);
  const capabilities=useRobotStore((s)=>s.capabilities);
  const plan=useRobotStore((s)=>s.plan);
  const executionStatus=useRobotStore((s)=>s.executionStatus);
  const setPlan=useRobotStore((s)=>s.setPlan);
  const setExecutionStatus=useRobotStore((s)=>s.setExecutionStatus);
  const setNotice=useRobotStore((s)=>s.setNotice);
  const notice=useRobotStore((s)=>s.notice);

  const [tab,setTab]=useState("tcp");
  const [tcpTarget,setTcpTarget]=useState<TcpTarget|null>(null);
  const [tcpDirty,setTcpDirty]=useState(false);
  const [jointTargets,setJointTargets]=useState<Record<string,number>>({});
  const [jointDirty,setJointDirty]=useState(false);
  const [busy,setBusy]=useState<string|null>(null);

  const status=deriveRobotStatus({
    bridgeState,connected:robotStatus?.is_connected,enabled:robotStatus?.is_enable,
    mode:robotMode,errors,lastCommand,planningStatus:plan?.status,
    executionStatus,lastJointChangeAt
  });
  const bounds=useMemo(()=>parseJointBounds(model?.urdf),[model?.urdf]);
  const jointNames=joints?.name??[];
  const actualJoints=joints?.position??[];
  const worldObjects=scene?.world?.collision_objects??[];
  const attachedObjects=scene?.robot_state?.attached_collision_objects??[];

  useEffect(()=>{
    if(!tcp||tcpDirty)return;
    setTcpTarget({
      x:tcp.x??0,y:tcp.y??0,z:tcp.z??0,
      rx:tcp.rx??0,ry:tcp.ry??0,rz:tcp.rz??0
    });
  },[tcp,tcpDirty]);

  useEffect(()=>{
    if(jointDirty)return;
    const next:Record<string,number>={};
    jointNames.forEach((name,index)=>{next[name]=actualJoints[index]??0;});
    setJointTargets(next);
  },[jointNames.join("|"),actualJoints.join("|"),jointDirty]);

  const canExecute=capabilities.motion_execution_enabled===true&&bridgeState==="connected"&&
    robotStatus?.is_connected===true&&robotStatus?.is_enable===true;
  const hasPlan=plan?.status==="SUCCEEDED"&&Boolean(plan.plan_id);

  const resetTargets=()=>{
    setTcpDirty(false);setJointDirty(false);
    if(tcp)setTcpTarget({
      x:tcp.x??0,y:tcp.y??0,z:tcp.z??0,rx:tcp.rx??0,ry:tcp.ry??0,rz:tcp.rz??0
    });
    const next:Record<string,number>={};
    jointNames.forEach((name,index)=>{next[name]=actualJoints[index]??0;});
    setJointTargets(next);
  };

  const planTcp=async()=>{
    if(!tcpTarget||!model)return;
    setBusy("plan");setNotice(null);setPlan({status:"PLANNING"});
    try{
      const quaternion=rpyDegreesToQuaternion(tcpTarget.rx,tcpTarget.ry,tcpTarget.rz);
      const result=await request("motion.plan",{
        group_name:model.group_name,
        target:{
          type:"pose",frame_id:model.base_frame,link_name:model.end_effector_link,
          position_m:[tcpTarget.x/1000,tcpTarget.y/1000,tcpTarget.z/1000],
          orientation_xyzw:quaternion,position_tolerance_m:0.002,orientation_tolerance_rad:0.02
        },
        allowed_planning_time:5,max_velocity_scaling:0.2,max_acceleration_scaling:0.2
      });
      setPlan(result);
      setNotice({
        tone:result.status==="SUCCEEDED"?"success":"error",
        message:result.status==="SUCCEEDED"?"Plan generated. Inspect the path before execution.":"MoveIt could not find a valid plan."
      });
    }catch(error){
      setPlan({status:"FAILED"});
      setNotice({tone:"error",message:error instanceof Error?error.message:String(error)});
    }finally{setBusy(null);}
  };

  const planJoints=async()=>{
    if(!model||!jointNames.length)return;
    setBusy("plan");setNotice(null);setPlan({status:"PLANNING"});
    try{
      const result=await request("motion.plan",{
        group_name:model.group_name,
        target:{
          type:"joints",joint_names:jointNames,
          positions_rad:jointNames.map((name)=>jointTargets[name]??0),tolerance_rad:0.01
        },
        allowed_planning_time:5,max_velocity_scaling:0.2,max_acceleration_scaling:0.2
      });
      setPlan(result);
      setNotice({
        tone:result.status==="SUCCEEDED"?"success":"error",
        message:result.status==="SUCCEEDED"?"Joint-space plan generated.":"MoveIt could not find a valid plan."
      });
    }catch(error){
      setPlan({status:"FAILED"});
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
      <div className="flex items-center gap-2">
        <StatusBadge state={status.key}/>
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
          loading={modelLoading||sceneLoading} error={modelError??sceneError}/>
        <div className="flex flex-wrap items-center justify-between gap-2 border-t border-zinc-100 px-4 py-3 text-[11px] text-zinc-500">
          <span>Blue line: planned end-effector path</span>
          <span>{plan?.status?"Plan: "+plan.status:"No planned trajectory"}</span>
        </div>
      </section>

      <aside className="space-y-4">
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
                      setTcpDirty(true);
                      setTcpTarget((current)=>({x:0,y:0,z:0,rx:0,ry:0,rz:0,...current,[key]:value}));
                    }}/>)}
                  {(["rx","ry","rz"] as const).map((key)=><NumericInput key={key} label={key.toUpperCase()}
                    value={tcpTarget?.[key]??0} suffix="deg"
                    onChange={(value)=>{
                      setTcpDirty(true);
                      setTcpTarget((current)=>({x:0,y:0,z:0,rx:0,ry:0,rz:0,...current,[key]:value}));
                    }}/>)}
                </div>
                <div className="mt-3 flex items-center justify-between text-[11px] text-zinc-400">
                  <span>Position mm · RPY degrees</span><span>Frame: {model?.base_frame??"—"}</span>
                </div>
                <Button className="mt-4 w-full" onClick={planTcp} disabled={busy!==null||!model||!tcpTarget}>
                  {busy==="plan"?<RefreshCw size={14} className="animate-spin"/>:<Route size={14}/>} Plan TCP target
                </Button>
              </TabsContent>
              <TabsContent value="joints">
                <div className="space-y-4">
                  {jointNames.map((name,index)=>{
                    const bound=bounds[name];
                    const actual=actualJoints[index]??0;
                    const target=jointTargets[name]??actual;
                    if(!bound)return <div key={name} className="text-xs text-zinc-500">{name}: position limits unavailable in URDF.</div>;
                    return <div key={name}>
                      <div className="mb-1.5 flex items-center justify-between">
                        <span className="text-xs font-medium text-zinc-700">{name}</span>
                        <span className="font-mono text-[11px] tabular-nums text-zinc-500">
                          {formatNumber(target*180/Math.PI,1)}°
                          <span className="ml-2 text-zinc-300">actual {formatNumber(actual*180/Math.PI,1)}°</span>
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
                </div>
                <Button className="mt-4 w-full" onClick={planJoints}
                  disabled={busy!==null||!model||!jointNames.length||jointNames.some((name)=>!bounds[name])}>
                  {busy==="plan"?<RefreshCw size={14} className="animate-spin"/>:<Route size={14}/>} Plan joint target
                </Button>
              </TabsContent>
            </Tabs>
          </div>
        </section>

        <section className="panel">
          <div className="panel-header">
            <div className="text-sm font-semibold">Motion plan</div>
            <span className="text-[11px] text-zinc-400">{plan?.plan_id??"Not planned"}</span>
          </div>
          <div className="px-4 py-4">
            <div className="flex items-center justify-between gap-3">
              <div className="text-xs text-zinc-500">Planning result</div>
              <span className="text-xs font-medium text-zinc-800">{plan?.status??"No plan"}</span>
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
            <Button className="mt-3 w-full" variant="secondary" onClick={execute}
              disabled={!hasPlan||!canExecute||busy!==null}>
              {busy==="execute"?<RefreshCw size={14} className="animate-spin"/>:<Play size={14}/>} Execute trajectory
            </Button>
            {!canExecute&&<p className="mt-2 text-[11px] text-zinc-400">Execution is disabled by bridge safety configuration or the robot is not enabled.</p>}
            {executionStatus&&<div className="mt-3 flex items-center gap-2 text-xs text-zinc-600">
              {executionStatus==="SUCCEEDED"?<CheckCircle2 size={14}/>:<CircleAlert size={14}/>} Execution: {executionStatus}
            </div>}
          </div>
        </section>

        <section className="panel">
          <div className="panel-header">
            <div className="text-sm font-semibold">Scene objects</div>
            <span className="text-[11px] text-zinc-400">{worldObjects.length+attachedObjects.length} total</span>
          </div>
          <div className="max-h-44 divide-y divide-zinc-100 overflow-auto">
            {worldObjects.map((object)=><div key={object.id} className="flex items-center justify-between gap-3 px-4 py-2.5">
              <div className="flex min-w-0 items-center gap-2"><Box size={13} className="shrink-0 text-zinc-400"/><span className="truncate text-xs text-zinc-700">{object.id}</span></div>
              <span className="text-[10px] text-zinc-400">{object.header?.frame_id??"frame ?"}</span>
            </div>)}
            {attachedObjects.map((item)=><div key={item.object?.id} className="flex items-center justify-between gap-3 px-4 py-2.5">
              <div className="flex min-w-0 items-center gap-2"><Box size={13} className="shrink-0 text-zinc-400"/><span className="truncate text-xs text-zinc-700">{item.object?.id??"Attached object"}</span></div>
              <span className="text-[10px] text-zinc-400">{item.link_name??"link ?"}</span>
            </div>)}
            {worldObjects.length===0&&attachedObjects.length===0&&<div className="px-4 py-5 text-xs text-zinc-400">No collision objects in the current planning scene.</div>}
          </div>
        </section>
      </aside>
    </div>
  </div>;
}
