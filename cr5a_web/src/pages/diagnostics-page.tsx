import {useMemo,useState} from "react";
import {Activity,AlertCircle,Check,Clipboard,Pause,Play,RefreshCw,Square} from "lucide-react";
import {Button} from "@/components/ui/button";
import {Metric} from "@/components/metric";
import {deriveRobotStatus,formatRobotMode,hasActiveError} from "@/lib/status";
import {formatNumber,safeDate} from "@/lib/utils";
import {useRobotSocket} from "@/lib/socket";
import {useRobotStore} from "@/store/robot-store";

export function DiagnosticsPage(){
  const {request}=useRobotSocket();
  const bridgeState=useRobotStore((s)=>s.bridgeState);
  const robotStatus=useRobotStore((s)=>s.robotStatus);
  const robotMode=useRobotStore((s)=>s.robotMode);
  const errors=useRobotStore((s)=>s.errors);
  const joints=useRobotStore((s)=>s.joints);
  const tcp=useRobotStore((s)=>s.tcp);
  const timestamps=useRobotStore((s)=>s.timestamps);
  const capabilities=useRobotStore((s)=>s.capabilities);
  const lastCommand=useRobotStore((s)=>s.lastCommand);
  const plan=useRobotStore((s)=>s.plan);
  const executionStatus=useRobotStore((s)=>s.executionStatus);
  const lastJointChangeAt=useRobotStore((s)=>s.lastJointChangeAt);
  const notice=useRobotStore((s)=>s.notice);
  const setNotice=useRobotStore((s)=>s.setNotice);
  const [busy,setBusy]=useState<string|null>(null);
  const [copied,setCopied]=useState(false);

  const status=deriveRobotStatus({
    bridgeState,connected:robotStatus?.is_connected,enabled:robotStatus?.is_enable,
    mode:robotMode,errors,lastCommand,planningStatus:plan?.status,
    executionStatus,lastJointChangeAt
  });
  const activeError=hasActiveError(errors);
  const canCommand=bridgeState==="connected"&&capabilities.robot_commands_enabled===true;
  const paused=status.key==="paused";

  const copyPayload=useMemo(()=>{
    const names=joints?.name??[];
    const positions=joints?.position??[];
    return {
      captured_at:new Date().toISOString(),
      robot:{
        connected:robotStatus?.is_connected??null,
        enabled:robotStatus?.is_enable??null,
        mode:robotMode?.value??null,
        status:status.label
      },
      tcp:tcp?{
        source_topic:"/dobot_msgs_v4/msg/ToolVectorActual",
        position_mm:[tcp.x??null,tcp.y??null,tcp.z??null],
        rpy_degrees:[tcp.rx??null,tcp.ry??null,tcp.rz??null],
        frame_id:null,
        timestamp:timestamps.tcp??null
      }:null,
      joints:joints?{
        source_topic:"/joint_states",unit:"rad",names,
        positions_rad:positions,
        positions_degrees:positions.map((value)=>value*180/Math.PI),
        timestamp:timestamps.joints??null
      }:null,
      errors
    };
  },[joints,tcp,timestamps,robotStatus,robotMode,status.label,errors]);

  const runCommand=async(command:string,label:string)=>{
    setBusy(command);setNotice(null);
    try{
      const response=await request("robot.command",{command});
      useRobotStore.getState().setCommand(command,response);
      setNotice({
        tone:response?.res===0||response?.res===undefined?"success":"error",
        message:label+" request returned controller code "+String(response?.res??"unknown")
      });
    }catch(error){
      setNotice({tone:"error",message:error instanceof Error?error.message:String(error)});
    }finally{setBusy(null);}
  };

  const clearErrors=async()=>{
    setBusy("clear-errors");setNotice(null);
    try{
      const response=await request("robot.clear_errors");
      const refreshed=await request("robot.get_errors");
      useRobotStore.getState().setErrors(refreshed?{GetErrorID:refreshed}:null);
      setNotice({
        tone:response?.res===0||response?.res===undefined?"success":"error",
        message:"Clear error request completed"
      });
    }catch(error){
      setNotice({tone:"error",message:error instanceof Error?error.message:String(error)});
    }finally{setBusy(null);}
  };

  const copyState=async()=>{
    try{
      await navigator.clipboard.writeText(JSON.stringify(copyPayload,null,2));
      setCopied(true);window.setTimeout(()=>setCopied(false),1600);
    }catch{
      setNotice({tone:"error",message:"Clipboard access was denied by the browser"});
    }
  };

  return <div className="space-y-6">
    <div className="flex flex-wrap items-end justify-between gap-3">
      <div>
        <div className="eyebrow">ROBOT / OVERVIEW</div>
        <h1 className="page-title">Diagnostics</h1>
        <p className="mt-1 text-sm text-zinc-500">Controller status, errors and live telemetry.</p>
      </div>
      <div className="flex items-center gap-2 text-xs text-zinc-500">
        <Activity size={14}/>{bridgeState==="connected"?"Streaming from ROS":"Waiting for bridge"}
      </div>
    </div>

    {notice&&<div className={notice.tone==="error"?"notice notice-error":notice.tone==="success"?"notice notice-success":"notice"}>{notice.message}</div>}

    <section className="panel overflow-hidden">
      <div className="flex flex-wrap items-center justify-between gap-4 border-b border-zinc-100 px-5 py-4">
        <div className="flex items-center gap-3">
          <div className="flex h-10 w-10 items-center justify-center rounded-lg bg-zinc-100 text-zinc-700"><Activity size={19}/></div>
          <div>
            <div className="text-sm font-semibold">Robot controller</div>
            <div className="mt-0.5 text-xs text-zinc-500">Dobot CR5A · Driver status topic</div>
          </div>
        </div>
      </div>
      <div className="grid gap-5 px-5 py-5 sm:grid-cols-3">
        <Metric label="Robot mode" value={formatRobotMode(robotMode?.value)} hint={robotMode?.res===0?"RobotMode service · successful":"Raw controller response"}/>
        <Metric label="Connection" value={robotStatus?.is_connected===true?"Connected":robotStatus?.is_connected===false?"Disconnected":"Unknown"}/>
        <Metric label="Motors" value={robotStatus?.is_enable===true?"Enabled":robotStatus?.is_enable===false?"Disabled":"Unknown"}/>
      </div>
      <div className="flex flex-wrap items-center gap-2 border-t border-zinc-100 bg-zinc-50/70 px-5 py-4">
        <Button variant="destructive" onClick={()=>runCommand("stop","Stop")} disabled={!canCommand||busy!==null}>
          <Square size={14} fill="currentColor"/> Stop
        </Button>
        <Button variant="outline" onClick={()=>runCommand("pause","Pause")} disabled={!canCommand||busy!==null||paused}>
          {busy==="pause"?<RefreshCw size={14} className="animate-spin"/>:<Pause size={14}/>} Pause
        </Button>
        <Button variant="outline" onClick={()=>runCommand("resume","Resume")} disabled={!canCommand||busy!==null||!paused}>
          {busy==="resume"?<RefreshCw size={14} className="animate-spin"/>:<Play size={14}/>} Play
        </Button>
        <div className="flex-1"/>
        <Button variant="outline" onClick={clearErrors} disabled={bridgeState!=="connected"||capabilities.error_clear_enabled!==true||busy!==null}>
          <RefreshCw size={14}/> Clear errors
        </Button>
      </div>
      {!canCommand&&<div className="border-t border-zinc-100 px-5 py-2.5 text-xs text-zinc-500">Robot commands are disabled by the bridge safety configuration.</div>}
    </section>

    {activeError&&<section className="rounded-xl border border-red-200 bg-red-50 px-5 py-4">
      <div className="flex items-center gap-2 text-sm font-semibold text-red-800"><AlertCircle size={16}/> Controller error reported</div>
      <pre className="mt-3 max-h-36 overflow-auto whitespace-pre-wrap break-words rounded-md border border-red-100 bg-white/70 p-3 font-mono text-xs text-red-900">{JSON.stringify(errors,null,2)}</pre>
    </section>}

    <div className="grid gap-5 lg:grid-cols-[1.15fr_0.85fr]">
      <section className="panel">
        <div className="panel-header">
          <div>
            <div className="text-sm font-semibold">Live TCP coordinates</div>
            <div className="mt-0.5 text-xs text-zinc-500">ToolVectorActual · controller-reported values</div>
          </div>
          <span className="text-[11px] text-zinc-400">{safeDate(timestamps.tcp)}</span>
        </div>
        <div className="grid grid-cols-3 gap-x-4 gap-y-5 px-5 py-5">
          <Metric label="X" value={formatNumber(tcp?.x,2)} suffix="mm"/>
          <Metric label="Y" value={formatNumber(tcp?.y,2)} suffix="mm"/>
          <Metric label="Z" value={formatNumber(tcp?.z,2)} suffix="mm"/>
          <Metric label="RX" value={formatNumber(tcp?.rx,2)} suffix="deg"/>
          <Metric label="RY" value={formatNumber(tcp?.ry,2)} suffix="deg"/>
          <Metric label="RZ" value={formatNumber(tcp?.rz,2)} suffix="deg"/>
        </div>
      </section>

      <section className="panel">
        <div className="panel-header">
          <div>
            <div className="text-sm font-semibold">Joint positions</div>
            <div className="mt-0.5 text-xs text-zinc-500">/joint_states · radians</div>
          </div>
          <span className="text-[11px] text-zinc-400">{safeDate(timestamps.joints)}</span>
        </div>
        <div className="grid grid-cols-2 gap-x-4 gap-y-4 px-5 py-5 sm:grid-cols-3">
          {(joints?.name??[]).map((name,index)=><Metric key={name} label={name} value={formatNumber(joints?.position?.[index],4)} suffix="rad"/>)}
          {!joints?.name?.length&&<div className="col-span-full text-sm text-zinc-400">Waiting for joint state messages.</div>}
        </div>
      </section>
    </div>

    <section className="panel overflow-hidden">
      <div className="panel-header">
        <div>
          <div className="text-sm font-semibold">Robot state snapshot</div>
          <div className="mt-0.5 text-xs text-zinc-500">Copyable telemetry with source topics, units and timestamps.</div>
        </div>
        <Button variant="outline" size="sm" onClick={copyState}>{copied?<Check size={14}/>:<Clipboard size={14}/>} {copied?"Copied":"Copy JSON"}</Button>
      </div>
      <pre className="max-h-64 overflow-auto bg-zinc-950 px-5 py-4 font-mono text-[11px] leading-5 text-zinc-200">{JSON.stringify(copyPayload,null,2)}</pre>
    </section>

    <div className="text-[11px] text-zinc-400">Stop is the normal controller Stop command, not a safety-rated emergency stop. The physical E-stop circuit remains independent.</div>
  </div>;
}
