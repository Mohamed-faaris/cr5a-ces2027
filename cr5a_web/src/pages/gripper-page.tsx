import {useEffect,useMemo,useState} from "react";
import {AlertCircle,Hand,MoveHorizontal,Power,RefreshCw,Square,Wrench} from "lucide-react";
import {Button} from "@/components/ui/button";
import {Metric} from "@/components/metric";
import {StatusBadge} from "@/components/status-badge";
import {hasActiveError} from "@/lib/status";
import {useRobotSocket} from "@/lib/socket";
import {useRobotStore} from "@/store/robot-store";

type GripperSnapshot={
  init?:number;init_text?:string;grip?:number;grip_text?:string;
  position?:number;current?:number;error?:number;temperature?:number|null;
  alarm?:{code?:number;name?:string;remedy?:string};
};
type GripperState={
  connected:boolean;simulation:boolean;status:GripperSnapshot|null;
  error:string|null;commands_enabled:boolean;message?:string;
};

const STROKE_MM=20;

export function GripperPage(){
  const {request}=useRobotSocket();
  const bridgeState=useRobotStore((state)=>state.bridgeState);
  const robotStatus=useRobotStore((state)=>state.robotStatus);
  const robotMode=useRobotStore((state)=>state.robotMode);
  const capabilities=useRobotStore((state)=>state.capabilities);
  const errors=useRobotStore((state)=>state.errors);
  const simulation=useRobotStore((state)=>state.simulation);
  const [gripper,setGripper]=useState<GripperState|null>(null);
  const [targetMm,setTargetMm]=useState("0");
  const [targetDirty,setTargetDirty]=useState(false);
  const [busy,setBusy]=useState<"move"|"initialize"|"calibrate"|"stop"|null>(null);
  const [notice,setNotice]=useState<{tone:"success"|"error"|"neutral";message:string}|null>(null);
  const [refreshing,setRefreshing]=useState(false);

  const refresh=async(syncTarget=false)=>{
    setRefreshing(true);
    try{
      const value=await request("gripper.get_status") as GripperState;
      setGripper(value);
      const actual=Number(value?.status?.position);
      if(syncTarget&&!targetDirty&&Number.isFinite(actual)){
        setTargetMm(String(Number((actual/1000*STROKE_MM).toFixed(2))));
      }
    }catch(error){
      setGripper({connected:false,simulation, status:null,error:error instanceof Error?error.message:String(error),commands_enabled:false});
    }finally{setRefreshing(false);}
  };

  useEffect(()=>{
    void refresh(true);
    const timer=window.setInterval(()=>{void refresh(false);},1500);
    return ()=>window.clearInterval(timer);
  },[request]);

  const status=gripper?.status??null;
  const positionMm=typeof status?.position==="number"?status.position/1000*STROKE_MM:null;
  const initialized=status?.init===1;
  const controllerEnabled=robotStatus?.is_connected===true&&robotStatus?.is_enable===true&&robotMode?.value==="5";
  const activeRobotError=hasActiveError(errors);
  const canControl=bridgeState==="connected"&&gripper?.connected===true&&
    gripper.commands_enabled===true&&capabilities.gripper_commands_enabled===true&&
    robotStatus?.is_connected===true&&!activeRobotError&&busy===null;
  const canMove=canControl&&initialized&&controllerEnabled;
  const numericTarget=Number(targetMm);
  const targetValid=Number.isFinite(numericTarget)&&numericTarget>=0&&numericTarget<=STROKE_MM;
  const currentLabel=positionMm===null?"—":positionMm.toFixed(2);
  const sourceLabel=gripper?.simulation||simulation?"Fake PGEA":"PGEA-100-40-W";
  const alarm=status?.alarm;
  const alarmActive=typeof alarm?.code==="number"&&alarm.code!==0;

  const sendMove=async(mm:number)=>{
    if(!Number.isFinite(mm)||mm<0||mm>STROKE_MM){
      setNotice({tone:"error",message:"Target must be between 0 and 20 mm per finger."});
      return;
    }
    const direction=mm===0?"open":mm===STROKE_MM?"close":"move";
    if(!window.confirm("Command the PGEA to "+direction+" to "+mm.toFixed(1)+" mm per finger? Confirm the finger travel area is clear."))return;
    setBusy("move");setNotice(null);
    try{
      const result=await request("gripper.move",{position_m:mm/1000,max_effort:0});
      setNotice({
        tone:result?.reached_goal===true?"success":"error",
        message:result?.reached_goal===true?"PGEA reached the requested position.":"PGEA did not confirm the requested position."
      });
      setTargetDirty(false);
      await refresh(true);
    }catch(error){
      setNotice({tone:"error",message:error instanceof Error?error.message:String(error)});
      setTargetDirty(false);
      await refresh(true);
    }finally{setBusy(null);}
  };

  const lifecycle=async(action:"initialize"|"calibrate")=>{
    const message=action==="calibrate"
      ?"Run full close-to-open PGEA calibration? Remove the workpiece and clear the full finger travel area. The fingers will move through their range."
      :"Initialize the PGEA? The fingers may move during initialization.";
    if(!window.confirm(message))return;
    setBusy(action);setNotice(null);
    try{
      const result=await request("gripper."+action);
      setNotice({tone:"success",message:result?.message??(action==="initialize"?"PGEA initialized.":"PGEA calibration completed.")});
      await refresh();
    }catch(error){
      setNotice({tone:"error",message:error instanceof Error?error.message:String(error)});
      await refresh();
    }finally{setBusy(null);}
  };

  const stop=async()=>{
    setBusy("stop");
    try{
      await request("gripper.stop");
      setNotice({tone:"neutral",message:"PGEA stop requested."});
    }catch(error){
      setNotice({tone:"error",message:error instanceof Error?error.message:String(error)});
    }finally{setBusy(null);await refresh();}
  };

  const readiness=useMemo(()=>{
    if(bridgeState!=="connected")return "Waiting for the robot bridge.";
    if(!gripper?.connected)return gripper?.error??"PGEA status service is not available. Check that the gripper node is launched.";
    if(!robotStatus?.is_connected)return "Dobot arm connection is not confirmed.";
    if(activeRobotError)return "Clear the active Dobot controller error before operating the gripper.";
    if(!gripper.commands_enabled||!capabilities.gripper_commands_enabled)return "PGEA commands are disabled by the bridge safety configuration.";
    if(!initialized)return "Initialize the PGEA before commanding finger movement.";
    if(!controllerEnabled)return "Enable the robot from Diagnostics before moving the gripper.";
    return "PGEA is ready for position commands.";
  },[bridgeState,gripper,robotStatus,activeRobotError,capabilities.gripper_commands_enabled,initialized,controllerEnabled]);

  return <div className="space-y-5">
    <div className="flex flex-wrap items-end justify-between gap-3">
      <div>
        <div className="eyebrow">END EFFECTOR / PGEA</div>
        <h1 className="page-title">Gripper control</h1>
        <p className="mt-1 text-sm text-zinc-500">Position control and initialization for the PGEA-100-40-W parallel gripper.</p>
      </div>
      <div className="flex items-center gap-2">
        <StatusBadge
          state={!gripper?.connected?"disconnected":busy==="move"||busy==="stop"?"moving":"idle"}
          label={gripper?.connected?sourceLabel:"Gripper offline"}
        />
        <Button variant="outline" size="sm" onClick={()=>void refresh(true)} disabled={refreshing}>
          <RefreshCw size={13} className={refreshing?"animate-spin":""}/> Refresh
        </Button>
      </div>
    </div>

    {notice&&<div className={notice.tone==="error"?"notice notice-error":notice.tone==="success"?"notice notice-success":"notice"}>{notice.message}</div>}
    {gripper?.error&&<div className="notice notice-error">{gripper.error}</div>}
    {alarmActive&&<div className="rounded-lg border border-red-200 bg-red-50 px-4 py-3">
      <div className="flex items-center gap-2 text-sm font-semibold text-red-800"><AlertCircle size={15}/>{alarm?.name??"PGEA alarm"} · code {alarm?.code}</div>
      {alarm?.remedy&&<p className="mt-1 text-xs text-red-700">{alarm.remedy}</p>}
    </div>}

    <section className="panel">
      <div className="panel-header">
        <div className="flex items-center gap-3">
          <div className="flex h-10 w-10 items-center justify-center rounded-lg bg-zinc-100 text-zinc-700"><Hand size={19}/></div>
          <div>
            <div className="text-sm font-semibold">Gripper telemetry</div>
            <div className="mt-0.5 text-xs text-zinc-500">{gripper?.simulation||simulation?"Simulated PGEA action and register state":"PGEA Modbus feedback · tool RS-485"}</div>
          </div>
        </div>
        <span className="text-[11px] text-zinc-400">{status?.grip_text??"Waiting for feedback"}</span>
      </div>
      <div className="grid grid-cols-2 gap-5 px-5 py-5 sm:grid-cols-4">
        <Metric label="Connection" value={gripper?.connected?"Connected":"Offline"}/>
        <Metric label="Initialization" value={status?.init_text??"Unknown"}/>
        <Metric label="Finger opening" value={currentLabel} suffix="mm / finger"/>
        <Metric label="Motor current" value={typeof status?.current==="number"?String(status.current):"—"} hint="Raw register counts · not Newtons"/>
      </div>
      <div className="border-t border-zinc-100 px-5 py-3 text-xs text-zinc-600">{readiness}</div>
    </section>

    <div className="grid items-start gap-5 lg:grid-cols-[minmax(0,1fr)_340px]">
      <section className="panel">
        <div className="panel-header">
          <div>
            <div className="text-sm font-semibold">Position control</div>
            <div className="mt-0.5 text-xs text-zinc-500">0 mm = open · 20 mm = fully closed · per finger</div>
          </div>
          <MoveHorizontal size={16} className="text-zinc-400"/>
        </div>
        <div className="space-y-5 px-5 py-5">
          <div className="flex items-end justify-between gap-4">
            <label className="block min-w-0">
              <span className="mb-1.5 block text-[11px] font-medium uppercase tracking-wider text-zinc-500">Target opening</span>
              <div className="flex h-10 items-center rounded-md border border-zinc-200 bg-white focus-within:border-zinc-500">
                <input type="number" min="0" max="20" step="0.1" value={targetMm}
                  onChange={(event)=>{setTargetDirty(true);setTargetMm(event.target.value);}}
                  className="min-w-0 flex-1 bg-transparent px-3 font-mono text-lg outline-none"/>
                <span className="px-3 text-xs text-zinc-400">mm / finger</span>
              </div>
            </label>
            <div className="pb-1 text-right">
              <div className="text-[10px] uppercase tracking-wider text-zinc-400">Actual</div>
              <div className="font-mono text-xl tabular-nums">{currentLabel}<span className="ml-1 text-xs text-zinc-400">mm</span></div>
            </div>
          </div>
          <input type="range" min="0" max="20" step="0.1" value={Math.max(0,Math.min(20,Number(targetMm)||0))}
            onChange={(event)=>{setTargetDirty(true);setTargetMm(event.target.value);}} className="range-control w-full" aria-label="Target PGEA opening"/>
          <div className="flex justify-between font-mono text-[10px] text-zinc-400"><span>0 mm · Open</span><span>10 mm · Mid travel</span><span>20 mm · Close</span></div>
          <div className="flex flex-wrap gap-2">
            <Button onClick={()=>void sendMove(Number(targetMm))} disabled={!canMove||!targetValid}>
              {busy==="move"?<RefreshCw size={14} className="animate-spin"/>:<MoveHorizontal size={14}/>} Move to target
            </Button>
            <Button variant="outline" onClick={()=>{setTargetMm("0");void sendMove(0);}} disabled={!canMove}><Hand size={14}/> Open</Button>
            <Button variant="outline" onClick={()=>{setTargetMm("20");void sendMove(20);}} disabled={!canMove}><Hand size={14}/> Close</Button>
            <div className="flex-1"/>
            <Button variant="destructive" onClick={()=>void stop()} disabled={busy!=="move"}><Square size={13} fill="currentColor"/> Stop</Button>
          </div>
          {!targetValid&&<p className="text-xs text-red-700">Enter a finite target between 0 and 20 mm.</p>}
          <div className="rounded-md border border-zinc-200 bg-zinc-50 px-3 py-2.5 text-xs leading-5 text-zinc-600">
            The PGEA force register uses percent, while ROS GripperCommand.max_effort uses Newtons. No calibrated conversion is available, so commands use max_effort = 0 and the gripper node's configured force percentage.
          </div>
        </div>
      </section>

      <section className="panel">
        <div className="panel-header">
          <div className="text-sm font-semibold">Initialization</div>
          <Wrench size={15} className="text-zinc-400"/>
        </div>
        <div className="space-y-4 px-4 py-4">
          <p className="text-xs leading-5 text-zinc-500">Initialize after power-up or reconnect. Full calibration performs a close-to-open stroke and re-measures the travel range.</p>
          <Button className="w-full" onClick={()=>void lifecycle("initialize")}
            disabled={!canControl||busy!==null}>
            {busy==="initialize"?<RefreshCw size={14} className="animate-spin"/>:<Power size={14}/>} Initialize
          </Button>
          <Button variant="outline" className="w-full" onClick={()=>void lifecycle("calibrate")}
            disabled={!canControl||busy!==null}>
            {busy==="calibrate"?<RefreshCw size={14} className="animate-spin"/>:<Wrench size={14}/>} Full calibration
          </Button>
          <div className="rounded-md border border-amber-200 bg-amber-50 px-3 py-2.5 text-xs leading-5 text-amber-800">
            Before calibration, remove the workpiece and clear the complete finger travel area. Calibration moves the fingers.
          </div>
          <div className="space-y-2 border-t border-zinc-100 pt-3 text-xs">
            <div className="flex justify-between gap-3"><span className="text-zinc-500">Grip state</span><span className="font-medium text-zinc-800">{status?.grip_text??"—"}</span></div>
            <div className="flex justify-between gap-3"><span className="text-zinc-500">Alarm</span><span className="font-medium text-zinc-800">{alarm?.name??"—"}</span></div>
            <div className="flex justify-between gap-3"><span className="text-zinc-500">Alarm code</span><span className="font-mono text-zinc-800">{alarm?.code??status?.error??"—"}</span></div>
            <div className="flex justify-between gap-3"><span className="text-zinc-500">Temperature</span><span className="font-mono text-zinc-800">{status?.temperature??"—"}</span></div>
          </div>
        </div>
      </section>
    </div>
    <div className="text-[11px] text-zinc-400">Software cancellation is not a safety-rated protective stop. Keep the physical E-stop accessible and verify speed/force during commissioning.</div>
  </div>;
}
