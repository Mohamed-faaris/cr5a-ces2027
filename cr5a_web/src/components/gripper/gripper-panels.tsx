import {AlertCircle,Hand,MoveHorizontal,Power,RefreshCw,Square,Wrench} from "lucide-react";
import {Link} from "react-router-dom";
import {Button} from "@/components/ui/button";
import {Metric} from "@/components/metric";
import {StatusBadge} from "@/components/status-badge";
import type {GripperController} from "./use-gripper-controller";

function Notice({controller}:{controller:GripperController}){
  const notice=controller.notice;
  if(!notice)return null;
  return <div className={notice.tone==="error"?"notice notice-error":notice.tone==="success"?"notice notice-success":"notice"}>
    {notice.message}
  </div>;
}

export function GripperDiagnosticsCard({controller}:{controller:GripperController}){
  const {gripper,status,alarm,alarmActive,sourceLabel,busy,refreshing,readiness}=controller;
  const moving=busy==="move"||busy==="stop";
  return <section className="panel overflow-hidden">
    <div className="panel-header">
      <div className="flex items-center gap-3">
        <div className="flex h-10 w-10 items-center justify-center rounded-lg bg-zinc-100 text-zinc-700">
          <Hand size={19}/>
        </div>
        <div>
          <div className="text-sm font-semibold">PGEA gripper diagnostics</div>
          <div className="mt-0.5 text-xs text-zinc-500">
            {gripper?.simulation?"Simulated PGEA action and register state":"PGEA Modbus feedback · tool RS-485"}
          </div>
        </div>
      </div>
      <div className="flex items-center gap-2">
        <StatusBadge
          state={!gripper?.connected?"disconnected":moving?"moving":"idle"}
          label={gripper?.connected?sourceLabel:"Gripper offline"}
        />
        <Button variant="outline" size="sm" onClick={()=>void controller.refresh(true)} disabled={refreshing}>
          <RefreshCw size={13} className={refreshing?"animate-spin":""}/> Refresh
        </Button>
      </div>
    </div>
    <div className="space-y-3 px-5 py-4">
      <Notice controller={controller}/>
      {gripper?.error&&<div className="notice notice-error">{gripper.error}</div>}
      {alarmActive&&<div className="rounded-lg border border-red-200 bg-red-50 px-4 py-3">
        <div className="flex items-center gap-2 text-sm font-semibold text-red-800">
          <AlertCircle size={15}/>{alarm?.name??"PGEA alarm"} · code {alarm?.code}
        </div>
        {alarm?.remedy&&<p className="mt-1 text-xs text-red-700">{alarm.remedy}</p>}
      </div>}
      <div className="grid grid-cols-2 gap-4 sm:grid-cols-4">
        <Metric label="Connection" value={gripper?.connected?"Connected":"Offline"}/>
        <Metric label="Initialization" value={status?.init_text??"Unknown"}/>
        <Metric label="Finger opening" value={controller.currentLabel} suffix="mm / finger"/>
        <Metric label="Motor current" value={typeof status?.current==="number"?String(status.current):"—"} hint="Raw register counts"/>
      </div>
      <div className="grid grid-cols-2 gap-x-4 gap-y-2 border-t border-zinc-100 pt-3 text-xs sm:grid-cols-4">
        <div><span className="text-zinc-500">Grip state</span><div className="mt-1 font-medium text-zinc-800">{status?.grip_text??"—"}</div></div>
        <div><span className="text-zinc-500">Alarm</span><div className="mt-1 font-medium text-zinc-800">{alarm?.name??"—"}</div></div>
        <div><span className="text-zinc-500">Alarm code</span><div className="mt-1 font-mono text-zinc-800">{alarm?.code??status?.error??"—"}</div></div>
        <div><span className="text-zinc-500">Temperature</span><div className="mt-1 font-mono text-zinc-800">{status?.temperature??"—"}</div></div>
      </div>
      <div className="flex flex-wrap items-center gap-2 border-t border-zinc-100 pt-3">
        <Button size="sm" onClick={()=>void controller.lifecycle("initialize")}
          disabled={!controller.canControl||busy!==null}>
          {busy==="initialize"?<RefreshCw size={13} className="animate-spin"/>:<Power size={13}/>} Initialize
        </Button>
        <Button size="sm" variant="outline" onClick={()=>void controller.lifecycle("calibrate")}
          disabled={!controller.canControl||busy!==null}>
          {busy==="calibrate"?<RefreshCw size={13} className="animate-spin"/>:<Wrench size={13}/>} Full calibration
        </Button>
        <span className="ml-auto text-[11px] text-zinc-500">{readiness}</span>
      </div>
      <div className="rounded-md border border-amber-200 bg-amber-50 px-3 py-2 text-xs leading-5 text-amber-800">
        Full calibration moves the fingers through their entire stroke. Remove the workpiece and clear the full travel area first.
      </div>
      <div className="text-right text-[11px]">
        <Link to="/gripper" className="font-medium text-zinc-600 underline decoration-zinc-300 underline-offset-2 hover:text-zinc-950">
          Open full gripper panel
        </Link>
      </div>
    </div>
  </section>;
}

export function GripperPositionCard({controller}:{controller:GripperController}){
  const {status,busy,currentLabel,targetMm,targetValid,readiness}=controller;
  return <section className="panel overflow-hidden">
    <div className="panel-header">
      <div>
        <div className="text-sm font-semibold">Gripper controls</div>
        <div className="mt-0.5 text-xs text-zinc-500">PGEA-100-40-W · 0–20 mm per finger</div>
      </div>
      <MoveHorizontal size={16} className="text-zinc-400"/>
    </div>
    <div className="space-y-4 px-4 py-4">
      <Notice controller={controller}/>
      <div className="flex items-end justify-between gap-4">
        <label className="block min-w-0 flex-1">
          <span className="mb-1.5 block text-[11px] font-medium uppercase tracking-wider text-zinc-500">Target opening</span>
          <div className="flex h-10 items-center rounded-md border border-zinc-200 bg-white focus-within:border-zinc-500">
            <input type="number" min="0" max="20" step="0.1" value={targetMm}
              disabled={busy!==null}
              onChange={(event)=>controller.setTargetMm(event.target.value)}
              className="min-w-0 flex-1 bg-transparent px-3 font-mono text-lg outline-none disabled:opacity-50"/>
            <span className="px-3 text-xs text-zinc-400">mm</span>
          </div>
        </label>
        <div className="pb-1 text-right">
          <div className="text-[10px] uppercase tracking-wider text-zinc-400">Actual</div>
          <div className="font-mono text-xl tabular-nums">{currentLabel}<span className="ml-1 text-xs text-zinc-400">mm</span></div>
        </div>
      </div>
      <input type="range" min="0" max="20" step="0.1"
        value={Math.max(0,Math.min(20,Number(targetMm)||0))}
        disabled={busy!==null}
        onChange={(event)=>controller.setTargetMm(event.target.value)}
        className="range-control w-full" aria-label="Target PGEA opening"/>
      <div className="flex justify-between font-mono text-[10px] text-zinc-400">
        <span>Open · 0 mm</span><span>Half · 10 mm</span><span>Close · 20 mm</span>
      </div>
      <div className="flex flex-wrap gap-2">
        <Button size="sm" onClick={()=>void controller.move(Number(targetMm))}
          disabled={!controller.canMove||!targetValid}>
          {busy==="move"?<RefreshCw size={13} className="animate-spin"/>:<MoveHorizontal size={13}/>} Move
        </Button>
        <Button size="sm" variant="outline" onClick={()=>{controller.setTargetMm("0");void controller.move(0);}}
          disabled={!controller.canMove}><Hand size={13}/> Open</Button>
        <Button size="sm" variant="outline" onClick={()=>{controller.setTargetMm("20");void controller.move(20);}}
          disabled={!controller.canMove}><Hand size={13}/> Close</Button>
        <div className="flex-1"/>
        <Button size="sm" variant="destructive" onClick={()=>void controller.stop()} disabled={busy!=="move"}>
          <Square size={12} fill="currentColor"/> Stop
        </Button>
      </div>
      {!targetValid&&<p className="text-xs text-red-700">Enter a finite target between 0 and 20 mm per finger.</p>}
      <div className="rounded-md bg-zinc-50 px-3 py-2.5 text-xs leading-5 text-zinc-600">{readiness}</div>
      <div className="rounded-md border border-zinc-200 bg-zinc-50 px-3 py-2.5 text-xs leading-5 text-zinc-600">
        No calibrated conversion exists between Newtons and the PGEA force register. Commands keep <code>max_effort = 0</code> and use the gripper node's configured force percentage.
      </div>
      <div className="text-right text-[11px]">
        <Link to="/gripper" className="font-medium text-zinc-600 underline decoration-zinc-300 underline-offset-2 hover:text-zinc-950">
          Open full gripper panel
        </Link>
      </div>
    </div>
  </section>;
}
