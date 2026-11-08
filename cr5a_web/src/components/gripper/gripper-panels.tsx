import {AlertCircle,Check,CopyPlus,Hand,MoveHorizontal,Power,RefreshCw,Save,SlidersHorizontal,Square,Trash2,Wrench} from "lucide-react";
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

export function GripperPositionCard({controller,disabled=false}:{controller:GripperController;disabled?:boolean}){
  const {
    gripper,busy,currentLabel,targetMm,targetValid,readiness,profiles,selectedProfile,
    selectedProfileId,profileName,forcePercent,speedPercent,profileValid,
    profileBusy,profileDraftDirty
  }=controller;
  const resetProfileDraft=controller.resetProfileDraft;
  const active=profiles?.active_profile_id===selectedProfileId;
  return <section className="panel overflow-hidden">
    <div className="panel-header">
      <div>
        <div className="text-sm font-semibold">Gripper controls</div>
        <div className="mt-0.5 text-xs text-zinc-500">PGEA-100-40-W · 0–20 mm per finger</div>
      </div>
      <div className="flex items-center gap-2">
        <StatusBadge state={!gripper?.connected?"disconnected":busy==="move"?"moving":"idle"}
          label={!gripper?.connected?"Offline":gripper.simulation?"Simulation":"Connected"} compact/>
        <MoveHorizontal size={16} className="text-zinc-400"/>
      </div>
    </div>
    <div className="space-y-4 px-4 py-4">
      <Notice controller={controller}/>
      <div className="flex items-end justify-between gap-4">
        <label className="block min-w-0 flex-1">
          <span className="mb-1.5 block text-[11px] font-medium uppercase tracking-wider text-zinc-500">Target opening · per finger</span>
          <div className="flex h-10 items-center rounded-md border border-zinc-200 bg-white focus-within:border-zinc-500">
            <input type="number" min="0" max="20" step="0.1" value={targetMm}
              disabled={disabled||busy!==null}
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
        disabled={disabled||busy!==null}
        onChange={(event)=>controller.setTargetMm(event.target.value)}
        className="range-control w-full" aria-label="Target PGEA opening"/>
      <div className="flex justify-between font-mono text-[10px] text-zinc-400">
        <span>Open · 0 mm</span><span>Half · 10 mm</span><span>Close · 20 mm</span>
      </div>
      <div className="flex flex-wrap gap-2">
        <Button size="sm" onClick={()=>void controller.move(Number(targetMm))}
          disabled={disabled||!controller.canMove||!targetValid}>
          {busy==="move"?<RefreshCw size={13} className="animate-spin"/>:<MoveHorizontal size={13}/>} Move
        </Button>
        <Button size="sm" variant="outline" onClick={()=>{controller.setTargetMm("0");void controller.move(0);}}
          disabled={disabled||!controller.canMove}><Hand size={13}/> Open</Button>
        <Button size="sm" variant="outline" onClick={()=>{controller.setTargetMm("20");void controller.move(20);}}
          disabled={disabled||!controller.canMove}><Hand size={13}/> Close</Button>
        <div className="flex-1"/>
        <Button size="sm" variant="destructive" onClick={()=>void controller.stop()} disabled={busy!=="move"}>
          <Square size={12} fill="currentColor"/> Stop
        </Button>
      </div>
      {!targetValid&&<p className="text-xs text-red-700">Enter a finite target between 0 and 20 mm per finger.</p>}

      <details className="rounded-md border border-zinc-200">
        <summary className="flex cursor-pointer list-none items-center justify-between border-b border-zinc-100 px-3 py-2.5">
          <div className="flex items-center gap-2 text-xs font-semibold text-zinc-800">
            <SlidersHorizontal size={14}/> Force &amp; speed profiles
          </div>
          <span className="flex items-center gap-2">{active&&<StatusBadge state="idle" label="Active"/>}<span className="text-[10px] font-normal text-zinc-400">Advanced</span></span>
        </summary>
        <div className="space-y-3 p-3">
          <label className="block">
            <span className="mb-1 block text-[11px] font-medium text-zinc-500">Profile</span>
            <select value={selectedProfileId} onChange={(event)=>controller.selectProfile(event.target.value)}
              disabled={!profiles||profileBusy}
              className="h-9 w-full rounded-md border border-zinc-200 bg-white px-2 text-sm">
              {(profiles?.profiles??[]).map((profile)=><option key={profile.id} value={profile.id}>
                {profile.name}{profile.id===profiles?.active_profile_id?" · Active":""}
              </option>)}
            </select>
          </label>
          <label className="block">
            <span className="mb-1 block text-[11px] font-medium text-zinc-500">Profile name</span>
            <input value={profileName} maxLength={48} onChange={(event)=>controller.updateProfileDraft({name:event.target.value})}
              disabled={profileBusy} className="h-9 w-full rounded-md border border-zinc-200 px-2 text-sm"/>
          </label>
          <div className="grid grid-cols-2 gap-3">
            <label className="block">
              <span title="PGEA force register percentage, not force in Newtons. The device-specific configured range is 20–100%." className="flex items-center justify-between text-[11px] font-medium text-zinc-500">
                Force <strong className="font-mono text-zinc-800">{forcePercent}%</strong>
              </span>
              <input type="range" min="20" max="100" step="1" value={Math.max(20,Math.min(100,Number(forcePercent)||20))}
                disabled={profileBusy} onChange={(event)=>controller.updateProfileDraft({force:event.target.value})}
                className="range-control mt-2 w-full" aria-label="PGEA force percentage"/>
              <input type="number" min="20" max="100" step="1" value={forcePercent}
                disabled={profileBusy} onChange={(event)=>controller.updateProfileDraft({force:event.target.value})}
                className="mt-1 h-8 w-full rounded-md border border-zinc-200 px-2 font-mono text-sm"/>
              <span className="mt-1 block text-[10px] text-zinc-400">20–100% · PGEA register</span>
            </label>
            <label className="block">
              <span title="PGEA speed register percentage (1–100%). Higher values command faster gripper motion." className="flex items-center justify-between text-[11px] font-medium text-zinc-500">
                Speed <strong className="font-mono text-zinc-800">{speedPercent}%</strong>
              </span>
              <input type="range" min="1" max="100" step="1" value={Math.max(1,Math.min(100,Number(speedPercent)||1))}
                disabled={profileBusy} onChange={(event)=>controller.updateProfileDraft({speed:event.target.value})}
                className="range-control mt-2 w-full" aria-label="PGEA speed percentage"/>
              <input type="number" min="1" max="100" step="1" value={speedPercent}
                disabled={profileBusy} onChange={(event)=>controller.updateProfileDraft({speed:event.target.value})}
                className="mt-1 h-8 w-full rounded-md border border-zinc-200 px-2 font-mono text-sm"/>
              <span className="mt-1 block text-[10px] text-zinc-400">1–100% · PGEA register</span>
            </label>
          </div>
          {!profileValid&&<p className="text-xs text-red-700">Check profile name, force (20–100), and speed (1–100). Values must be whole numbers.</p>}
          <div className="flex flex-wrap gap-2">
            <Button size="sm" onClick={()=>void controller.applyProfile()}
              disabled={!controller.canControl||!selectedProfile||profileDraftDirty||active||profileBusy}>
              {busy==="profile-apply"?<RefreshCw size={13} className="animate-spin"/>:<Check size={13}/>} Apply
            </Button>
            <Button size="sm" variant="outline" onClick={()=>void controller.saveProfile(false)}
              disabled={!profileValid||!profileDraftDirty||profileBusy}>
              {busy==="profile-save"?<RefreshCw size={13} className="animate-spin"/>:<Save size={13}/>} Save changes
            </Button>
            {profileDraftDirty&&<Button size="sm" variant="ghost" onClick={resetProfileDraft} disabled={profileBusy}>
              Discard edits
            </Button>}
            <Button size="sm" variant="outline" onClick={()=>void controller.saveProfile(true)}
              disabled={!profileValid||profileBusy}>
              <CopyPlus size={13}/> Save as new
            </Button>
            <Button size="sm" variant="destructive" onClick={()=>void controller.deleteProfile()}
              disabled={!selectedProfile||active||profileBusy}>
              {busy==="profile-delete"?<RefreshCw size={13} className="animate-spin"/>:<Trash2 size={13}/>} Delete
            </Button>
          </div>
          <div className="text-[11px] leading-4 text-zinc-500">
            {profiles?.runtime_synced
              ?"Runtime settings are synchronized with the PGEA node."
              :profiles?.runtime_error??"Waiting for the PGEA node to accept the active profile."}
          </div>
          {profiles?.storage_path&&<div className="break-all border-t border-zinc-100 pt-2 font-mono text-[10px] text-zinc-400">
            Saved on bridge host: {profiles.storage_path}
          </div>}
        </div>
      </details>

      <div className="rounded-md bg-zinc-50 px-3 py-2.5 text-xs leading-5 text-zinc-600">{readiness}</div>
      <div className="rounded-md border border-zinc-200 bg-zinc-50 px-3 py-2.5 text-xs leading-5 text-zinc-600">
        Force is PGEA percentage, not Newtons. Profiles apply to the standard Move/Open/Close position action. No calibrated conversion exists between this register and physical force in N.
      </div>
      <div className="text-right text-[11px]">
        <Link to="/gripper" className="font-medium text-zinc-600 underline decoration-zinc-300 underline-offset-2 hover:text-zinc-950">
          Open full gripper panel
        </Link>
      </div>
    </div>
  </section>;
}
