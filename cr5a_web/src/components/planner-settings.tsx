import {useEffect,useMemo,useState} from "react";
import {ChevronDown,Copy,FileJson,Save,Trash2,Info,SlidersHorizontal,Play,Columns2} from "lucide-react";
import {Button} from "@/components/ui/button";
import {
  DEFAULT_PLANNER_PROFILE,DEFAULT_PLANNER_PROFILES,
  type PlannerProfile,type PlanningPipeline
} from "@/lib/planner-profiles";
import {describeMoveItError} from "@/lib/moveit-errors";

type HostResources={logical_cpus:number;memory_total_bytes:number;memory_available_bytes:number;max_planning_threads:number;recommended_planning_threads:number;memory_pressure:boolean};
export type PlannerBenchmarkResult={
  profile:PlannerProfile;status:string;targetType:"pose"|"joints";successfulRuns:number;totalRuns:number;
  planningTime:number|null;trajectoryDuration:number|null;trajectoryPoints:number;
  jointTravel:number|null;errorCode:number|null;error?:string;plan:any;color:string
};
const STORAGE_KEY="cr5a.planner-profiles.v1";
const ACTIVE_KEY="cr5a.planner-profile-selected.v1";
const FIELD="block w-full rounded-md border border-zinc-200 bg-white px-2.5 py-2 text-xs text-zinc-800 outline-none focus:border-zinc-500";
const LABEL="mb-1.5 flex items-center gap-1 text-[10px] font-semibold uppercase tracking-wider text-zinc-500";
const pipelineLabel=(value:PlanningPipeline)=>value==="ompl"?"OMPL":value==="pilz_industrial_motion_planner"?"Pilz":"CHOMP";
function bytesLabel(value:number){return !Number.isFinite(value)||value<=0?"—":(value/(1024**3)).toFixed(1)+" GiB";}
function isPlannerProfile(value:any):value is PlannerProfile{
  return value&&typeof value==="object"&&typeof value.id==="string"&&typeof value.name==="string"&&
    ["ompl","pilz_industrial_motion_planner","chomp"].includes(value.pipeline_id)&&typeof value.planner_id==="string"&&
    Number.isFinite(value.allowed_planning_time)&&Number.isInteger(value.num_planning_attempts)&&
    Number.isFinite(value.max_velocity_scaling)&&Number.isFinite(value.max_acceleration_scaling)&&
    Number.isInteger(value.max_planning_threads)&&value.planner_parameters&&typeof value.planner_parameters==="object"&&!Array.isArray(value.planner_parameters);
}
function loadSavedProfiles():PlannerProfile[]{try{const raw=window.localStorage.getItem(STORAGE_KEY);const value=raw?JSON.parse(raw):[];return Array.isArray(value)?value.filter(isPlannerProfile):[];}catch{return [];}}
function validateProfile(value:any):PlannerProfile{
  if(!value||typeof value!=="object"||Array.isArray(value))throw new Error("Profile JSON must be an object.");
  const options:Record<PlanningPipeline,string[]>={ompl:["RRTConnect","RRTstar","PRMstar"],pilz_industrial_motion_planner:["PTP","LIN"],chomp:[""]};
  if(!options[value.pipeline_id as PlanningPipeline]||!options[value.pipeline_id as PlanningPipeline].includes(value.planner_id))throw new Error("Invalid pipeline or planner.");
  const number=(key:string,min:number,max:number)=>{const n=Number(value[key]);if(!Number.isFinite(n)||n<min||n>max)throw new Error(key+" must be between "+min+" and "+max+".");return n;};
  const params=value.planner_parameters??{};
  if(!params||typeof params!=="object"||Array.isArray(params))throw new Error("planner_parameters must be an object.");
  if(value.pipeline_id!=="ompl"&&Object.keys(params).length)throw new Error("Planner parameters are supported only for OMPL.");
  for(const [key,item] of Object.entries(params)){if(!/^[A-Za-z0-9_]{1,64}$/.test(key)||!["number","boolean","string"].includes(typeof item)||(typeof item==="number"&&!Number.isFinite(item)))throw new Error("Invalid planner parameter: "+key);}
  const attempts=number("num_planning_attempts",1,20),threads=number("max_planning_threads",1,4);
  if(!Number.isInteger(attempts)||!Number.isInteger(threads))throw new Error("Attempts and threads must be whole numbers.");
  return {id:typeof value.id==="string"?value.id:"custom",name:String(value.name||"Custom profile").slice(0,48),
    description:typeof value.description==="string"?value.description.slice(0,140):"Custom profile",
    pipeline_id:value.pipeline_id,planner_id:value.planner_id,allowed_planning_time:number("allowed_planning_time",0.1,60),
    num_planning_attempts:attempts,max_velocity_scaling:number("max_velocity_scaling",0.01,1),
    max_acceleration_scaling:number("max_acceleration_scaling",0.01,1),max_planning_threads:threads,planner_parameters:params};
}
function Hint({text}:{text:string}){return <span title={text} aria-label={text} role="img" tabIndex={0} className="inline-flex cursor-help text-zinc-400 focus:outline-none focus:ring-2 focus:ring-zinc-400 rounded-full"><Info size={11}/></span>;}
function MetricLabel({children,hint}:{children:React.ReactNode;hint:string}){return <span className={LABEL}>{children}<Hint text={hint}/></span>;}

export function PlannerSettings({
  onProfileChange,disabled,onBenchmark,benchmarking,benchmarkResults,benchmarkDisabledReason,onUseBenchmarkPlan,benchmarkTargetType,
  benchmarkPathMode,onBenchmarkPathModeChange
}:{onProfileChange:(profile:PlannerProfile)=>void;disabled:boolean;
  onBenchmark:(profiles:PlannerProfile[],repeatCount:1|3|5)=>void;benchmarking:boolean;
  benchmarkResults:PlannerBenchmarkResult[];benchmarkDisabledReason:string|null;
  onUseBenchmarkPlan:(result:PlannerBenchmarkResult)=>void;benchmarkTargetType:"pose"|"joints";
  benchmarkPathMode:"representative"|"all";onBenchmarkPathModeChange:(mode:"representative"|"all")=>void;
}){
  const [profile,setProfile]=useState<PlannerProfile>(DEFAULT_PLANNER_PROFILE);
  const [customProfiles,setCustomProfiles]=useState<PlannerProfile[]>([]);
  const [resources,setResources]=useState<HostResources|null>(null);
  const [resourceError,setResourceError]=useState("");
  const [jsonOpen,setJsonOpen]=useState(false);
  const [jsonDraft,setJsonDraft]=useState("");
  const [plannerParamsDraft,setPlannerParamsDraft]=useState("{}");
  const [message,setMessage]=useState("");
  const [messageError,setMessageError]=useState(false);
  const [profileName,setProfileName]=useState(profile.name);
  const [selectedForCompare,setSelectedForCompare]=useState<string[]>(["balanced","quality"]);
  const [repeatCount,setRepeatCount]=useState<1|3|5>(3);
  const [initialized,setInitialized]=useState(false);
  useEffect(()=>{
    const saved=loadSavedProfiles();setCustomProfiles(saved);
    const activeId=window.localStorage.getItem(ACTIVE_KEY);
    const active=[...DEFAULT_PLANNER_PROFILES,...saved].find(item=>item.id===activeId);
    if(active)setProfile(active);
    setInitialized(true);
    const token=import.meta.env.VITE_BRIDGE_TOKEN as string|undefined;
    fetch("/api/v1/system/resources",{headers:token?{Authorization:"Bearer "+token}:undefined})
      .then(async response=>{if(!response.ok)throw new Error("Bridge resource endpoint returned HTTP "+response.status);return response.json();})
      .then(setResources).catch(error=>setResourceError(error instanceof Error?error.message:String(error)));
  },[]);
  useEffect(()=>{onProfileChange(profile);},[profile,onProfileChange]);
  useEffect(()=>{try{window.localStorage.setItem(STORAGE_KEY,JSON.stringify(customProfiles));}catch{}},[customProfiles]);
  useEffect(()=>{if(initialized)try{window.localStorage.setItem(ACTIVE_KEY,profile.id);}catch{}},[profile.id,initialized]);
  useEffect(()=>{
    setJsonDraft(JSON.stringify({name:profile.name,description:profile.description,pipeline_id:profile.pipeline_id,planner_id:profile.planner_id,
      allowed_planning_time:profile.allowed_planning_time,num_planning_attempts:profile.num_planning_attempts,
      max_velocity_scaling:profile.max_velocity_scaling,max_acceleration_scaling:profile.max_acceleration_scaling,
      max_planning_threads:profile.max_planning_threads,planner_parameters:profile.planner_parameters},null,2));
    setProfileName(profile.name);setPlannerParamsDraft(JSON.stringify(profile.planner_parameters,null,2));
  },[profile]);
  const allProfiles=useMemo(()=>{const profiles=[...DEFAULT_PLANNER_PROFILES,...customProfiles];return profiles.some(item=>item.id===profile.id)?profiles:[...profiles,profile];},[customProfiles,profile]);
  useEffect(()=>{setSelectedForCompare(current=>current.filter(id=>allProfiles.some(item=>item.id===id)));},[allProfiles]);
  const comparedProfiles=selectedForCompare.map(id=>id===profile.id?profile:allProfiles.find(item=>item.id===id)).filter((item):item is PlannerProfile=>Boolean(item));
  const sameOmplParameters=comparedProfiles.every(item=>JSON.stringify(item.planner_parameters)===JSON.stringify(comparedProfiles[0]?.planner_parameters));
  const hasUnsupportedChomp=benchmarkTargetType==="pose"&&comparedProfiles.some(item=>item.pipeline_id==="chomp");
  const compareReason=comparedProfiles.length<2?"Select at least two profiles (up to eight per comparison).":hasUnsupportedChomp
    ?"CHOMP accepts joint-space goals only. Switch to the Joints target tab or unselect CHOMP for a TCP pose comparison.":comparedProfiles.some(item=>item.pipeline_id==="ompl")&&!sameOmplParameters
    ?"OMPL planner overrides are merged by MoveIt and cannot be safely reset between runs. Compare profiles with identical OMPL parameter overrides.":benchmarkDisabledReason;
  const threadLimit=resources?.max_planning_threads??4;
  const threadChoices=Array.from(new Set([...Array.from({length:threadLimit},(_,index)=>index+1),profile.max_planning_threads])).sort((a,b)=>a-b);
  const plannerOptions:Record<PlanningPipeline,{id:string;label:string}[]>={ompl:[
    {id:"RRTConnect",label:"RRTConnect · fast feasible path"},{id:"RRTstar",label:"RRTstar · path optimization"},{id:"PRMstar",label:"PRMstar · roadmap"}],
    pilz_industrial_motion_planner:[{id:"PTP",label:"PTP · point-to-point"},{id:"LIN",label:"LIN · straight TCP line"}],chomp:[{id:"",label:"CHOMP default"}]};
  const update=(patch:Partial<PlannerProfile>)=>setProfile(current=>({...current,...patch}));
  const chooseProfile=(id:string)=>{const selected=allProfiles.find(item=>item.id===id);if(selected){setProfile({...selected});setMessage("");}};
  const saveProfile=()=>{
    const isBuiltin=DEFAULT_PLANNER_PROFILES.some(item=>item.id===profile.id);
    const id=isBuiltin?"custom-"+Date.now().toString(36):profile.id;
    const saved={...profile,id,name:profileName.trim()||"Custom profile"};
    setCustomProfiles(current=>current.some(item=>item.id===id)?current.map(item=>item.id===id?saved:item):[...current,saved]);
    setProfile(saved);setMessage("Profile saved in this browser.");setMessageError(false);
  };
  const deleteProfile=()=>{if(DEFAULT_PLANNER_PROFILES.some(item=>item.id===profile.id))return;setCustomProfiles(current=>current.filter(item=>item.id!==profile.id));setProfile(DEFAULT_PLANNER_PROFILE);setMessage("Custom profile deleted.");setMessageError(false);};
  const applyJson=()=>{try{const parsed=validateProfile(JSON.parse(jsonDraft));const saved={...parsed,id:"import-"+Date.now().toString(36)};setProfile(saved);setMessage("JSON profile loaded. Save it to keep it in this browser.");setMessageError(false);}catch(error){setMessage(error instanceof Error?error.message:"Invalid profile JSON.");setMessageError(true);}};
  const copyJson=async()=>{try{await navigator.clipboard.writeText(jsonDraft);setMessage("Profile JSON copied.");setMessageError(false);}catch{setMessage("Clipboard unavailable; select and copy the JSON manually.");setMessageError(true);}};
  const copyThreadsYaml=async()=>{const snippet="  max_planning_threads: "+profile.max_planning_threads+"\n";try{await navigator.clipboard.writeText(snippet);setMessage("Replace max_planning_threads under ompl: in ompl_planning.yaml, then restart MoveGroup.");setMessageError(false);}catch{setMessage(snippet.trim()+" (apply under ompl: and restart MoveGroup)");setMessageError(true);}};
  const selectedIsBuiltin=DEFAULT_PLANNER_PROFILES.some(item=>item.id===profile.id);
  const isHighThreadChoice=profile.pipeline_id==="ompl"&&profile.max_planning_threads>(resources?.recommended_planning_threads??1);
  const toggleCompared=(id:string)=>setSelectedForCompare(current=>current.includes(id)?current.filter(item=>item!==id):current.length<8?[...current,id]:current);
  const input=(label:string,hint:string,key:keyof PlannerProfile,min:number,max:number,step:number)=><label className="block min-w-0"><MetricLabel hint={hint}>{label}</MetricLabel><input className={FIELD} type="number" min={min} max={max} step={step} value={profile[key] as number} disabled={disabled} onChange={event=>update({[key]:Number(event.target.value)} as Partial<PlannerProfile>)}/></label>;

  return <section className="panel overflow-hidden">
    <div className="space-y-3 p-3">
      <div className="flex items-center justify-between gap-2">
        <div className="flex min-w-0 items-center gap-2"><SlidersHorizontal size={15} className="shrink-0 text-zinc-500"/><div className="min-w-0"><div className="text-sm font-semibold">Planning profile</div><div className="truncate text-[11px] text-zinc-500">Applied to the next planning request</div></div></div>
        <span className="rounded-md bg-zinc-100 px-2 py-1 text-[10px] font-medium text-zinc-600">{pipelineLabel(profile.pipeline_id)} · {profile.planner_id||"Default"}</span>
      </div>
      <div className="grid grid-cols-[minmax(0,1fr)_auto] items-end gap-2">
        <label className="min-w-0"><span className={LABEL}>Selected profile</span><select className={FIELD} value={profile.id} onChange={event=>chooseProfile(event.target.value)} disabled={disabled}>{allProfiles.map(item=><option key={item.id} value={item.id}>{item.name}{item.id.startsWith("custom-")||item.id.startsWith("import-")?" · custom":""}</option>)}</select></label>
        <Button size="sm" variant="outline" title="Save the current settings as a reusable profile" onClick={saveProfile} disabled={disabled||!profileName.trim()}><Save size={12}/> Save</Button>
      </div>
      <div className="flex items-center justify-between gap-2 text-[11px]">
        <span className="min-w-0 truncate text-zinc-500">{profile.description}</span>
        {!selectedIsBuiltin&&<button type="button" title="Delete this custom profile" className="shrink-0 text-red-600 hover:underline" onClick={deleteProfile} disabled={disabled}>Delete</button>}
      </div>
      <details className="rounded-md border border-zinc-200">
        <summary className="flex cursor-pointer list-none items-center justify-between gap-2 px-3 py-2.5 text-xs font-semibold text-zinc-700"><span>Planner settings <span className="ml-1 font-normal text-zinc-400">Advanced settings collapsed</span></span><ChevronDown size={14} className="text-zinc-400"/></summary>
        <div className="space-y-3 border-t border-zinc-100 p-3">
          <div className="grid grid-cols-2 gap-3">
            <label className="block min-w-0"><MetricLabel hint="Planning pipeline family selected by MoveIt. Different pipelines use different planning algorithms.">Pipeline</MetricLabel><select className={FIELD} value={profile.pipeline_id} disabled={disabled} onChange={event=>{const pipeline=event.target.value as PlanningPipeline;update({pipeline_id:pipeline,planner_id:plannerOptions[pipeline][0].id,planner_parameters:{}});}}><option value="ompl">OMPL</option><option value="pilz_industrial_motion_planner">Pilz</option><option value="chomp">CHOMP</option></select></label>
            <label className="block min-w-0"><MetricLabel hint="Specific planner implementation within the chosen pipeline.">Planner</MetricLabel><select className={FIELD} value={profile.planner_id} disabled={disabled} onChange={event=>update({planner_id:event.target.value,planner_parameters:{}})}>{plannerOptions[profile.pipeline_id].map(item=><option key={item.id||"default"} value={item.id}>{item.label}</option>)}</select></label>
            {input("Planning time","Maximum time in seconds MoveIt may spend finding a trajectory. Longer budgets can improve success but increase wait time.","allowed_planning_time",0.1,60,0.5)}
            {input("Attempts","Number of planning attempts. More attempts can improve the chance of finding a path, at the cost of more computation.","num_planning_attempts",1,20,1)}
            {input("Velocity scaling","Fraction of the configured joint velocity limits, from 1% to 100%. This is not a speed in mm/s.","max_velocity_scaling",0.01,1,0.05)}
            {input("Acceleration scaling","Fraction of configured joint acceleration limits, from 1% to 100%.","max_acceleration_scaling",0.01,1,0.05)}
          </div>
          <details className="rounded-md bg-zinc-50 p-3">
            <summary className="cursor-pointer text-xs font-semibold text-zinc-700">OMPL resources & planner parameters</summary>
            <div className="mt-3 space-y-3">
              <div className="flex items-center justify-between gap-3"><div><div className="text-xs font-medium">Parallel planning threads <Hint text="MoveGroup startup setting, not a per-request value. Changing it requires restarting MoveGroup."/></div><div className="text-[10px] text-zinc-500">{resources?resources.logical_cpus+" logical CPUs · "+bytesLabel(resources.memory_total_bytes)+" RAM":"Reading host resources…"}</div></div><select className="w-24 rounded-md border border-zinc-200 bg-white px-2 py-1.5 text-xs" value={profile.max_planning_threads} disabled={disabled||profile.pipeline_id!=="ompl"} onChange={event=>update({max_planning_threads:Number(event.target.value)})}>{threadChoices.map(value=><option key={value} value={value}>{value}</option>)}</select></div>
              {resources&&<div className="flex items-center justify-between text-[10px] text-zinc-500"><span>{bytesLabel(resources.memory_available_bytes)} available</span><span>Suggested: {resources.recommended_planning_threads}</span></div>}
              {resources?.memory_pressure&&<p className="text-[11px] text-amber-800">Memory pressure detected. Prefer one or two OMPL threads.</p>}
              {resourceError&&<p className="text-[10px] text-amber-700">Host resources unavailable: {resourceError}</p>}
              <Button variant="outline" size="sm" onClick={copyThreadsYaml} title="Copy MoveGroup startup YAML setting"><Copy size={11}/> Copy thread YAML</Button>
              {isHighThreadChoice&&<p className="text-[10px] text-amber-700">This profile exceeds the resource-based recommendation. It is not applied until MoveGroup is restarted.</p>}
              {profile.pipeline_id==="ompl"&&<label className="block"><MetricLabel hint="OMPL key/value overrides sent through MoveIt's set_planner_params service. MoveIt merges these and they persist in MoveGroup until overwritten or restarted.">OMPL parameters (JSON)</MetricLabel><textarea className={FIELD+" min-h-20 resize-y font-mono text-[10px]"} spellCheck={false} value={plannerParamsDraft} disabled={disabled} onChange={event=>setPlannerParamsDraft(event.target.value)}/><Button className="mt-2" size="sm" variant="outline" disabled={disabled} onClick={()=>{try{const parsed=JSON.parse(plannerParamsDraft);if(!parsed||typeof parsed!=="object"||Array.isArray(parsed))throw new Error("Use a JSON object.");update({planner_parameters:parsed});setMessage("OMPL parameters updated in this profile.");setMessageError(false);}catch(error){setMessage(error instanceof Error?error.message:"Invalid JSON.");setMessageError(true);}}}>Apply parameters</Button></label>}
            </div>
          </details>
          <label className="block"><MetricLabel hint="Name used to identify this reusable profile in the profile selector and comparison table.">Profile name</MetricLabel><input className={FIELD} value={profileName} maxLength={48} disabled={disabled} onChange={event=>setProfileName(event.target.value)}/></label>
          <div className="flex flex-wrap gap-2"><Button size="sm" onClick={saveProfile} disabled={disabled||!profileName.trim()}><Save size={12}/> Save profile</Button><Button size="sm" variant="outline" onClick={()=>setJsonOpen(value=>!value)}><FileJson size={12}/> JSON import / export</Button></div>
          {jsonOpen&&<div className="space-y-2"><textarea className={FIELD+" min-h-40 resize-y font-mono text-[10px]"} spellCheck={false} value={jsonDraft} disabled={disabled} onChange={event=>setJsonDraft(event.target.value)}/><div className="flex gap-2"><Button size="sm" onClick={applyJson} disabled={disabled}>Import profile</Button><Button size="sm" variant="outline" onClick={copyJson}><Copy size={12}/> Copy JSON</Button></div></div>}
          {message&&<div className={"rounded-md px-3 py-2 text-[11px] "+(messageError?"bg-amber-50 text-amber-800":"bg-zinc-50 text-zinc-600")}>{message}</div>}
        </div>
      </details>
      <details className="rounded-md border border-zinc-200">
        <summary className="flex cursor-pointer list-none items-center justify-between gap-2 px-3 py-2.5 text-xs font-semibold text-zinc-700"><span className="flex items-center gap-2"><Columns2 size={13}/> Compare profiles <span className="font-normal text-zinc-400">({selectedForCompare.length}/8 selected)</span></span><ChevronDown size={14} className="text-zinc-400"/></summary>
        <div className="space-y-3 border-t border-zinc-100 p-3">
          <p className="text-[11px] leading-4 text-zinc-500">Select 2–8 profiles. The same target is planned repeatedly in alternating profile order. Benchmarking never executes robot motion. CHOMP supports joint-space targets only.</p>
          <div className="grid grid-cols-1 gap-1.5 sm:grid-cols-2">
            {allProfiles.map(item=>{const selected=selectedForCompare.includes(item.id);const chompPoseUnsupported=benchmarkTargetType==="pose"&&item.pipeline_id==="chomp";return <label key={item.id} className="flex min-w-0 items-center gap-2 rounded-md border border-zinc-100 px-2 py-1.5 text-xs"><input type="checkbox" checked={selected} disabled={disabled||benchmarking||(!selected&&(selectedForCompare.length>=8||chompPoseUnsupported))} onChange={()=>toggleCompared(item.id)}/><span className="min-w-0 flex-1 truncate">{item.name}</span><span className="shrink-0 text-[10px] text-zinc-400">{chompPoseUnsupported?"Joint goals only":item.planner_id||pipelineLabel(item.pipeline_id)}</span></label>;})}
          </div>
          {comparedProfiles.length>=2&&<div className="overflow-x-auto rounded-md border border-zinc-100"><table className="w-full min-w-[420px] text-left text-[10px]"><thead className="bg-zinc-50"><tr><th className="p-2">Setting</th>{comparedProfiles.map(item=><th key={item.id} className="p-2">{item.name}</th>)}</tr></thead><tbody>{([
            ["Pipeline",item=>pipelineLabel(item.pipeline_id)],["Planner",item=>item.planner_id||"Default"],["Time",item=>item.allowed_planning_time+" s"],["Attempts",item=>String(item.num_planning_attempts)],["Velocity",item=>Math.round(item.max_velocity_scaling*100)+"%"],["Acceleration",item=>Math.round(item.max_acceleration_scaling*100)+"%"],["Threads*",item=>String(item.max_planning_threads)],["OMPL overrides",item=>Object.keys(item.planner_parameters).length?JSON.stringify(item.planner_parameters):"Default"]
          ] as [string,(item:PlannerProfile)=>string][]).map(([name,get])=><tr key={name} className="border-t border-zinc-100"><td className="p-2 font-medium text-zinc-500">{name}</td>{comparedProfiles.map(item=><td key={item.id} className="p-2 font-mono">{get(item)}</td>)}</tr>)}</tbody></table><p className="px-2 py-1 text-[9px] text-zinc-400">* Thread count is a MoveGroup startup setting, not applied dynamically per benchmark.</p></div>}
          <div className="flex flex-wrap items-end justify-between gap-2">
            <label className="block min-w-0"><MetricLabel hint="Each profile is run this many times. Plan time and trajectory metrics use the median of successful runs.">Repeated runs</MetricLabel><select className={FIELD+" w-28"} value={repeatCount} disabled={disabled||benchmarking} onChange={event=>setRepeatCount(Number(event.target.value) as 1|3|5)}><option value={1}>1 run</option><option value={3}>3 runs</option><option value={5}>5 runs</option></select></label>
            <Button size="sm" onClick={()=>onBenchmark(comparedProfiles,repeatCount)} disabled={disabled||benchmarking||Boolean(compareReason)||comparedProfiles.length<2}><Play size={12} className={benchmarking?"animate-pulse":""}/>{benchmarking?"Benchmarking…":"Benchmark selected profiles"}</Button>
          </div>
          {compareReason&&<p className="text-[11px] text-amber-700">{compareReason}</p>}
          {benchmarkResults.length>0&&<div className="space-y-2 border-t border-zinc-100 pt-3">
            <div className="flex flex-wrap items-center justify-between gap-2">
              <div><div className="text-xs font-semibold">Benchmark results</div><div className="text-[10px] text-zinc-500">3D trajectory display</div></div>
              <div className="inline-flex rounded-md border border-zinc-200 bg-zinc-50 p-0.5" role="group" aria-label="Benchmark path display">
                <button type="button" aria-pressed={benchmarkPathMode==="representative"} disabled={benchmarking}
                  onClick={()=>onBenchmarkPathModeChange("representative")}
                  className={"rounded px-2.5 py-1.5 text-[10px] font-medium "+(benchmarkPathMode==="representative"?"bg-white text-zinc-900 shadow-sm":"text-zinc-500 hover:text-zinc-800")}>
                  Median run
                </button>
                <button type="button" aria-pressed={benchmarkPathMode==="all"} disabled={benchmarking}
                  onClick={()=>onBenchmarkPathModeChange("all")}
                  className={"rounded px-2.5 py-1.5 text-[10px] font-medium "+(benchmarkPathMode==="all"?"bg-white text-zinc-900 shadow-sm":"text-zinc-500 hover:text-zinc-800")}>
                  All successful runs
                </button>
              </div>
            </div>
            <div className="overflow-x-auto"><table className="w-full min-w-[420px] text-left text-[10px]"><thead className="bg-zinc-50 text-zinc-500"><tr><th className="p-2">Profile</th><th className="p-2">Result</th><th className="p-2">Plan time</th><th className="p-2">Duration</th><th className="p-2">Points</th><th className="p-2" title="Sum of waypoint-to-waypoint joint-space distances; not Cartesian TCP distance.">Joint travel</th><th className="p-2"></th></tr></thead><tbody>{benchmarkResults.map(result=><tr key={result.profile.id} className="border-t border-zinc-100"><td className="p-2"><span className="mr-1 inline-block h-2 w-2 rounded-full" style={{backgroundColor:result.color}}/>{result.profile.name}</td><td className="p-2 align-top"><div>{result.status} · {result.successfulRuns}/{result.totalRuns}{result.errorCode!==null?" ("+result.errorCode+")":""}</div>{result.error&&<p className="mt-1 max-w-56 whitespace-normal text-[9px] leading-4 text-amber-800">{result.error}</p>}{result.errorCode!==null&&<p className="mt-1 max-w-56 whitespace-normal text-[9px] leading-4 text-zinc-500">{describeMoveItError(result.errorCode,{pipeline:result.profile.pipeline_id,targetType:result.targetType})}</p>}</td><td className="p-2 font-mono">{result.planningTime===null?"—":result.planningTime.toFixed(3)+" s median"}</td><td className="p-2 font-mono">{result.trajectoryDuration===null?"—":result.trajectoryDuration.toFixed(2)+" s median"}</td><td className="p-2 font-mono">{result.trajectoryPoints||"—"}</td><td className="p-2 font-mono" title="Euclidean joint-space travel accumulated across trajectory points. Not a Cartesian TCP path length.">{result.jointTravel===null?"—":result.jointTravel.toFixed(3)+" rad median"}</td><td className="p-2">{result.successfulRuns>0&&result.plan?.plan_id&&<button type="button" className="font-medium text-blue-700 hover:underline" onClick={()=>onUseBenchmarkPlan(result)} title="Make the representative median-time successful run active, ready for inspection and explicit execution">Use</button>}</td></tr>)}</tbody></table></div><p className="text-[10px] text-zinc-500">Metrics show medians across successful runs. Median run displays one representative trajectory per planner; All successful runs displays every successful repeat. Select Use to make the representative plan active; execution remains a separate action.</p></div>}
        </div>
      </details>
    </div>
  </section>;
}
