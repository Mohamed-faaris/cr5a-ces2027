import {useEffect,useMemo,useState} from "react";
import {Copy,FileJson,Save,Trash2,SlidersHorizontal} from "lucide-react";
import {Button} from "@/components/ui/button";
import {
  DEFAULT_PLANNER_PROFILE,DEFAULT_PLANNER_PROFILES,
  type PlannerProfile,type PlanningPipeline
} from "@/lib/planner-profiles";

type HostResources={
  logical_cpus:number;
  memory_total_bytes:number;
  memory_available_bytes:number;
  swap_total_bytes:number;
  swap_free_bytes:number;
  max_planning_threads:number;
  recommended_planning_threads:number;
  memory_pressure:boolean;
};

const STORAGE_KEY="cr5a.planner-profiles.v1";
const FIELD="block w-full rounded-md border border-zinc-200 bg-white px-2.5 py-2 text-xs text-zinc-800 outline-none focus:border-zinc-500";
const LABEL="mb-1.5 block text-[10px] font-semibold uppercase tracking-wider text-zinc-500";

function bytesLabel(value:number){
  if(!Number.isFinite(value)||value<=0)return "—";
  return (value/(1024**3)).toFixed(1)+" GiB";
}
function loadSavedProfiles():PlannerProfile[]{
  try{
    const raw=window.localStorage.getItem(STORAGE_KEY);
    if(!raw)return [];
    const value=JSON.parse(raw);
    return Array.isArray(value)?value.filter(isPlannerProfile):[];
  }catch{return [];}
}
function isPlannerProfile(value:any):value is PlannerProfile{
  return value&&typeof value==="object"&&
    typeof value.id==="string"&&typeof value.name==="string"&&
    ["ompl","pilz_industrial_motion_planner","chomp"].includes(value.pipeline_id)&&
    typeof value.planner_id==="string"&&
    Number.isFinite(value.allowed_planning_time)&&
    Number.isInteger(value.num_planning_attempts)&&
    Number.isFinite(value.max_velocity_scaling)&&
    Number.isFinite(value.max_acceleration_scaling)&&
    Number.isInteger(value.max_planning_threads)&&
    value.planner_parameters&&typeof value.planner_parameters==="object"&&!Array.isArray(value.planner_parameters);
}
function validateProfile(value:any):PlannerProfile{
  if(!value||typeof value!=="object"||Array.isArray(value))throw new Error("Profile JSON must be an object.");
  const required=["name","pipeline_id","planner_id","allowed_planning_time","num_planning_attempts","max_velocity_scaling","max_acceleration_scaling","max_planning_threads"];
  for(const key of required)if(!(key in value))throw new Error("Missing required field: "+key);
  if(!["ompl","pilz_industrial_motion_planner","chomp"].includes(value.pipeline_id))throw new Error("Unsupported pipeline_id.");
  const number=(key:string,min:number,max:number)=>{
    const candidate=Number(value[key]);
    if(!Number.isFinite(candidate)||candidate<min||candidate>max)throw new Error(key+" must be between "+min+" and "+max+".");
    return candidate;
  };
  const plannerOptions:Record<PlanningPipeline,string[]> = {
    ompl:["RRTConnect","RRTstar","PRMstar"],
    pilz_industrial_motion_planner:["PTP","LIN"],
    chomp:[""]
  };
  if(typeof value.planner_id!=="string"||!plannerOptions[value.pipeline_id as PlanningPipeline].includes(value.planner_id)){
    throw new Error("planner_id is not valid for the selected pipeline.");
  }
  const params=value.planner_parameters??{};
  if(!params||typeof params!=="object"||Array.isArray(params))throw new Error("planner_parameters must be an object.");
  if(value.pipeline_id!=="ompl"&&Object.keys(params).length>0)throw new Error("planner_parameters are currently supported only for OMPL.");
  for(const [key,item] of Object.entries(params)){
    if(!/^[A-Za-z0-9_]{1,64}$/.test(key))throw new Error("Invalid planner parameter name: "+key);
    if(typeof item==="number"&&!Number.isFinite(item))throw new Error("Planner parameter values must be finite.");
    if(typeof item==="string"&&item.length>64)throw new Error("Planner parameter strings are limited to 64 characters.");
    if(!["number","boolean","string"].includes(typeof item))throw new Error("Planner parameter values must be numbers, booleans, or strings.");
  }
  const attempts=number("num_planning_attempts",1,20);
  const threads=number("max_planning_threads",1,4);
  if(!Number.isInteger(attempts))throw new Error("num_planning_attempts must be an integer.");
  if(!Number.isInteger(threads))throw new Error("max_planning_threads must be an integer.");
  return {
    id:typeof value.id==="string"?value.id:"custom",
    name:String(value.name).slice(0,48),
    description:typeof value.description==="string"?value.description.slice(0,140):"Custom profile",
    pipeline_id:value.pipeline_id,
    planner_id:value.planner_id,
    allowed_planning_time:number("allowed_planning_time",0.1,60),
    num_planning_attempts:attempts,
    max_velocity_scaling:number("max_velocity_scaling",0.01,1),
    max_acceleration_scaling:number("max_acceleration_scaling",0.01,1),
    max_planning_threads:threads,
    planner_parameters:params
  };
}

export function PlannerSettings({onProfileChange,disabled}:{onProfileChange:(profile:PlannerProfile)=>void;disabled:boolean}){
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

  useEffect(()=>{
    setCustomProfiles(loadSavedProfiles());
    const token=import.meta.env.VITE_BRIDGE_TOKEN as string|undefined;
    fetch("/api/v1/system/resources",{
      headers:token?{Authorization:"Bearer "+token}:undefined
    }).then(async response=>{
      if(!response.ok)throw new Error("Bridge resource endpoint returned HTTP "+response.status);
      return response.json();
    }).then(setResources).catch(error=>setResourceError(error instanceof Error?error.message:String(error)));
  },[]);
  useEffect(()=>{onProfileChange(profile);},[profile,onProfileChange]);
  useEffect(()=>{
    try{window.localStorage.setItem(STORAGE_KEY,JSON.stringify(customProfiles));}catch{}
  },[customProfiles]);
  useEffect(()=>{
    setJsonDraft(JSON.stringify({
      name:profile.name,description:profile.description,
      pipeline_id:profile.pipeline_id,planner_id:profile.planner_id,
      allowed_planning_time:profile.allowed_planning_time,
      num_planning_attempts:profile.num_planning_attempts,
      max_velocity_scaling:profile.max_velocity_scaling,
      max_acceleration_scaling:profile.max_acceleration_scaling,
      max_planning_threads:profile.max_planning_threads,
      planner_parameters:profile.planner_parameters
    },null,2));
    setProfileName(profile.name);
    setPlannerParamsDraft(JSON.stringify(profile.planner_parameters,null,2));
  },[profile]);

  const allProfiles=useMemo(()=>{
    const profiles=[...DEFAULT_PLANNER_PROFILES,...customProfiles];
    return profiles.some(item=>item.id===profile.id)?profiles:[...profiles,profile];
  },[customProfiles,profile]);
  const threadLimit=resources?.max_planning_threads??4;
  const threadChoices=Array.from(
    new Set([...Array.from({length:threadLimit},(_,index)=>index+1),profile.max_planning_threads])
  ).sort((a,b)=>a-b);
  const plannerOptions:Record<PlanningPipeline,{id:string;label:string}[]> = {
    ompl:[
      {id:"RRTConnect",label:"RRTConnect · fast feasible path"},
      {id:"RRTstar",label:"RRTstar · path optimization"},
      {id:"PRMstar",label:"PRMstar · roadmap"}
    ],
    pilz_industrial_motion_planner:[
      {id:"PTP",label:"PTP · synchronized point-to-point"},
      {id:"LIN",label:"LIN · straight TCP line"}
    ],
    chomp:[{id:"",label:"CHOMP default"}]
  };
  const update=(patch:Partial<PlannerProfile>)=>setProfile(current=>({...current,...patch}));
  const chooseProfile=(id:string)=>{
    const selected=allProfiles.find(item=>item.id===id);
    if(selected)setProfile({...selected});
    setMessage("");
  };
  const saveProfile=()=>{
    const isBuiltin=DEFAULT_PLANNER_PROFILES.some(item=>item.id===profile.id);
    const id=isBuiltin?"custom-"+Date.now().toString(36):profile.id;
    const saved={...profile,id,name:profileName.trim()||"Custom profile"};
    setCustomProfiles(current=>current.some(item=>item.id===id)
      ?current.map(item=>item.id===id?saved:item)
      :[...current,saved]);
    setProfile(saved);
    setMessage("Profile saved in this browser.");
    setMessageError(false);
  };
  const deleteProfile=()=>{
    if(DEFAULT_PLANNER_PROFILES.some(item=>item.id===profile.id))return;
    setCustomProfiles(current=>current.filter(item=>item.id!==profile.id));
    setProfile(DEFAULT_PLANNER_PROFILE);
    setMessage("Custom profile deleted.");
    setMessageError(false);
  };
  const applyJson=()=>{
    try{
      const parsed=validateProfile(JSON.parse(jsonDraft));
      const saved={...parsed,id:"import-"+Date.now().toString(36)};
      setProfile(saved);
      setMessage("JSON profile loaded. Save it to keep it in this browser.");
      setMessageError(false);
    }catch(error){
      setMessage(error instanceof Error?error.message:"Invalid profile JSON.");
      setMessageError(true);
    }
  };
  const copyJson=async()=>{
    try{await navigator.clipboard.writeText(jsonDraft);setMessage("Profile JSON copied.");setMessageError(false);}
    catch{setMessage("Clipboard access is unavailable. Select and copy the JSON manually.");setMessageError(true);}
  };
  const copyThreadsYaml=async()=>{
    const snippet="  max_planning_threads: "+profile.max_planning_threads+"\n";
    try{await navigator.clipboard.writeText(snippet);setMessage("Replace the existing max_planning_threads line under ompl: in cr5_moveit/config/ompl_planning.yaml, then restart MoveGroup.");setMessageError(false);}
    catch{setMessage(snippet.trim()+" (replace the existing line under ompl: in cr5_moveit/config/ompl_planning.yaml, then restart MoveGroup)");setMessageError(true);}
  };
  const selectedIsBuiltin=DEFAULT_PLANNER_PROFILES.some(item=>item.id===profile.id);
  const availableRam=resources?.memory_available_bytes??0;
  const isHighThreadChoice=profile.pipeline_id==="ompl"&&profile.max_planning_threads>(resources?.recommended_planning_threads??1);

  return <section className="panel">
    <div className="panel-header">
      <div className="flex items-center gap-2">
        <SlidersHorizontal size={15} className="text-zinc-500"/>
        <div>
          <div className="text-sm font-semibold">Planner configuration</div>
          <div className="mt-0.5 text-[11px] text-zinc-500">Profiles apply to the next plan request</div>
        </div>
      </div>
      <span className="text-[10px] text-zinc-400">MoveIt</span>
    </div>
    <div className="space-y-4 px-4 py-4">
      <div>
        <label className={LABEL}>Profile</label>
        <select className={FIELD} value={profile.id} onChange={event=>chooseProfile(event.target.value)} disabled={disabled}>
          {allProfiles.map(item=><option key={item.id} value={item.id}>{item.name}{item.id.startsWith("custom-")||item.id.startsWith("import-")?" · custom":""}</option>)}
        </select>
        <p className="mt-1.5 text-[11px] leading-4 text-zinc-500">{profile.description}</p>
      </div>

      <div className="grid grid-cols-2 gap-3">
        <label>
          <span className={LABEL}>Pipeline</span>
          <select className={FIELD} value={profile.pipeline_id} disabled={disabled}
            onChange={event=>{
              const pipeline=event.target.value as PlanningPipeline;
              update({pipeline_id:pipeline,planner_id:plannerOptions[pipeline][0].id,planner_parameters:{}});
            }}>
            <option value="ompl">OMPL</option>
            <option value="pilz_industrial_motion_planner">Pilz</option>
            <option value="chomp">CHOMP</option>
          </select>
        </label>
        <label>
          <span className={LABEL}>Planner</span>
          <select className={FIELD} value={profile.planner_id} disabled={disabled}
            onChange={event=>update({planner_id:event.target.value,planner_parameters:{}})}>
            {plannerOptions[profile.pipeline_id].map(item=><option key={item.id||"default"} value={item.id}>{item.label}</option>)}
          </select>
        </label>
        <label>
          <span className={LABEL}>Planning time (s)</span>
          <input className={FIELD} type="number" min="0.1" max="60" step="0.5" value={profile.allowed_planning_time}
            disabled={disabled} onChange={event=>update({allowed_planning_time:Number(event.target.value)})}/>
        </label>
        <label>
          <span className={LABEL}>Attempts</span>
          <input className={FIELD} type="number" min="1" max="20" step="1" value={profile.num_planning_attempts}
            disabled={disabled} onChange={event=>update({num_planning_attempts:Number(event.target.value)})}/>
        </label>
        <label>
          <span className={LABEL}>Velocity scaling</span>
          <input className={FIELD} type="number" min="0.01" max="1" step="0.05" value={profile.max_velocity_scaling}
            disabled={disabled} onChange={event=>update({max_velocity_scaling:Number(event.target.value)})}/>
        </label>
        <label>
          <span className={LABEL}>Acceleration scaling</span>
          <input className={FIELD} type="number" min="0.01" max="1" step="0.05" value={profile.max_acceleration_scaling}
            disabled={disabled} onChange={event=>update({max_acceleration_scaling:Number(event.target.value)})}/>
        </label>
      </div>

      <div className="rounded-lg border border-zinc-200 bg-zinc-50 p-3">
        <div className="flex items-center justify-between gap-3">
          <div>
            <div className="text-xs font-semibold text-zinc-800">OMPL parallel threads</div>
            <div className="mt-0.5 text-[10px] text-zinc-500">
              {resources?resources.logical_cpus+" logical CPUs · "+bytesLabel(resources.memory_total_bytes)+" RAM":"Reading host resource limits…"}
            </div>
          </div>
          <select className="w-20 rounded-md border border-zinc-200 bg-white px-2 py-1.5 text-xs"
            value={profile.max_planning_threads} disabled={disabled||profile.pipeline_id!=="ompl"}
            onChange={event=>update({max_planning_threads:Number(event.target.value)})}>
            {threadChoices.map(value=><option key={value} value={value}>{value}{resources&&value>resources.max_planning_threads?" · over host limit":""}</option>)}
          </select>
        </div>
        {resources&&<div className="mt-2 flex flex-wrap items-center justify-between gap-2 text-[10px] text-zinc-500">
          <span>{bytesLabel(availableRam)} RAM available</span>
          <span>Suggested: {resources.recommended_planning_threads} thread{resources.recommended_planning_threads===1?"":"s"}</span>
        </div>}
        {resources?.memory_pressure&&<p className="mt-2 text-[11px] leading-4 text-amber-800">
          Low memory headroom or heavy swap use detected. Prefer one or two OMPL threads until memory pressure clears.
        </p>}
        {resourceError&&<p className="mt-2 text-[10px] text-amber-700">Host resources unavailable: {resourceError}</p>}
        <p className="mt-2 text-[10px] leading-4 text-zinc-500">
          Thread count is a MoveGroup startup setting, not a per-plan field. Copy the YAML setting below and restart MoveGroup to change the active limit.
        </p>
        <Button variant="outline" size="sm" className="mt-2 h-7 text-[10px]" onClick={copyThreadsYaml}>
          <Copy size={11}/> Copy OMPL thread YAML
        </Button>
        {isHighThreadChoice&&<p className="mt-2 text-[10px] text-amber-700">This profile exceeds the resource-based recommendation{resources&&profile.max_planning_threads>resources.max_planning_threads?" and the detected safe limit":""}. It is saved as a setting, not applied to the current MoveGroup process.</p>}
      </div>

      {profile.pipeline_id==="ompl"&&<div>
        <label className={LABEL}>Advanced OMPL planner parameters (JSON object)</label>
        <textarea className={FIELD+" min-h-16 resize-y font-mono text-[11px]"} spellCheck={false}
          value={plannerParamsDraft} disabled={disabled}
          onChange={event=>setPlannerParamsDraft(event.target.value)}/>
        <div className="mt-2 flex items-center justify-between gap-2">
          <p className="text-[10px] leading-4 text-zinc-500">Applied through MoveIt’s set_planner_params service before the next plan. Overrides persist in MoveGroup until overwritten or restarted; an empty profile does not clear them.</p>
          <Button size="sm" variant="outline" disabled={disabled} onClick={()=>{
            try{
              const parsed=JSON.parse(plannerParamsDraft);
              if(!parsed||typeof parsed!=="object"||Array.isArray(parsed))throw new Error("Use a JSON object.");
              update({planner_parameters:parsed});
              setMessage("OMPL planner parameters applied to the profile.");
              setMessageError(false);
            }catch(error){
              setMessage(error instanceof Error?error.message:"Invalid planner parameter JSON.");
              setMessageError(true);
            }
          }}>Apply params</Button>
        </div>
      </div>}

      <div className="flex flex-wrap gap-2">
        <label className="min-w-0 flex-1">
          <span className={LABEL}>Profile name</span>
          <input className={FIELD} value={profileName} maxLength={48} disabled={disabled} onChange={event=>setProfileName(event.target.value)}/>
        </label>
        <div className="flex items-end gap-1.5">
          <Button size="sm" onClick={saveProfile} disabled={disabled||!profileName.trim()}><Save size={12}/> Save</Button>
          <Button size="sm" variant="outline" onClick={deleteProfile} disabled={disabled||selectedIsBuiltin}><Trash2 size={12}/></Button>
        </div>
      </div>

      <div className="border-t border-zinc-100 pt-3">
        <button className="flex w-full items-center justify-between text-left text-xs font-semibold text-zinc-700"
          onClick={()=>setJsonOpen(value=>!value)} type="button">
          <span className="flex items-center gap-2"><FileJson size={13}/> Manual JSON profile</span>
          <span className="text-[10px] font-normal text-zinc-400">{jsonOpen?"Hide":"Import / export"}</span>
        </button>
        {jsonOpen&&<div className="mt-3 space-y-2">
          <textarea className={FIELD+" min-h-52 resize-y font-mono text-[10px] leading-4"} spellCheck={false}
            value={jsonDraft} disabled={disabled} onChange={event=>setJsonDraft(event.target.value)}/>
          <div className="flex gap-2">
            <Button size="sm" onClick={applyJson} disabled={disabled}>Apply JSON</Button>
            <Button size="sm" variant="outline" onClick={copyJson}><Copy size={12}/> Copy JSON</Button>
          </div>
        </div>}
      </div>
      {message&&<div className={"rounded-md px-3 py-2 text-[11px] "+(messageError?"bg-amber-50 text-amber-800":"bg-zinc-50 text-zinc-600")}>{message}</div>}
    </div>
  </section>;
}
