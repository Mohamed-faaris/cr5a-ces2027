import {useCallback,useEffect,useMemo,useState} from "react";
import {extractRobotModeCode,hasActiveError} from "@/lib/status";
import {useRobotSocket} from "@/lib/socket";
import {useRobotStore} from "@/store/robot-store";

export const PGEA_STROKE_MM=20;

export type GripperSnapshot={
  init?:number;init_text?:string;grip?:number;grip_text?:string;
  position?:number;current?:number;error?:number;temperature?:number|null;
  alarm?:{code?:number;name?:string;remedy?:string};
};
export type GripperProfile={
  id:string;name:string;force_percent:number;speed_percent:number;
};
export type GripperProfilesState={
  profiles:GripperProfile[];active_profile_id:string;storage_path:string;
  runtime_synced:boolean;runtime_error?:string|null;saved_profile?:GripperProfile;
};
export type GripperState={
  connected:boolean;simulation:boolean;status:GripperSnapshot|null;
  error:string|null;commands_enabled:boolean;message?:string;
};

export function useGripperController(){
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
  const [busy,setBusy]=useState<"move"|"initialize"|"calibrate"|"stop"|"profile-save"|"profile-apply"|"profile-delete"|null>(null);
  const [notice,setNotice]=useState<{tone:"success"|"error"|"neutral";message:string}|null>(null);
  const [refreshing,setRefreshing]=useState(false);
  const [profiles,setProfiles]=useState<GripperProfilesState|null>(null);
  const [selectedProfileId,setSelectedProfileId]=useState("");
  const [profileName,setProfileName]=useState("");
  const [forcePercent,setForcePercent]=useState("60");
  const [speedPercent,setSpeedPercent]=useState("50");
  const [profileDraftDirty,setProfileDraftDirty]=useState(false);

  const refresh=useCallback(async(syncTarget=false)=>{
    setRefreshing(true);
    try{
      const value=await request("gripper.get_status") as GripperState;
      setGripper(value);
      const actual=Number(value?.status?.position);
      if(syncTarget&&!targetDirty&&Number.isFinite(actual)){
        setTargetMm(String(Number((actual/1000*PGEA_STROKE_MM).toFixed(2))));
      }
    }catch(error){
      setGripper({
        connected:false,simulation,status:null,
        error:error instanceof Error?error.message:String(error),
        commands_enabled:false
      });
    }finally{
      setRefreshing(false);
    }
  },[request,simulation,targetDirty]);

  const fetchProfiles=useCallback(async()=>{
    try{
      const value=await request("gripper.get_profiles") as GripperProfilesState;
      setProfiles(value);
      if(!profileDraftDirty){
        const active=value.profiles.find((item)=>item.id===value.active_profile_id)??value.profiles[0];
        if(active){
          setSelectedProfileId(active.id);
          setProfileName(active.name);
          setForcePercent(String(active.force_percent));
          setSpeedPercent(String(active.speed_percent));
        }
      }
    }catch(error){
      setNotice({tone:"error",message:error instanceof Error?error.message:String(error)});
    }
  },[request]);

  useEffect(()=>{
    void refresh(true);
    const timer=window.setInterval(()=>{void refresh(false);},1500);
    return ()=>window.clearInterval(timer);
  },[refresh]);

  useEffect(()=>{void fetchProfiles();},[fetchProfiles]);

  const status=gripper?.status??null;
  const positionMm=typeof status?.position==="number"
    ?status.position/1000*PGEA_STROKE_MM:null;
  const initialized=status?.init===1;
  const modeCode=extractRobotModeCode(robotMode?.value);
  const armIdleOrDisabled=modeCode===3||modeCode===4||modeCode===5;
  const controllerEnabled=robotStatus?.is_connected===true&&
    robotStatus?.is_enable===true&&modeCode===5;
  const activeRobotError=hasActiveError(errors);
  const canControl=bridgeState==="connected"&&gripper?.connected===true&&
    gripper.commands_enabled===true&&capabilities.gripper_commands_enabled===true&&
    robotStatus?.is_connected===true&&armIdleOrDisabled&&!activeRobotError&&busy===null;
  const canMove=canControl&&initialized&&controllerEnabled;
  const numericTarget=Number(targetMm);
  const targetValid=Number.isFinite(numericTarget)&&numericTarget>=0&&numericTarget<=PGEA_STROKE_MM;
  const currentLabel=positionMm===null?"—":positionMm.toFixed(2);
  const sourceLabel=gripper?.simulation||simulation?"Fake PGEA":"PGEA-100-40-W";
  const alarm=status?.alarm;
  const alarmActive=typeof alarm?.code==="number"&&alarm.code!==0;
  const selectedProfile=profiles?.profiles.find((item)=>item.id===selectedProfileId)??null;
  const numericForce=Number(forcePercent);
  const numericSpeed=Number(speedPercent);
  const profileValid=Number.isInteger(numericForce)&&numericForce>=20&&numericForce<=100&&
    Number.isInteger(numericSpeed)&&numericSpeed>=1&&numericSpeed<=100&&profileName.trim().length>0&&profileName.trim().length<=48;

  const updateProfileDraft=(updates:{name?:string;force?:string;speed?:string})=>{
    setProfileDraftDirty(true);
    if(updates.name!==undefined)setProfileName(updates.name);
    if(updates.force!==undefined)setForcePercent(updates.force);
    if(updates.speed!==undefined)setSpeedPercent(updates.speed);
  };
  const selectProfile=(id:string)=>{
    if(profileDraftDirty&&!window.confirm("Discard unsaved profile edits?"))return;
    const profile=profiles?.profiles.find((item)=>item.id===id);
    if(!profile)return;
    setSelectedProfileId(id);
    setProfileName(profile.name);
    setForcePercent(String(profile.force_percent));
    setSpeedPercent(String(profile.speed_percent));
    setProfileDraftDirty(false);
    setNotice(null);
  };

  const resetProfileDraft=()=>{
    const profile=profiles?.profiles.find((item)=>item.id===selectedProfileId);
    if(!profile)return;
    setProfileName(profile.name);
    setForcePercent(String(profile.force_percent));
    setSpeedPercent(String(profile.speed_percent));
    setProfileDraftDirty(false);
  };

  const saveProfile=useCallback(async(asNew=false)=>{
    if(!profileValid){
      setNotice({tone:"error",message:"Profile name must be 1–48 characters; force must be 20–100% and speed 1–100% (whole numbers)."});
      return;
    }
    setBusy("profile-save");setNotice(null);
    try{
      const profile={
        ...(!asNew&&selectedProfileId?{id:selectedProfileId}:{}),
        name:profileName.trim(),force_percent:numericForce,speed_percent:numericSpeed
      };
      const value=await request("gripper.save_profile",{profile}) as GripperProfilesState;
      setProfiles(value);
      const saved=value.saved_profile??value.profiles.find((item)=>item.id===selectedProfileId);
      if(saved){
        setSelectedProfileId(saved.id);
        setProfileName(saved.name);
        setForcePercent(String(saved.force_percent));
        setSpeedPercent(String(saved.speed_percent));
      }
      setProfileDraftDirty(false);
      setNotice(value.runtime_error
        ?{tone:"error",message:"Profile saved, but the PGEA did not accept the runtime settings: "+value.runtime_error}
        :{tone:"success",message:asNew?"New gripper profile saved.":"Gripper profile saved."});
    }catch(error){
      setNotice({tone:"error",message:error instanceof Error?error.message:String(error)});
    }finally{setBusy(null);}
  },[request,profileValid,selectedProfileId,profileName,numericForce,numericSpeed]);

  const applyProfile=useCallback(async()=>{
    if(!selectedProfileId)return;
    setBusy("profile-apply");setNotice(null);
    try{
      const value=await request("gripper.apply_profile",{profile_id:selectedProfileId}) as GripperProfilesState;
      setProfiles(value);
      setProfileDraftDirty(false);
      setNotice({tone:"success",message:"Active PGEA profile applied. Subsequent position moves will use "+forcePercent+"% force and "+speedPercent+"% speed."});
    }catch(error){
      setNotice({tone:"error",message:error instanceof Error?error.message:String(error)});
    }finally{setBusy(null);}
  },[request,selectedProfileId,forcePercent,speedPercent]);

  const deleteProfile=useCallback(async()=>{
    if(!selectedProfileId||selectedProfileId===profiles?.active_profile_id)return;
    if(!window.confirm("Delete the gripper profile '"+profileName+"'?"))return;
    setBusy("profile-delete");setNotice(null);
    try{
      const value=await request("gripper.delete_profile",{profile_id:selectedProfileId}) as GripperProfilesState;
      setProfiles(value);
      const active=value.profiles.find((item)=>item.id===value.active_profile_id)??value.profiles[0];
      if(active)selectProfile(active.id);
      setNotice({tone:"success",message:"Gripper profile deleted."});
    }catch(error){
      setNotice({tone:"error",message:error instanceof Error?error.message:String(error)});
    }finally{setBusy(null);}
  },[request,selectedProfileId,profiles,profileName]);

  const sendMove=useCallback(async(mm:number)=>{
    if(!Number.isFinite(mm)||mm<0||mm>PGEA_STROKE_MM){
      setNotice({tone:"error",message:"Target must be between 0 and 20 mm per finger."});
      return;
    }
    const direction=mm===0?"open":mm===PGEA_STROKE_MM?"close":"move";
    if(!window.confirm(
      "Command the PGEA to "+direction+" to "+mm.toFixed(1)+
      " mm per finger? Confirm the finger travel area is clear."
    ))return;
    setBusy("move");setNotice(null);
    try{
      const result=await request("gripper.move",{position_m:mm/1000,max_effort:0});
      setNotice({
        tone:result?.reached_goal===true?"success":"error",
        message:result?.reached_goal===true
          ?"PGEA reached the requested position."
          :"PGEA did not confirm the requested position."
      });
      setTargetDirty(false);
      await refresh(true);
    }catch(error){
      setNotice({tone:"error",message:error instanceof Error?error.message:String(error)});
      setTargetDirty(false);
      await refresh(true);
    }finally{
      setBusy(null);
    }
  },[request,refresh]);

  const lifecycle=useCallback(async(action:"initialize"|"calibrate")=>{
    const message=action==="calibrate"
      ?"Run full close-to-open PGEA calibration? Remove the workpiece and clear the full finger travel area. The fingers will move through their range."
      :"Initialize the PGEA? The fingers may move during initialization.";
    if(!window.confirm(message))return;
    setBusy(action);setNotice(null);
    try{
      const result=await request("gripper."+action);
      setNotice({
        tone:"success",
        message:result?.message??(action==="initialize"?"PGEA initialized.":"PGEA calibration completed.")
      });
      await refresh();
    }catch(error){
      setNotice({tone:"error",message:error instanceof Error?error.message:String(error)});
      await refresh();
    }finally{
      setBusy(null);
    }
  },[request,refresh]);

  const stop=useCallback(async()=>{
    setBusy("stop");
    try{
      await request("gripper.stop");
      setNotice({tone:"neutral",message:"PGEA stop requested."});
    }catch(error){
      setNotice({tone:"error",message:error instanceof Error?error.message:String(error)});
    }finally{
      setBusy(null);
      await refresh();
    }
  },[request,refresh]);

  const readiness=useMemo(()=>{
    if(bridgeState!=="connected")return "Waiting for the robot bridge.";
    if(!gripper?.connected)return gripper?.error??"PGEA status service is not available. Check that the gripper node is launched.";
    if(!robotStatus?.is_connected)return "Dobot arm connection is not confirmed.";
    if(!armIdleOrDisabled)return "Wait until the arm is disabled or enabled and idle; gripper initialization and calibration are blocked during motion, pause or drag mode.";
    if(activeRobotError)return "Clear the active Dobot controller error before operating the gripper.";
    if(!gripper.commands_enabled||!capabilities.gripper_commands_enabled)return "PGEA commands are disabled by the bridge safety configuration.";
    if(!initialized)return "Initialize the PGEA before commanding finger movement.";
    if(!controllerEnabled)return "Enable the robot from Diagnostics before moving the gripper.";
    return "PGEA is ready for position commands.";
  },[
    bridgeState,gripper,robotStatus,armIdleOrDisabled,activeRobotError,
    capabilities.gripper_commands_enabled,initialized,controllerEnabled
  ]);

  return {
    gripper,status,positionMm,currentLabel,sourceLabel,alarm,alarmActive,
    targetMm,targetValid,busy,notice,refreshing,readiness,
    initialized,controllerEnabled,activeRobotError,canControl,canMove,
    profiles,selectedProfile,selectedProfileId,profileName,forcePercent,speedPercent,profileValid,
    profileBusy:busy==="profile-save"||busy==="profile-apply"||busy==="profile-delete",
    profileDraftDirty,updateProfileDraft,selectProfile,resetProfileDraft,saveProfile,applyProfile,deleteProfile,fetchProfiles,
    setTargetMm:(value:string)=>{
      setTargetDirty(true);
      setTargetMm(value);
    },
    move:sendMove,lifecycle,stop,refresh
  };
}

export type GripperController=ReturnType<typeof useGripperController>;
