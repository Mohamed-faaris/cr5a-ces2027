import { create } from "zustand";
import type {
  BridgeState, JointStateMessage, MotionPlan, PlanningScene, RobotCapabilities,
  RobotModeMessage, RobotModel, RobotStatusMessage, TcpMessage
} from "@/lib/types";

interface RobotStore {
  bridgeState: BridgeState; connectionError: string | null;
  robotStatus: RobotStatusMessage | null; robotMode: RobotModeMessage | null;
  errors: Record<string,unknown> | null; joints: JointStateMessage | null; tcp: TcpMessage | null;
  timestamps: {status?:string;joints?:string;tcp?:string}; capabilities: RobotCapabilities;
  lastCommand: string | null; lastJointChangeAt: number | undefined;
  scene: PlanningScene | null; sceneLoading: boolean; sceneError: string | null;
  model: RobotModel | null; modelLoading: boolean; modelError: string | null;
  jointLimits: Record<string,any> | null; plan: MotionPlan | null; executionStatus: string | null;
  notice: {tone:"success"|"error"|"neutral";message:string} | null;
  setBridgeState:(value:BridgeState,error?:string|null)=>void;
  setRobotStatus:(value:RobotStatusMessage)=>void; setRobotMode:(value:RobotModeMessage)=>void;
  setErrors:(value:Record<string,unknown>|null)=>void; setJoints:(value:JointStateMessage)=>void;
  setTcp:(value:TcpMessage)=>void; mergeSnapshot:(value:any)=>void;
  setScene:(value:PlanningScene|null)=>void; setSceneLoading:(value:boolean,error?:string|null)=>void;
  setModel:(value:RobotModel|null)=>void; setModelLoading:(value:boolean,error?:string|null)=>void;
  setJointLimits:(value:Record<string,any>)=>void; setPlan:(value:MotionPlan|null)=>void;
  setExecutionStatus:(value:string|null)=>void;
  setNotice:(value:RobotStore["notice"])=>void;
  setCommand:(value:string,response?:Record<string,unknown>)=>void;
}

export const useRobotStore = create<RobotStore>((set)=>({
  bridgeState:"connecting",connectionError:null,robotStatus:null,robotMode:null,errors:null,
  joints:null,tcp:null,timestamps:{},capabilities:{},lastCommand:null,lastJointChangeAt:undefined,
  scene:null,sceneLoading:false,sceneError:null,model:null,modelLoading:false,modelError:null,
  jointLimits:null,plan:null,executionStatus:null,notice:null,
  setBridgeState:(value,error=null)=>set({bridgeState:value,connectionError:error}),
  setRobotStatus:(value)=>set({robotStatus:value}),
  setRobotMode:(value)=>set({robotMode:value}),
  setErrors:(value)=>set({errors:value}),
  setJoints:(value)=>set((state)=>{
    const previous=state.joints?.position??[];
    const next=value.position??[];
    const changed=previous.length===next.length && next.some((item,index)=>Math.abs(item-(previous[index]??item))>0.002);
    return {joints:value,lastJointChangeAt:changed?Date.now():state.lastJointChangeAt};
  }),
  setTcp:(value)=>set({tcp:value}),
  mergeSnapshot:(value)=>set((state)=>({
    robotStatus:{is_connected:value.connected,is_enable:value.enabled},
    robotMode:value.mode??state.robotMode,errors:value.errors??state.errors,
    joints:value.joints??state.joints,tcp:value.tcp??state.tcp,
    timestamps:value.timestamps??state.timestamps,capabilities:value.capabilities??state.capabilities
  })),
  setScene:(value)=>set({scene:value,sceneLoading:false,sceneError:null}),
  setSceneLoading:(value,error=null)=>set({sceneLoading:value,sceneError:error}),
  setModel:(value)=>set({model:value,modelLoading:false,modelError:null}),
  setModelLoading:(value,error=null)=>set({modelLoading:value,modelError:error}),
  setJointLimits:(value)=>set({jointLimits:value}),
  setPlan:(value)=>set({plan:value}),
  setExecutionStatus:(value)=>set({executionStatus:value}),
  setNotice:(value)=>set({notice:value}),
  setCommand:(value,response)=>{
    if(response?.res===undefined || response.res===0) set({lastCommand:value});
  }
}));
