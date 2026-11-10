import { create } from "zustand";
import type {
  BridgeState, JointStateMessage, MotionPlan, PlanningScene, RobotCapabilities,
  RobotModeMessage, RobotModel, RobotStatusMessage, TcpMessage
} from "@/lib/types";

export interface TopicMessageState {
  data:any;
  timestamp?:string;
  sequence?:number;
}

const JOINT_STATE_FIELDS=["position","velocity","effort"] as const;

function mergeJointStateMessages(
  previous:JointStateMessage|null,
  incoming:JointStateMessage|null|undefined
):JointStateMessage|null{
  if(!incoming)return previous;
  const incomingNames=Array.isArray(incoming.name)?incoming.name:[];
  if(incomingNames.length===0)return previous??incoming;

  type JointStateValues=Partial<Record<(typeof JOINT_STATE_FIELDS)[number],number|null>>;
  const merged=new Map<string,JointStateValues>();
  const consume=(message:JointStateMessage|null|undefined)=>{
    if(!message||!Array.isArray(message.name))return;
    message.name.forEach((rawName,index)=>{
      const name=String(rawName??"").trim();
      if(!name)return;
      const values=merged.get(name)??{};
      JOINT_STATE_FIELDS.forEach(field=>{
        const fieldValues=message[field];
        if(Array.isArray(fieldValues)&&index<fieldValues.length){
          values[field]=fieldValues[index]??null;
        }
      });
      merged.set(name,values);
    });
  };
  consume(previous);
  consume(incoming);

  const names=[...merged.keys()];
  const result:JointStateMessage={name:names};
  JOINT_STATE_FIELDS.forEach(field=>{
    if(names.some(name=>field in (merged.get(name)??{}))){
      result[field]=names.map(name=>merged.get(name)?.[field]??null);
    }
  });
  return result;
}

interface RobotStore {
  bridgeState: BridgeState; connectionError: string | null; simulation: boolean;
  robotStatus: RobotStatusMessage | null; robotMode: RobotModeMessage | null;
  errors: Record<string,unknown> | null; joints: JointStateMessage | null; tcp: TcpMessage | null;
  timestamps: {status?:string;joints?:string;tcp?:string}; capabilities: RobotCapabilities;
  lastCommand: string | null; lastJointChangeAt: number | undefined;
  scene: PlanningScene | null; sceneLoading: boolean; sceneError: string | null;
  model: RobotModel | null; modelLoading: boolean; modelError: string | null;
  jointLimits: Record<string,any> | null; plan: MotionPlan | null; executionStatus: string | null;
  notice: {tone:"success"|"error"|"neutral";message:string} | null;
  setBridgeState:(value:BridgeState,error?:string|null)=>void;
  setRobotStatus:(value:RobotStatusMessage,timestamp?:string)=>void; setRobotMode:(value:RobotModeMessage)=>void;
  setErrors:(value:Record<string,unknown>|null)=>void; setJoints:(value:JointStateMessage,timestamp?:string)=>void;
  setTcp:(value:TcpMessage,timestamp?:string)=>void; mergeSnapshot:(value:any)=>void;
  setScene:(value:PlanningScene|null)=>void; setSceneLoading:(value:boolean,error?:string|null)=>void;
  setModel:(value:RobotModel|null)=>void; setModelLoading:(value:boolean,error?:string|null)=>void;
  setJointLimits:(value:Record<string,any>)=>void; setPlan:(value:MotionPlan|null)=>void;
  setExecutionStatus:(value:string|null)=>void;
  topicMessages:Record<string,TopicMessageState>;
  topicSubscriptionErrors:Record<string,string>;
  setTopicMessage:(topic:string,data:any,timestamp?:string,sequence?:number)=>void;
  setTopicSubscriptionError:(topic:string,error:string|null)=>void;
  setNotice:(value:RobotStore["notice"])=>void;
  setCommand:(value:string,response?:Record<string,unknown>)=>void;
}

export const useRobotStore = create<RobotStore>((set)=>({
  bridgeState:"connecting",connectionError:null,simulation:false,robotStatus:null,robotMode:null,errors:null,
  joints:null,tcp:null,timestamps:{},capabilities:{},lastCommand:null,lastJointChangeAt:undefined,
  scene:null,sceneLoading:false,sceneError:null,model:null,modelLoading:false,modelError:null,
  jointLimits:null,plan:null,executionStatus:null,topicMessages:{},topicSubscriptionErrors:{},notice:null,
  setBridgeState:(value,error=null)=>set({bridgeState:value,connectionError:error}),
  setRobotStatus:(value,timestamp)=>set((state)=>({robotStatus:value,timestamps:{...state.timestamps,status:timestamp??state.timestamps.status}})),
  setRobotMode:(value)=>set({robotMode:value}),
  setErrors:(value)=>set({errors:value}),
  setJoints:(value,timestamp)=>set((state)=>{
    const merged=mergeJointStateMessages(state.joints,value)??value;
    const previousByName=new Map((state.joints?.name??[]).map((name,index)=>[
      name,state.joints?.position?.[index]
    ]));
    const changed=(merged.name??[]).some((name,index)=>{
      const before=previousByName.get(name);
      const current=merged.position?.[index];
      return typeof current==="number"&&Number.isFinite(current)&&
        typeof before==="number"&&Number.isFinite(before)&&Math.abs(current-before)>0.002;
    });
    return {joints:merged,lastJointChangeAt:changed?Date.now():state.lastJointChangeAt,timestamps:{...state.timestamps,joints:timestamp??state.timestamps.joints}};
  }),
  setTcp:(value,timestamp)=>set((state)=>({tcp:value,timestamps:{...state.timestamps,tcp:timestamp??state.timestamps.tcp}})),
  mergeSnapshot:(value)=>set((state)=>({
    robotStatus:{is_connected:value.connected,is_enable:value.enabled},
    robotMode:value.mode??state.robotMode,errors:value.errors??state.errors,
    joints:mergeJointStateMessages(state.joints,value.joints)??state.joints,tcp:value.tcp??state.tcp,
    timestamps:value.timestamps??state.timestamps,capabilities:value.capabilities??state.capabilities,
    simulation:value.simulation===true
  })),
  setScene:(value)=>set({scene:value,sceneLoading:false,sceneError:null}),
  setSceneLoading:(value,error=null)=>set({sceneLoading:value,sceneError:error}),
  setModel:(value)=>set({model:value,modelLoading:false,modelError:null}),
  setModelLoading:(value,error=null)=>set({modelLoading:value,modelError:error}),
  setJointLimits:(value)=>set({jointLimits:value}),
  setPlan:(value)=>set({plan:value}),
  setExecutionStatus:(value)=>set({executionStatus:value}),
  setTopicMessage:(topic,data,timestamp,sequence)=>set((state)=>{
    const current=state.topicMessages[topic];
    if(current&&sequence!==undefined&&current.sequence!==undefined&&sequence<current.sequence)return state;
    const subscriptionErrors={...state.topicSubscriptionErrors};
    delete subscriptionErrors[topic];
    return {
      topicMessages:{...state.topicMessages,[topic]:{data,timestamp,sequence}},
      topicSubscriptionErrors:subscriptionErrors
    };
  }),
  setTopicSubscriptionError:(topic,error)=>set((state)=>{
    const subscriptionErrors={...state.topicSubscriptionErrors};
    if(error)subscriptionErrors[topic]=error;else delete subscriptionErrors[topic];
    return {topicSubscriptionErrors:subscriptionErrors};
  }),
  setNotice:(value)=>set({notice:value}),
  setCommand:(value,response)=>{
    if(response?.res===undefined || response.res===0) set({lastCommand:value});
  }
}));
