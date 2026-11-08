import { useRobotStore } from "@/store/robot-store";

type PendingRequest={resolve:(value:any)=>void;reject:(reason:Error)=>void;timeout:number};
let socket:WebSocket|null=null;
let reconnectTimer:number|undefined;
let pingTimer:number|undefined;
let requestSequence=0;
let socketGeneration=0;
let sceneRefreshTimer:number|undefined;
const pending=new Map<string,PendingRequest>();
const channelReferences=new Map<string,number>();
let stopRequested=false;
let retryCount=0;

function wsUrl(){
  const configured=import.meta.env.VITE_BRIDGE_WS_URL as string|undefined;
  const url=configured||((window.location.protocol==="https:"?"wss://":"ws://")+window.location.host+"/ws/v1");
  const token=import.meta.env.VITE_BRIDGE_TOKEN as string|undefined;
  if(!token)return url;
  const parsed=new URL(url,window.location.href);
  parsed.searchParams.set("token",token);
  return parsed.toString();
}

function sendSubscription(operation:"subscribe"|"unsubscribe",channels:string[]){
  if(!socket||socket.readyState!==WebSocket.OPEN||channels.length===0)return;
  socket.send(JSON.stringify({op:operation,channels}));
}

export function subscribeChannels(channels:string[]){
  const newlyAdded:string[]=[];
  for(const channel of channels){
    const count=channelReferences.get(channel)??0;
    channelReferences.set(channel,count+1);
    if(count===0)newlyAdded.push(channel);
  }
  sendSubscription("subscribe",newlyAdded);
  let released=false;
  return ()=>{
    if(released)return;
    released=true;
    const removed:string[]=[];
    for(const channel of channels){
      const count=channelReferences.get(channel)??0;
      if(count<=1){
        channelReferences.delete(channel);
        if(count===1)removed.push(channel);
      }else channelReferences.set(channel,count-1);
    }
    sendSubscription("unsubscribe",removed);
  };
}
export function request(method:string,params:Record<string,unknown>={}):Promise<any>{
  if(!socket||socket.readyState!==WebSocket.OPEN)return Promise.reject(new Error("ROS bridge WebSocket is not connected"));
  const id="web_"+Date.now().toString(36)+"_"+(++requestSequence).toString(36);
  return new Promise((resolve,reject)=>{
    const timeout=window.setTimeout(()=>{
      pending.delete(id);reject(new Error("Request timed out: "+method));
    },method==="robot.direct.execute"?120000:(method==="motion.plan"||method==="robot.direct.check"?60000:20000));
    pending.set(id,{resolve,reject,timeout});
    try{socket?.send(JSON.stringify({op:"request",id,method,params}));}
    catch(error){
      window.clearTimeout(timeout);pending.delete(id);
      reject(error instanceof Error?error:new Error(String(error)));
    }
  });
}
function rejectPending(message:string){
  pending.forEach((item)=>{window.clearTimeout(item.timeout);item.reject(new Error(message));});
  pending.clear();
}
function handleEvent(channel:string,data:any,timestamp?:string,sequence?:number){
  const store=useRobotStore.getState();
  if(channel.startsWith("topic:")){
    store.setTopicMessage(channel.slice(6),data,timestamp,sequence);
    return;
  }
  switch(channel){
    case "robot.status":store.setRobotStatus(data,timestamp);break;
    case "robot.mode":store.setRobotMode(data);break;
    case "robot.joints":store.setJoints(data,timestamp);break;
    case "robot.tcp":store.setTcp(data,timestamp);break;
    case "robot.errors":store.setErrors(data);break;
    case "robot.state":if(data?.command)store.setCommand(data.command,data.response);break;
    case "motion.plan":store.setPlan({...(store.plan??{}),...data});break;
    case "motion.execution":store.setExecutionStatus(data?.status??null);break;
    case "moveit.scene":
      store.setSceneLoading(true);
      if(sceneRefreshTimer!==undefined)window.clearTimeout(sceneRefreshTimer);
      sceneRefreshTimer=window.setTimeout(()=>{
        sceneRefreshTimer=undefined;
        void request("moveit.get_scene").then((scene)=>useRobotStore.getState().setScene(scene))
          .catch((error)=>useRobotStore.getState().setSceneLoading(false,String(error)));
      },120);
      break;
    default:break;
  }
}
const modelRetryDelays=[250,500,750,1000,1500,2000,2000,2000,2000];

async function requestRobotModel(){
  for(let attempt=0;;attempt++){
    try{
      return await request("robot.get_model");
    }catch(reason){
      const error=reason instanceof Error?reason:new Error(String(reason));
      const retryable=/robot_description is unavailable|Service unavailable:/.test(error.message);
      if(!retryable||attempt>=modelRetryDelays.length)throw error;
      await new Promise<void>((resolve)=>window.setTimeout(resolve,modelRetryDelays[attempt]));
      if(!socket||socket.readyState!==WebSocket.OPEN){
        throw new Error("ROS bridge disconnected while waiting for robot_description");
      }
    }
  }
}

async function loadInitialState(){
  const store=useRobotStore.getState();
  const needsModel=store.model===null;
  store.setSceneLoading(true);
  if(needsModel)store.setModelLoading(true);
  const tasks=[
    request("robot.get_snapshot").then((value)=>useRobotStore.getState().mergeSnapshot(value)),
    request("moveit.get_scene").then((value)=>useRobotStore.getState().setScene(value))
      .catch((error)=>useRobotStore.getState().setSceneLoading(false,String(error))),
    ...(needsModel?[
      requestRobotModel().then((value)=>useRobotStore.getState().setModel(value))
        .catch((error)=>useRobotStore.getState().setModelLoading(false,String(error)))
    ]:[]),
    request("robot.get_joint_limits").then((value)=>useRobotStore.getState().setJointLimits(value)).catch(()=>undefined)
  ];
  await Promise.allSettled(tasks);
}
export function startSocket(){
  const generation=++socketGeneration;
  stopRequested=false;
  let firstAttempt=true;
  const scheduleReconnect=()=>{
    if(generation!==socketGeneration||stopRequested||reconnectTimer!==undefined)return;
    const delay=Math.min(1000*Math.pow(1.6,retryCount++),8000);
    reconnectTimer=window.setTimeout(()=>{
      reconnectTimer=undefined;
      connect();
    },delay);
  };
  const connect=()=>{
    if(generation!==socketGeneration||stopRequested)return;
    useRobotStore.getState().setBridgeState(firstAttempt?"connecting":"reconnecting");
    firstAttempt=false;
    let currentSocket:WebSocket;
    try{
      currentSocket=new WebSocket(wsUrl());
      socket=currentSocket;
    }catch(error){
      useRobotStore.getState().setBridgeState("disconnected",String(error));
      scheduleReconnect();
      return;
    }
    currentSocket.onopen=()=>{
      if(generation!==socketGeneration||socket!==currentSocket)return;
      retryCount=0;
      useRobotStore.getState().setBridgeState("connected");
      const defaultChannels=[
        "robot.status","robot.mode","robot.joints","robot.tcp","robot.errors",
        "robot.state","moveit.scene","motion.plan","motion.execution","bridge.health"
      ];
      const channels=[...new Set([...defaultChannels,...channelReferences.keys()])];
      currentSocket.send(JSON.stringify({op:"subscribe",channels}));
      void loadInitialState();
      pingTimer=window.setInterval(()=>{
        if(socket===currentSocket&&currentSocket.readyState===WebSocket.OPEN){
          currentSocket.send(JSON.stringify({op:"ping"}));
        }
      },15000);
    };
    currentSocket.onmessage=(event)=>{
      if(generation!==socketGeneration||socket!==currentSocket)return;
      let message:any;
      try{message=JSON.parse(event.data);}catch{return;}
      if(message.op==="response"){
        const item=pending.get(String(message.id));
        if(!item)return;
        window.clearTimeout(item.timeout);
        pending.delete(String(message.id));
        if(message.ok)item.resolve(message.data);
        else item.reject(new Error(message.error?.message||"WebSocket request failed"));
      }else if(message.op==="event"){
        handleEvent(message.channel,message.data,message.timestamp,message.sequence);
      }else if(message.op==="subscription_error"&&typeof message.channel==="string"&&message.channel.startsWith("topic:")){
        useRobotStore.getState().setTopicSubscriptionError(message.channel.slice(6),String(message.detail??"Subscription failed"));
      }
    };
    currentSocket.onerror=()=>{
      if(generation===socketGeneration&&socket===currentSocket){
        useRobotStore.getState().setBridgeState("reconnecting","WebSocket connection error");
      }
    };
    currentSocket.onclose=()=>{
      if(socket===currentSocket)socket=null;
      if(generation!==socketGeneration)return;
      if(pingTimer!==undefined)window.clearInterval(pingTimer);
      pingTimer=undefined;
      rejectPending("WebSocket disconnected");
      if(!stopRequested){
        useRobotStore.getState().setBridgeState("reconnecting","Waiting for the ROS bridge");
        scheduleReconnect();
      }
    };
  };
  connect();
  return ()=>{
    if(generation!==socketGeneration)return;
    socketGeneration++;
    stopRequested=true;
    if(reconnectTimer!==undefined)window.clearTimeout(reconnectTimer);
    if(pingTimer!==undefined)window.clearInterval(pingTimer);
    if(sceneRefreshTimer!==undefined)window.clearTimeout(sceneRefreshTimer);
    reconnectTimer=undefined;
    pingTimer=undefined;
    sceneRefreshTimer=undefined;
    rejectPending("WebSocket client stopped");
    const current=socket;
    socket=null;
    current?.close();
    useRobotStore.getState().setBridgeState("disconnected");
  };
}
