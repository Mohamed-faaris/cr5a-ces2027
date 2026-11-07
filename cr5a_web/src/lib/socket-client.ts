import { useRobotStore } from "@/store/robot-store";

type PendingRequest={resolve:(value:any)=>void;reject:(reason:Error)=>void;timeout:number};
let socket:WebSocket|null=null;
let reconnectTimer:number|undefined;
let pingTimer:number|undefined;
let requestSequence=0;
const pending=new Map<string,PendingRequest>();
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
export function request(method:string,params:Record<string,unknown>={}):Promise<any>{
  if(!socket||socket.readyState!==WebSocket.OPEN)return Promise.reject(new Error("ROS bridge WebSocket is not connected"));
  const id="web_"+Date.now().toString(36)+"_"+(++requestSequence).toString(36);
  return new Promise((resolve,reject)=>{
    const timeout=window.setTimeout(()=>{
      pending.delete(id);reject(new Error("Request timed out: "+method));
    },method==="motion.plan"?60000:20000);
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
function handleEvent(channel:string,data:any){
  const store=useRobotStore.getState();
  switch(channel){
    case "robot.status":store.setRobotStatus(data);break;
    case "robot.mode":store.setRobotMode(data);break;
    case "robot.joints":store.setJoints(data);break;
    case "robot.tcp":store.setTcp(data);break;
    case "robot.errors":store.setErrors(data);break;
    case "robot.state":if(data?.command)store.setCommand(data.command,data.response);break;
    case "motion.plan":store.setPlan({...store.plan,...data});break;
    case "motion.execution":store.setExecutionStatus(data?.status??null);break;
    case "moveit.scene":
      store.setSceneLoading(true);
      void request("moveit.get_scene").then((scene)=>useRobotStore.getState().setScene(scene))
        .catch((error)=>useRobotStore.getState().setSceneLoading(false,String(error)));
      break;
    default:break;
  }
}
async function loadInitialState(){
  const store=useRobotStore.getState();
  store.setSceneLoading(true);store.setModelLoading(true);
  const tasks=[
    request("robot.get_snapshot").then((value)=>useRobotStore.getState().mergeSnapshot(value)),
    request("moveit.get_scene").then((value)=>useRobotStore.getState().setScene(value))
      .catch((error)=>useRobotStore.getState().setSceneLoading(false,String(error))),
    request("robot.get_model").then((value)=>useRobotStore.getState().setModel(value))
      .catch((error)=>useRobotStore.getState().setModelLoading(false,String(error))),
    request("robot.get_joint_limits").then((value)=>useRobotStore.getState().setJointLimits(value)).catch(()=>undefined)
  ];
  await Promise.allSettled(tasks);
}
export function startSocket(){
  stopRequested=false;
  let firstAttempt=true;
  const scheduleReconnect=()=>{
    if(stopRequested||reconnectTimer!==undefined)return;
    const delay=Math.min(1000*Math.pow(1.6,retryCount++),8000);
    reconnectTimer=window.setTimeout(()=>{reconnectTimer=undefined;connect();},delay);
  };
  const connect=()=>{
    if(stopRequested)return;
    useRobotStore.getState().setBridgeState(firstAttempt?"connecting":"reconnecting");
    firstAttempt=false;
    try{socket=new WebSocket(wsUrl());}
    catch(error){
      useRobotStore.getState().setBridgeState("disconnected",String(error));scheduleReconnect();return;
    }
    socket.onopen=()=>{
      retryCount=0;useRobotStore.getState().setBridgeState("connected");
      socket?.send(JSON.stringify({op:"subscribe",channels:[
        "robot.status","robot.mode","robot.joints","robot.tcp","robot.errors",
        "robot.state","moveit.scene","motion.plan","motion.execution","bridge.health"
      ]}));
      void loadInitialState();
      pingTimer=window.setInterval(()=>{
        if(socket?.readyState===WebSocket.OPEN)socket.send(JSON.stringify({op:"ping"}));
      },15000);
    };
    socket.onmessage=(event)=>{
      let message:any;
      try{message=JSON.parse(event.data);}catch{return;}
      if(message.op==="response"){
        const item=pending.get(String(message.id));if(!item)return;
        window.clearTimeout(item.timeout);pending.delete(String(message.id));
        if(message.ok)item.resolve(message.data);
        else item.reject(new Error(message.error?.message||"WebSocket request failed"));
      }else if(message.op==="event")handleEvent(message.channel,message.data);
    };
    socket.onerror=()=>useRobotStore.getState().setBridgeState("reconnecting","WebSocket connection error");
    socket.onclose=()=>{
      if(pingTimer!==undefined)window.clearInterval(pingTimer);pingTimer=undefined;
      rejectPending("WebSocket disconnected");socket=null;
      if(!stopRequested){
        useRobotStore.getState().setBridgeState("reconnecting","Waiting for the ROS bridge");scheduleReconnect();
      }
    };
  };
  connect();
  return ()=>{
    stopRequested=true;
    if(reconnectTimer!==undefined)window.clearTimeout(reconnectTimer);
    if(pingTimer!==undefined)window.clearInterval(pingTimer);
    reconnectTimer=undefined;pingTimer=undefined;rejectPending("WebSocket client stopped");
    const current=socket;socket=null;current?.close();
    useRobotStore.getState().setBridgeState("disconnected");
  };
}
