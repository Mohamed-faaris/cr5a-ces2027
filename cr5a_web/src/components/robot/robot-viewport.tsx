import {Suspense,useCallback,useEffect,useRef,useState,type RefObject} from "react";
import {Canvas,useFrame} from "@react-three/fiber";
import {Grid,OrbitControls} from "@react-three/drei";
import * as THREE from "three";
import URDFLoader from "urdf-loader";
import type {CollisionObject,JointStateMessage,MotionPlan,RobotModel} from "@/lib/types";

type RobotObject=THREE.Object3D&{
  setJointValues:(values:Record<string,number>)=>void;
  links:Record<string,THREE.Object3D>;
  joints:Record<string,THREE.Object3D>;
};

type StickSegment={
  rod:THREE.Mesh;
  marker:THREE.Mesh;
  parent:THREE.Object3D;
  child:THREE.Object3D;
};

function RobotModelNode({
  model,jointState,onLoaded,showRods,showMeshes
}:{
  model:RobotModel;
  jointState:JointStateMessage|null;
  onLoaded:(robot:RobotObject|null,error?:string)=>void;
  showRods:boolean;
  showMeshes:boolean;
}) {
  const [robot,setRobot]=useState<RobotObject|null>(null);
  const segments=useRef<StickSegment[]>([]);
  const stickGroup=useRef<THREE.Group|null>(null);

  useEffect(()=>{
    let active=true;
    setRobot(null);
    onLoaded(null);
    try{
      const loader=new URDFLoader() as URDFLoader&{
        parseVisual:boolean;
        parseCollision:boolean;
        parse:(content:string)=>RobotObject;
        packages:Record<string,string>;
      };
      loader.parseVisual=true;
      loader.parseCollision=false;
      loader.packages={
        dobot_rviz:new URL("/assets/dobot_rviz",window.location.origin).toString()
      };
      const modelObject=loader.parse(model.urdf);
      const group=new THREE.Group();
      group.name="CR5 rod skeleton";
      const rodMaterial=new THREE.MeshStandardMaterial({color:0x9299a3,metalness:0.12,roughness:0.62});
      const jointMaterial=new THREE.MeshStandardMaterial({color:0x2563eb,metalness:0.08,roughness:0.48});
      const rods:StickSegment[]=[];
      Object.values(modelObject.joints).forEach((joint)=>{
        const parent=joint.parent;
        const child=joint.children.find((item)=>(item as THREE.Object3D&{isURDFLink?:boolean}).isURDFLink);
        if(!parent||!child)return;
        const rod=new THREE.Mesh(new THREE.CylinderGeometry(0.014,0.014,1,12),rodMaterial);
        const marker=new THREE.Mesh(new THREE.SphereGeometry(0.024,16,12),jointMaterial);
        rod.castShadow=true;
        marker.castShadow=true;
        group.add(rod,marker);
        rods.push({rod,marker,parent,child});
      });
      const baseMarker=new THREE.Mesh(new THREE.SphereGeometry(0.032,16,12),jointMaterial);
      baseMarker.position.set(0,0,0);
      group.add(baseMarker);
      modelObject.add(group);
      segments.current=rods;
      stickGroup.current=group;
      if(!active)return;
      setRobot(modelObject);
      onLoaded(modelObject);
    }catch(error){
      if(active)onLoaded(null,error instanceof Error?error.message:String(error));
    }
    return ()=>{
      active=false;
      segments.current.forEach(({rod,marker})=>{
        rod.geometry.dispose();
        marker.geometry.dispose();
      });
      segments.current=[];
      stickGroup.current=null;
    };
  },[model.urdf,onLoaded]);

  useEffect(()=>{
    if(!robot)return;
    robot.traverse((node)=>{
      if((node as THREE.Object3D&{isURDFVisual?:boolean}).isURDFVisual){
        node.visible=showMeshes;
        node.traverse((child)=>{child.visible=showMeshes;});
      }
    });
  },[robot,showMeshes]);

  useFrame(()=>{
    if(!robot)return;
    const values:Record<string,number>={};
    (jointState?.name??[]).forEach((name,index)=>{
      const value=jointState?.position?.[index];
      if(Number.isFinite(value))values[name]=value as number;
    });
    if(Object.keys(values).length)robot.setJointValues(values);
    robot.updateMatrixWorld(true);
    const group=stickGroup.current;
    if(!group)return;
    group.visible=showRods;
    const up=new THREE.Vector3(0,1,0);
    segments.current.forEach(({rod,marker,parent,child})=>{
      const start=robot.worldToLocal(parent.getWorldPosition(new THREE.Vector3()));
      const end=robot.worldToLocal(child.getWorldPosition(new THREE.Vector3()));
      const delta=end.clone().sub(start);
      const length=delta.length();
      marker.position.copy(end);
      rod.visible=length>0.004;
      if(length<=0.004)return;
      rod.position.copy(start).add(end).multiplyScalar(0.5);
      rod.quaternion.setFromUnitVectors(up,delta.normalize());
      rod.scale.set(1,length,1);
    });
  });
  return robot?<primitive object={robot}/>:null;
}

function isFiniteNumber(value:unknown):value is number{
  return typeof value==="number"&&Number.isFinite(value);
}

function ScenePrimitive({object}:{object:CollisionObject}) {
  const primitive=object.primitives?.[0];
  const pose=object.primitive_poses?.[0];
  if(!primitive||!pose)return null;
  const positionValues=[pose.position.x,pose.position.y,pose.position.z];
  const quaternionValues=[
    pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w
  ];
  const dimensions=primitive.dimensions??[];
  if(!positionValues.every(isFiniteNumber)||!quaternionValues.every(isFiniteNumber)||
    !dimensions.every(isFiniteNumber)||dimensions.some((value)=>value<=0))return null;
  const position=positionValues as [number,number,number];
  const quaternion=quaternionValues as [number,number,number,number];
  return <mesh position={position} quaternion={quaternion}>
    {primitive.type===1&&dimensions.length>=3&&<boxGeometry args={[dimensions[0],dimensions[1],dimensions[2]]}/>}
    {primitive.type===2&&dimensions.length>=1&&<sphereGeometry args={[dimensions[0],24,16]}/>}
    {primitive.type===3&&dimensions.length>=2&&<cylinderGeometry args={[dimensions[1],dimensions[1],dimensions[0],24]}/>}
    <meshStandardMaterial color="#737982" transparent opacity={0.36} roughness={0.8}/>
  </mesh>;
}

function TrajectoryPreview({
  robot,model,plan,jointState,rootRef
}:{
  robot:RobotObject|null;model:RobotModel;plan:MotionPlan|null;
  jointState:JointStateMessage|null;rootRef:RefObject<THREE.Group|null>;
}) {
  const [line,setLine]=useState<THREE.Line|null>(null);
  useEffect(()=>{
    if(!robot||!plan?.trajectory?.joint_trajectory?.points?.length||!rootRef.current){
      setLine(null);return;
    }
    const trajectory=plan.trajectory.joint_trajectory;
    const names:string[]=trajectory.joint_names??[];
    const points=trajectory.points??[];
    const actual:Record<string,number>={};
    (jointState?.name??[]).forEach((name,index)=>{
      const value=jointState?.position?.[index];
      if(isFiniteNumber(value))actual[name]=value;
    });
    if(!names.length||names.some((name)=>!isFiniteNumber(actual[name]))||
      !points.length||!points.every((point:any)=>Array.isArray(point.positions)&&
        point.positions.length===names.length&&point.positions.every(isFiniteNumber))){
      setLine(null);
      return;
    }
    const root=rootRef.current;
    const path:THREE.Vector3[]=[];
    try{
      points.forEach((point:any)=>{
        const values:Record<string,number>={};
        names.forEach((name,index)=>{values[name]=point.positions[index];});
        robot.setJointValues(values);robot.updateMatrixWorld(true);
        const link=robot.links[model.end_effector_link];
        if(link){
          const position=new THREE.Vector3();link.getWorldPosition(position);
          path.push(root.worldToLocal(position));
        }
      });
    }finally{
      robot.setJointValues(actual);robot.updateMatrixWorld(true);
    }
    if(path.length<2){setLine(null);return;}
    const geometry=new THREE.BufferGeometry().setFromPoints(path);
    const material=new THREE.LineBasicMaterial({color:0x2563eb});
    const next=new THREE.Line(geometry,material);
    setLine(next);
    return ()=>{geometry.dispose();material.dispose();};
  },[robot,model.end_effector_link,plan,jointState,rootRef]);
  return line?<primitive object={line}/>:null;
}

export function RobotViewport({
  model,jointState,scene,plan,loading,error
}:{
  model:RobotModel|null;jointState:JointStateMessage|null;scene:any;
  plan:MotionPlan|null;loading:boolean;error:string|null;
}) {
  const rootRef=useRef<THREE.Group|null>(null);
  const [robot,setRobot]=useState<RobotObject|null>(null);
  const [modelError,setModelError]=useState<string|null>(null);
  const [showRods,setShowRods]=useState(true);
  const [showMeshes,setShowMeshes]=useState(false);
  const [showPath,setShowPath]=useState(true);
  const [showAxes,setShowAxes]=useState(false);
  const [showGrid,setShowGrid]=useState(true);
  const [showSceneObjects,setShowSceneObjects]=useState(true);
  const allObjects:CollisionObject[]=scene?.world?.collision_objects??[];
  const hasTrajectory=(plan?.status==="SUCCEEDED"||plan?.status==="EXECUTING")&&
    (plan?.trajectory?.joint_trajectory?.points?.length??0)>1;
  const worldObjects=allObjects.filter((item)=>!item.header?.frame_id||item.header.frame_id===model?.base_frame);
  const attachedCount=scene?.robot_state?.attached_collision_objects?.length??0;
  const onLoaded=useCallback((value:RobotObject|null,message?:string)=>{
    setRobot(value);setModelError(message??null);
  },[]);

  return <div className="robot-viewport relative">
    <div className="absolute left-4 top-4 z-10 flex flex-wrap gap-2">
      <span className="viewport-chip">MODEL / {model?.group_name??"CR5 GROUP"}</span>
      <span className="viewport-chip">WORLD / {allObjects.length}</span>
      {attachedCount>0&&<span className="viewport-chip">ATTACHED / {attachedCount}</span>}
    </div>
    <div className="absolute right-3 top-3 z-10 flex flex-wrap justify-end gap-1.5">
      {([
        ["Rods",showRods,()=>setShowRods((value)=>!value),false],
        ["Meshes",showMeshes,()=>setShowMeshes((value)=>!value),false],
        ["Path",showPath,()=>setShowPath((value)=>!value),!hasTrajectory],
        ["Axes",showAxes,()=>setShowAxes((value)=>!value),false],
        ["Grid",showGrid,()=>setShowGrid((value)=>!value),false],
        ["Objects",showSceneObjects,()=>setShowSceneObjects((value)=>!value),false],
      ] as const).map(([label,active,onClick,disabled])=><button key={label} type="button" aria-pressed={active} onClick={onClick} disabled={disabled}
        className={disabled?"cursor-not-allowed rounded-md border border-zinc-100 bg-zinc-50 px-2.5 py-1.5 text-[10px] font-medium text-zinc-300":active?"rounded-md border border-zinc-800 bg-zinc-900 px-2.5 py-1.5 text-[10px] font-medium text-white shadow-sm":"rounded-md border border-zinc-200 bg-white/90 px-2.5 py-1.5 text-[10px] font-medium text-zinc-500 hover:border-zinc-400"}>
        {label}
      </button>)}
    </div>
    <div className="absolute bottom-4 left-4 z-10 text-[10px] text-zinc-400">Drag to orbit · Scroll to zoom · Right-drag to pan</div>
    {loading&&<div className="absolute inset-0 z-20 flex items-center justify-center bg-white/85 text-sm text-zinc-500">Loading robot description…</div>}
    {!loading&&!model&&<div className="absolute inset-0 z-20 flex flex-col items-center justify-center gap-2 px-6 text-center">
      <div className="text-sm font-medium text-zinc-800">Robot model unavailable</div>
      <div className="max-w-sm text-xs text-zinc-500">{error??"Start robot_state_publisher and confirm robot_description is available."}</div>
    </div>}
    {modelError&&<div className="absolute bottom-10 right-4 z-20 max-w-xs rounded-md border border-red-200 bg-red-50 px-3 py-2 text-xs text-red-700">Robot model parse failed: {modelError}</div>}
    {allObjects.some((item)=>item.header?.frame_id&&item.header.frame_id!==model?.base_frame)&&
      <div className="absolute bottom-10 right-4 z-10 rounded-md border border-amber-200 bg-amber-50 px-2.5 py-1.5 text-[10px] text-amber-800">
        Objects outside {model?.base_frame??"base frame"} are listed but not drawn without TF transforms.
      </div>}
    <Canvas shadows camera={{position:[1.25,1.05,1.35],fov:42,near:0.01,far:100}} dpr={[1,1.75]} gl={{antialias:true,alpha:false}}>
      <color attach="background" args={["#fafafa"]}/>
      <ambientLight intensity={1.1}/>
      <directionalLight position={[2,4,2]} intensity={2.1} castShadow/>
      <directionalLight position={[-2,2,-3]} intensity={0.65}/>
      <Suspense fallback={null}>
        <group ref={rootRef} rotation={[-Math.PI/2,0,0]}>
          {model&&<RobotModelNode model={model} jointState={jointState} onLoaded={onLoaded} showRods={showRods} showMeshes={showMeshes}/>}
          {showSceneObjects&&worldObjects.map((object)=><ScenePrimitive key={object.id} object={object}/>)}
          {showPath&&hasTrajectory&&robot&&model&&<TrajectoryPreview robot={robot} model={model} plan={plan} jointState={jointState} rootRef={rootRef}/>}
          {showAxes&&<axesHelper args={[0.18]}/>}
        </group>
      </Suspense>
      {showGrid&&<Grid position={[0,-0.002,0]} args={[3,3]} cellSize={0.1} cellThickness={0.5} sectionSize={0.5} sectionThickness={0.9} fadeDistance={4} fadeStrength={1.3} infiniteGrid/>}
      <OrbitControls makeDefault target={[0,0.25,0]} minDistance={0.45} maxDistance={4}/>
    </Canvas>
    <div className="absolute bottom-4 right-4 z-10 rounded-md border border-zinc-200 bg-white/80 px-2.5 py-1.5 text-[10px] text-zinc-500">{model?.base_frame??"dummy_link"} · metres</div>
  </div>;
}
