import {Suspense,useCallback,useEffect,useRef,useState,type RefObject} from "react";
import {Canvas,useFrame} from "@react-three/fiber";
import {Grid,OrbitControls} from "@react-three/drei";
import * as THREE from "three";
import URDFLoader from "urdf-loader";
import type {CollisionObject,JointStateMessage,MotionPlan,RobotModel} from "@/lib/types";

type RobotObject=THREE.Object3D&{
  setJointValues:(values:Record<string,number>)=>void;
  links:Record<string,THREE.Object3D>;
};

function RobotModelNode({
  model,jointState,onLoaded
}:{
  model:RobotModel;
  jointState:JointStateMessage|null;
  onLoaded:(robot:RobotObject|null,error?:string)=>void;
}) {
  const [robot,setRobot]=useState<RobotObject|null>(null);
  useEffect(()=>{
    let active=true;
    const loader=new URDFLoader();
    const meshBase=new URL(model.mesh_base_url,window.location.origin).toString();
    const urdf=model.urdf.split(model.mesh_prefix).join(meshBase);
    const blob=new Blob([urdf],{type:"application/xml"});
    const objectUrl=URL.createObjectURL(blob);
    loader.load(objectUrl,(loaded)=>{
      URL.revokeObjectURL(objectUrl);
      if(!active)return;
      const modelObject=loaded as RobotObject;
      modelObject.traverse((item)=>{
        const mesh=item as THREE.Mesh;
        if(mesh.isMesh){
          mesh.castShadow=true;mesh.receiveShadow=true;
          mesh.material=new THREE.MeshStandardMaterial({color:0xb7bbc2,metalness:0.16,roughness:0.58});
        }
      });
      setRobot(modelObject);onLoaded(modelObject);
    },undefined,(error)=>{
      URL.revokeObjectURL(objectUrl);
      if(active)onLoaded(null,error instanceof Error?error.message:String(error));
    });
    return ()=>{active=false;URL.revokeObjectURL(objectUrl);};
  },[model.urdf,model.mesh_base_url,model.mesh_prefix,onLoaded]);

  useFrame(()=>{
    if(!robot)return;
    const values:Record<string,number>={};
    (jointState?.name??[]).forEach((name,index)=>{
      const value=jointState?.position?.[index];
      if(Number.isFinite(value))values[name]=value as number;
    });
    if(Object.keys(values).length)robot.setJointValues(values);
    robot.updateMatrixWorld(true);
  });
  return robot?<primitive object={robot}/>:null;
}

function ScenePrimitive({object}:{object:CollisionObject}) {
  const primitive=object.primitives?.[0];
  const pose=object.primitive_poses?.[0];
  if(!primitive||!pose)return null;
  const position:[number,number,number]=[pose.position.x,pose.position.y,pose.position.z];
  const quaternion:[number,number,number,number]=[
    pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w
  ];
  const dimensions=primitive.dimensions??[];
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
    (jointState?.name??[]).forEach((name,index)=>{actual[name]=jointState?.position?.[index]??0;});
    const root=rootRef.current;
    const path:THREE.Vector3[]=[];
    try{
      points.forEach((point:any)=>{
        const values:Record<string,number>={};
        names.forEach((name,index)=>{values[name]=point.positions?.[index]??0;});
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
  const allObjects:CollisionObject[]=scene?.world?.collision_objects??[];
  const worldObjects=allObjects.filter((item)=>!item.header?.frame_id||item.header.frame_id===model?.base_frame);
  const attachedCount=scene?.robot_state?.attached_collision_objects?.length??0;
  const onLoaded=useCallback((value:RobotObject|null,message?:string)=>{
    setRobot(value);setModelError(message??null);
  },[]);

  return <div className="robot-viewport relative">
    <div className="absolute left-4 top-4 z-10 flex flex-wrap gap-2">
      <span className="viewport-chip">MODEL / {model?.group_name??"CR5 GROUP"}</span>
      <span className="viewport-chip">WORLD OBJECTS / {allObjects.length}</span>
      {attachedCount>0&&<span className="viewport-chip">ATTACHED / {attachedCount}</span>}
    </div>
    <div className="absolute bottom-4 left-4 z-10 text-[10px] text-zinc-400">Drag to orbit · Scroll to zoom · Right-drag to pan</div>
    {loading&&<div className="absolute inset-0 z-20 flex items-center justify-center bg-white/85 text-sm text-zinc-500">Loading robot description…</div>}
    {!loading&&!model&&<div className="absolute inset-0 z-20 flex flex-col items-center justify-center gap-2 px-6 text-center">
      <div className="text-sm font-medium text-zinc-800">Robot model unavailable</div>
      <div className="max-w-sm text-xs text-zinc-500">{error??"Start robot_state_publisher and confirm robot_description is available."}</div>
    </div>}
    {modelError&&<div className="absolute bottom-10 right-4 z-20 max-w-xs rounded-md border border-red-200 bg-red-50 px-3 py-2 text-xs text-red-700">Mesh load failed: {modelError}</div>}
    {allObjects.some((item)=>item.header?.frame_id&&item.header.frame_id!==model?.base_frame)&&
      <div className="absolute right-4 top-4 z-10 rounded-md border border-amber-200 bg-amber-50 px-2.5 py-1.5 text-[10px] text-amber-800">
        Objects outside {model?.base_frame??"base frame"} are listed but not drawn without TF transforms.
      </div>}
    <Canvas shadows camera={{position:[1.25,1.05,1.35],fov:42,near:0.01,far:100}} dpr={[1,1.75]} gl={{antialias:true,alpha:false}}>
      <color attach="background" args={["#fafafa"]}/>
      <ambientLight intensity={1.1}/>
      <directionalLight position={[2,4,2]} intensity={2.1} castShadow/>
      <directionalLight position={[-2,2,-3]} intensity={0.65}/>
      <Suspense fallback={null}>
        <group ref={rootRef} rotation={[-Math.PI/2,0,0]}>
          {model&&<RobotModelNode model={model} jointState={jointState} onLoaded={onLoaded}/>}
          {worldObjects.map((object)=><ScenePrimitive key={object.id} object={object}/>)}
          {robot&&model&&<TrajectoryPreview robot={robot} model={model} plan={plan} jointState={jointState} rootRef={rootRef}/>}
          <axesHelper args={[0.18]}/>
        </group>
      </Suspense>
      <Grid position={[0,-0.002,0]} args={[3,3]} cellSize={0.1} cellThickness={0.5} sectionSize={0.5} sectionThickness={0.9} fadeDistance={4} fadeStrength={1.3} infiniteGrid/>
      <OrbitControls makeDefault target={[0,0.25,0]} minDistance={0.45} maxDistance={4}/>
    </Canvas>
    <div className="absolute bottom-4 right-4 z-10 rounded-md border border-zinc-200 bg-white/80 px-2.5 py-1.5 text-[10px] text-zinc-500">{model?.base_frame??"dummy_link"} · metres</div>
  </div>;
}
