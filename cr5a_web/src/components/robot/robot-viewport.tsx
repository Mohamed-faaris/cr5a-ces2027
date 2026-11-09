import {Suspense,useCallback,useEffect,useRef,useState,type Dispatch,type RefObject,type SetStateAction} from "react";
import {Canvas,useFrame} from "@react-three/fiber";
import {Billboard,Grid,OrbitControls,Text} from "@react-three/drei";
import {ChevronDown,Eye,Grid3X3} from "lucide-react";
import * as THREE from "three";
import URDFLoader from "urdf-loader";
import type {CollisionObject,JointStateMessage,MotionPlan,RobotModel} from "@/lib/types";
import {rpyDegreesToQuaternion} from "@/lib/utils";
import type {VisualizerViewPreferences} from "@/lib/visualizer-view-preferences";
import {useRobotSocket} from "@/lib/socket";
import {useRobotStore} from "@/store/robot-store";

type RobotObject=THREE.Object3D&{
  setJointValues:(values:Record<string,number>)=>void;
  links:Record<string,THREE.Object3D>;
  joints:Record<string,THREE.Object3D&{jointValue:number[];angle:number}>;
};

type StickSegment={
  rod:THREE.Mesh;
  marker:THREE.Mesh;
  parent:THREE.Object3D;
  child:THREE.Object3D;
};

function RobotModelNode({
  model,jointState,onLoaded,showRods,showMeshes,showGripperMeshes,previewPositions,gripperPositionMm
}:{
  model:RobotModel;
  jointState:JointStateMessage|null;
  onLoaded:(robot:RobotObject|null,error?:string)=>void;
  showRods:boolean;
  showMeshes:boolean;
  showGripperMeshes:boolean;
  previewPositions:Record<string,number>|null;
  gripperPositionMm:number|null;
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
      // Render the PGEA-specific URDF when its visual model is installed.
      // The live arm model and planning frames remain supplied by MoveIt.
      const modelObject=loader.parse(model.pgea_urdf??model.urdf);
      const group=new THREE.Group();
      group.name="CR5 rod skeleton";
      const rodMaterial=new THREE.MeshStandardMaterial({color:0x9299a3,metalness:0.12,roughness:0.62});
      const jointMaterial=new THREE.MeshStandardMaterial({color:0x2563eb,metalness:0.08,roughness:0.48});
      const rods:StickSegment[]=[];
      Object.entries(modelObject.joints).forEach(([jointName,joint])=>{
        if(!/^joint[1-6]$/.test(jointName))return;
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
  },[model.urdf,model.pgea_urdf,onLoaded]);

  useEffect(()=>{
    if(!robot)return;
    const gripperLinks=new Set(["connector_slabs","gripper_body","left_finger","right_finger"]);
    robot.traverse((node)=>{
      if(!(node as THREE.Object3D&{isURDFVisual?:boolean}).isURDFVisual)return;
      const parent=node.parent as (THREE.Object3D&{isURDFLink?:boolean})|null;
      const isGripperVisual=Boolean(parent?.isURDFLink&&gripperLinks.has(parent.name));
      const visible=isGripperVisual?showGripperMeshes:showMeshes;
      node.visible=visible;
      node.traverse((child)=>{child.visible=visible;});
    });
  },[robot,showMeshes,showGripperMeshes]);

  useFrame(()=>{
    if(!robot)return;
    const values:Record<string,number>={};
    if(previewPositions){
      Object.assign(values,previewPositions);
    }else{
      (jointState?.name??[]).forEach((name,index)=>{
        const value=jointState?.position?.[index];
        if(Number.isFinite(value))values[name]=value as number;
      });
    }
    if(gripperPositionMm!==null&&Number.isFinite(gripperPositionMm)){
      values.left_finger_joint=Math.max(0,Math.min(20,gripperPositionMm))/1000;
    }
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

function BaseAxes() {
  const axesRef=useRef<THREE.AxesHelper|null>(null);
  useEffect(()=>{
    const axes=axesRef.current;
    if(!axes)return;
    // Keep the base-frame axes visible over the robot base and scene geometry.
    const materials=Array.isArray(axes.material)?axes.material:[axes.material];
    materials.forEach((material)=>{
      material.depthTest=false;
      material.depthWrite=false;
      material.needsUpdate=true;
    });
    axes.renderOrder=20;
  },[]);
  return <axesHelper ref={axesRef} args={[0.4]} position={[0,0,0.04]}/>;
}

function TcpFrame({
  robot,linkName,visible
}:{robot:RobotObject|null;linkName:string;visible:boolean}) {
  useEffect(()=>{
    if(!robot||!linkName||!visible)return;
    const link=robot.links[linkName];
    if(!link)return;
    const axes=new THREE.AxesHelper(0.16);
    const axisMaterials=Array.isArray(axes.material)?axes.material:[axes.material];
    axisMaterials.forEach((material)=>{
      material.depthTest=false;
      material.depthWrite=false;
      material.needsUpdate=true;
    });
    axes.renderOrder=30;
    const markerGeometry=new THREE.SphereGeometry(0.009,16,12);
    const markerMaterial=new THREE.MeshBasicMaterial({
      color:0x111827,depthTest:false,depthWrite:false
    });
    const marker=new THREE.Mesh(markerGeometry,markerMaterial);
    marker.renderOrder=31;
    link.add(axes,marker);
    return ()=>{
      link.remove(axes,marker);
      axes.geometry.dispose();
      axisMaterials.forEach((material)=>material.dispose());
      markerGeometry.dispose();
      markerMaterial.dispose();
    };
  },[robot,linkName,visible]);
  return null;
}

type FrameTransform={translation:THREE.Vector3;rotation:THREE.Quaternion};

function resolveFrameTransform(topicData:any,sourceFrame:string,targetFrame:string):FrameTransform|null {
  if(!topicData||!sourceFrame||!targetFrame||sourceFrame===targetFrame)return null;
  const transforms=Array.isArray(topicData)?topicData:(topicData.transforms??[]);
  const normalize=(frame:string)=>frame.replace(/^\/+/,"");
  const source=normalize(sourceFrame),target=normalize(targetFrame);
  for(const item of transforms){
    const parent=normalize(String(item?.header?.frame_id??""));
    const child=normalize(String(item?.child_frame_id??""));
    const t=item?.transform?.translation,q=item?.transform?.rotation;
    if(!t||!q||![t.x,t.y,t.z,q.x,q.y,q.z,q.w].every(isFiniteNumber))continue;
    const translation=new THREE.Vector3(t.x,t.y,t.z);
    const rotation=new THREE.Quaternion(q.x,q.y,q.z,q.w);
    if(rotation.lengthSq()<1e-12)continue;
    rotation.normalize();
    // TF stores parent<-child. Scene coordinates are converted source->target.
    if(parent===target&&child===source)return {translation,rotation};
    if(parent===source&&child===target){
      const inverse=rotation.clone().invert();
      return {translation:translation.negate().applyQuaternion(inverse),rotation:inverse};
    }
  }
  return null;
}

function ScenePrimitive({object,showLabel,collisionEnabled,frameTransform}:{
  object:CollisionObject;showLabel:boolean;collisionEnabled:boolean;frameTransform?:FrameTransform|null;
}) {

  const primitives=object.primitives??[];
  const poses=object.primitive_poses??[];
  if(!primitives.length)return null;

  // MoveIt primitive poses are relative to CollisionObject.pose. The bridge
  // stores the actual XYZ in the parent pose and identity local primitive poses.
  const parentPosition=object.pose?.position;
  const parentOrientation=object.pose?.orientation;
  const parentPositionValues=[
    parentPosition?.x??0,parentPosition?.y??0,parentPosition?.z??0
  ];
  const parentQuaternionValues=[
    parentOrientation?.x??0,parentOrientation?.y??0,
    parentOrientation?.z??0,parentOrientation?.w??1
  ];
  if(!parentPositionValues.every(isFiniteNumber)||
    !parentQuaternionValues.every(isFiniteNumber)||
    parentQuaternionValues.every((value)=>value===0))return null;
  const parentPositionVector=new THREE.Vector3(...parentPositionValues);
  const parentQuaternion=new THREE.Quaternion(...parentQuaternionValues).normalize();
  if(frameTransform){
    parentPositionVector.applyQuaternion(frameTransform.rotation).add(frameTransform.translation);
    parentQuaternion.premultiply(frameTransform.rotation);
  }

  return <>
    {primitives.map((primitive,index)=>{
      const pose=poses[index];
      const localPositionValues=[
        pose?.position.x??0,pose?.position.y??0,pose?.position.z??0
      ];
      const localQuaternionValues=[
        pose?.orientation.x??0,pose?.orientation.y??0,
        pose?.orientation.z??0,pose?.orientation.w??1
      ];
      const dimensions=primitive.dimensions??[];
      if(!localPositionValues.every(isFiniteNumber)||
        !localQuaternionValues.every(isFiniteNumber)||
        localQuaternionValues.every((value)=>value===0)||
        !dimensions.every(isFiniteNumber)||dimensions.some((value)=>value<=0))return null;
      const localPosition=new THREE.Vector3(...localPositionValues);
      const localQuaternion=new THREE.Quaternion(...localQuaternionValues).normalize();
      const position=localPosition.applyQuaternion(parentQuaternion).add(parentPositionVector);
      const quaternion=parentQuaternion.clone().multiply(localQuaternion);
      const extentZ=primitive.type===1&&dimensions.length>=3?dimensions[2]/2:
        primitive.type===2&&dimensions.length>=1?dimensions[0]:
        primitive.type===3&&dimensions.length>=2?dimensions[0]/2:0;
      const supported=(primitive.type===1&&dimensions.length>=3)||
        (primitive.type===2&&dimensions.length>=1)||
        (primitive.type===3&&dimensions.length>=2);
      if(!supported)return null;
      return <group key={object.id+"-"+index} position={position} quaternion={quaternion}>
        <mesh rotation={primitive.type===3?[Math.PI/2,0,0]:[0,0,0]}>
          {primitive.type===1&&<boxGeometry args={[dimensions[0],dimensions[1],dimensions[2]]}/>}
          {primitive.type===2&&<sphereGeometry args={[dimensions[0],24,16]}/>}
          {primitive.type===3&&<cylinderGeometry args={[dimensions[1],dimensions[1],dimensions[0],24]}/>}
          <meshStandardMaterial
            color={collisionEnabled?"#64748b":"#f59e0b"}
            emissive={collisionEnabled?"#1d4ed8":"#b45309"}
            emissiveIntensity={collisionEnabled?0.08:0.16}
            transparent opacity={collisionEnabled?0.36:0.48} roughness={0.8}/>
        </mesh>
        {showLabel&&index===0&&<Billboard position={[0,0,extentZ+0.025]}>
          <Text fontSize={0.035} color="#111827" outlineColor="#ffffff" outlineWidth={0.0025}
            anchorX="center" anchorY="bottom">
            {object.id}
          </Text>
        </Billboard>}
      </group>;
    })}
  </>;
}

function TrajectoryPreview({
  robot,model,plan,rootRef,color
}:{
  robot:RobotObject|null;model:RobotModel;plan:MotionPlan|null;
  rootRef:RefObject<THREE.Group|null>;color:string;
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
    Object.entries(robot.joints).forEach(([name,joint])=>{
      if(joint.jointValue?.length===1&&isFiniteNumber(joint.angle))actual[name]=joint.angle;
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
        const values:Record<string,number>={...actual};
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
    const material=new THREE.LineBasicMaterial({color});
    const next=new THREE.Line(geometry,material);
    setLine(next);
    return ()=>{geometry.dispose();material.dispose();};
  },[robot,model.end_effector_link,plan,rootRef]);
  return line?<primitive object={line}/>:null;
}

export function RobotViewport({
  model,jointState,scene,plan,comparisonPaths=[],targetTcp=null,gripperPositionMm=null,poseMarkers=[],loading,error,viewPreferences,onViewPreferencesChange,
  objectLabels,disabledObjects,hiddenObjects
}:{
  model:RobotModel|null;jointState:JointStateMessage|null;scene:any;
  plan:MotionPlan|null;comparisonPaths?:Array<{key:string;label:string;color:string;plan:MotionPlan}>;
  poseMarkers?:Array<{id:string;label:string;position:[number,number,number];orientation?:[number,number,number,number];frame_id?:string;color?:string;selected?:boolean}>;
  targetTcp?:{x:number;y:number;z:number;rx:number;ry:number;rz:number}|null;
  gripperPositionMm?:number|null;loading:boolean;error:string|null;
  viewPreferences:VisualizerViewPreferences;
  onViewPreferencesChange:Dispatch<SetStateAction<VisualizerViewPreferences>>;
  objectLabels:Record<string,boolean>;
  disabledObjects:Record<string,CollisionObject>;hiddenObjects:Record<string,boolean>;
}) {
  const rootRef=useRef<THREE.Group|null>(null);
  const [robot,setRobot]=useState<RobotObject|null>(null);
  const {subscribeChannels}=useRobotSocket();
  const tfStatic=useRobotStore((state)=>state.topicMessages["/tf_static"]?.data);
  useEffect(()=>subscribeChannels(["topic:/tf_static"]),[subscribeChannels]);
  const [modelError,setModelError]=useState<string|null>(null);
  const showRods=viewPreferences.robotStyle==="skeleton";
  const showMeshes=viewPreferences.robotStyle==="mesh";
  const {showGripperMeshes,showTrajectoryPaths:showPath,showBaseAxes:showAxes,showActualTcp:showTcp,
    showTargetTcp,showGrid,showSceneObjects,showObjectLabels:showLabels}=viewPreferences;
  const updateView=(patch:Partial<VisualizerViewPreferences>)=>
    onViewPreferencesChange(current=>({...current,...patch}));
  const [trajectoryPosition,setTrajectoryPosition]=useState<number|null>(null);
  const activeObjects:CollisionObject[]=scene?.world?.collision_objects??[];
  const allObjects:CollisionObject[]=[
    ...activeObjects,
    ...Object.values(disabledObjects).filter((object)=>!activeObjects.some((item)=>item.id===object.id))
  ];
  const trajectory=plan?.trajectory?.joint_trajectory;
  const trajectoryNames:string[]=trajectory?.joint_names??[];
  const trajectoryPoints:any[]=trajectory?.points??[];
  const hasTrajectory=(plan?.status==="SUCCEEDED"||plan?.status==="EXECUTING")&&
    trajectoryNames.length>0&&trajectoryPoints.length>1&&
    trajectoryPoints.every((point)=>Array.isArray(point.positions)&&
      point.positions.length===trajectoryNames.length&&point.positions.every(isFiniteNumber));
  const selectedPosition=trajectoryPosition===null?null:Math.max(0,Math.min(trajectoryPoints.length-1,trajectoryPosition));
  const lowerIndex=selectedPosition===null?0:Math.floor(selectedPosition);
  const upperIndex=Math.min(lowerIndex+1,trajectoryPoints.length-1);
  const interpolation=selectedPosition===null?0:selectedPosition-lowerIndex;
  const lowerPoint=trajectoryPoints[lowerIndex];
  const upperPoint=trajectoryPoints[upperIndex];
  const previewPositions:Record<string,number>|null=selectedPosition===null||!hasTrajectory?null:
    Object.fromEntries(trajectoryNames.map((name,index)=>[
      name,
      lowerPoint.positions[index]+(upperPoint.positions[index]-lowerPoint.positions[index])*interpolation
    ]));
  const timeSeconds=(duration:any)=>{
    const sec=Number(duration?.sec??0);
    const nanosec=Number(duration?.nanosec??0);
    return Number.isFinite(sec)&&Number.isFinite(nanosec)?sec+nanosec/1e9:0;
  };
  const selectedTime=selectedPosition===null?0:
    timeSeconds(lowerPoint?.time_from_start)+
    (timeSeconds(upperPoint?.time_from_start)-timeSeconds(lowerPoint?.time_from_start))*interpolation;
  const totalTime=timeSeconds(trajectoryPoints[trajectoryPoints.length-1]?.time_from_start);
  useEffect(()=>{setTrajectoryPosition(null);},[plan?.plan_id]);
  const frameTransformFor=(frameId?:string)=>{
    if(!frameId||frameId===model?.base_frame)return null;
    return resolveFrameTransform(tfStatic,frameId,model?.base_frame??"");
  };
  const renderObjects=allObjects.filter((item)=>
    hiddenObjects[item.id]!==true&&(!item.header?.frame_id||item.header.frame_id===model?.base_frame||Boolean(frameTransformFor(item.header.frame_id)))
  );
  const attachedCount=scene?.robot_state?.attached_collision_objects?.length??0;
  const onLoaded=useCallback((value:RobotObject|null,message?:string)=>{
    setRobot(value);setModelError(message??null);
  },[]);

  return <div className="robot-viewport relative">
    <div className="absolute left-4 top-4 z-10 flex flex-wrap gap-2">
      <span className="viewport-chip">{model?.group_name??"CR5 GROUP"}</span>
      <span className="viewport-chip">{renderObjects.length} objects · {poseMarkers.length} poses</span>
      {attachedCount>0&&<span className="viewport-chip">{attachedCount} attached</span>}
    </div>
    <div className="absolute right-3 top-3 z-10 flex items-start gap-1.5">
      <details className="group relative">
        <summary title="Show or hide robot geometry, gripper, scene objects, labels and paths" className="flex cursor-pointer list-none items-center gap-1.5 rounded-md border border-zinc-200 bg-white/95 px-2.5 py-2 text-[11px] font-medium text-zinc-700 shadow-sm">
          <Eye size={13}/> Display <ChevronDown size={12}/>
        </summary>
        <div className="absolute right-0 mt-1 w-52 space-y-1 rounded-lg border border-zinc-200 bg-white p-2 shadow-xl">
          <div className="px-2 pb-1 text-[9px] font-semibold uppercase tracking-wider text-zinc-400">Robot</div>
          <button type="button" title="Show the full URDF visual meshes for the arm." onClick={()=>updateView({robotStyle:"mesh"})} className={"w-full rounded px-2 py-1.5 text-left text-xs "+(showMeshes?"bg-zinc-900 text-white":"hover:bg-zinc-100")}>Visual mesh</button>
          <button type="button" title="Show a simplified joint-to-joint skeleton instead of the detailed meshes." onClick={()=>updateView({robotStyle:"skeleton"})} className={"w-full rounded px-2 py-1.5 text-left text-xs "+(showRods?"bg-zinc-900 text-white":"hover:bg-zinc-100")}>Joint skeleton</button>
          <button type="button" title="Toggle the PGEA gripper visual model. This does not connect or disconnect the gripper." aria-pressed={showGripperMeshes} onClick={()=>updateView({showGripperMeshes:!showGripperMeshes})} disabled={!model?.pgea_urdf} className={"w-full rounded px-2 py-1.5 text-left text-xs disabled:opacity-40 "+(showGripperMeshes?"bg-zinc-100":"hover:bg-zinc-100")}>Gripper model {showGripperMeshes?"· On":"· Off"}</button>
          <div className="my-1 border-t border-zinc-100"/>
          <button type="button" title="Hide or show scene collision geometry only. It does not change MoveIt's collision checking." aria-pressed={showSceneObjects} onClick={()=>updateView({showSceneObjects:!showSceneObjects})} className="flex w-full items-center justify-between rounded px-2 py-1.5 text-left text-xs hover:bg-zinc-100">Scene objects <span>{showSceneObjects?"On":"Off"}</span></button>
          <button type="button" title="Show or hide object name labels. This is a visual-only setting." aria-pressed={showLabels} onClick={()=>updateView({showObjectLabels:!showLabels})} className="flex w-full items-center justify-between rounded px-2 py-1.5 text-left text-xs hover:bg-zinc-100">Object labels <span>{showLabels?"On":"Off"}</span></button>
          <button type="button" title="Show or hide the planned end-effector trajectory. Compared profiles retain their own colors." aria-pressed={showPath} onClick={()=>updateView({showTrajectoryPaths:!showPath})} disabled={!hasTrajectory&&!comparisonPaths.length} className="flex w-full items-center justify-between rounded px-2 py-1.5 text-left text-xs hover:bg-zinc-100 disabled:opacity-40">Trajectory paths <span>{showPath?"On":"Off"}</span></button>
        </div>
      </details>
      <details className="group relative">
        <summary title="Show or hide coordinate references and the ground grid" className="flex cursor-pointer list-none items-center gap-1.5 rounded-md border border-zinc-200 bg-white/95 px-2.5 py-2 text-[11px] font-medium text-zinc-700 shadow-sm">
          <Grid3X3 size={13}/> View <ChevronDown size={12}/>
        </summary>
        <div className="absolute right-0 mt-1 w-48 space-y-1 rounded-lg border border-zinc-200 bg-white p-2 shadow-xl">
          <button type="button" title="Show base_link axes at the robot base. X/Y/Z axes are visual references." aria-pressed={showAxes} onClick={()=>updateView({showBaseAxes:!showAxes})} className="flex w-full items-center justify-between rounded px-2 py-1.5 text-left text-xs hover:bg-zinc-100">Base axes <span>{showAxes?"On":"Off"}</span></button>
          <button type="button" title="Show the actual TCP frame attached to the current robot model pose." aria-pressed={showTcp} onClick={()=>updateView({showActualTcp:!showTcp})} disabled={!robot||!model?.end_effector_link} className="flex w-full items-center justify-between rounded px-2 py-1.5 text-left text-xs hover:bg-zinc-100 disabled:opacity-40">Actual TCP frame <span>{showTcp?"On":"Off"}</span></button>
          <button type="button" title="Preview the entered TCP target as an orange marker and oriented axes in the MoveIt base frame. Controller-reported actual values may use a different tool/user frame or TCP offset. This does not move the robot." aria-pressed={showTargetTcp} onClick={()=>updateView({showTargetTcp:!showTargetTcp})} disabled={!targetTcp||![targetTcp.x,targetTcp.y,targetTcp.z,targetTcp.rx,targetTcp.ry,targetTcp.rz].every(isFiniteNumber)} className="flex w-full items-center justify-between rounded px-2 py-1.5 text-left text-xs hover:bg-zinc-100 disabled:opacity-40">Target TCP <span>{showTargetTcp?"On":"Off"}</span></button>
          <button type="button" title="Show or hide the world ground grid. This does not represent a collision object." aria-pressed={showGrid} onClick={()=>updateView({showGrid:!showGrid})} className="flex w-full items-center justify-between rounded px-2 py-1.5 text-left text-xs hover:bg-zinc-100">Ground grid <span>{showGrid?"On":"Off"}</span></button>
          <div className="border-t border-zinc-100 px-2 pt-2 text-[10px] leading-4 text-zinc-400">Drag to orbit · Scroll to zoom · Right-drag to pan</div>
        </div>
      </details>
    </div>
    {loading&&<div className="absolute inset-0 z-20 flex items-center justify-center bg-white/85 text-sm text-zinc-500">Loading robot description…</div>}
    {!loading&&!model&&<div className="absolute inset-0 z-20 flex flex-col items-center justify-center gap-2 px-6 text-center">
      <div className="text-sm font-medium text-zinc-800">Robot model unavailable</div>
      <div className="max-w-sm text-xs text-zinc-500">{error??"Start robot_state_publisher and confirm robot_description is available."}</div>
    </div>}
    {modelError&&<div className="absolute bottom-10 right-4 z-20 max-w-xs rounded-md border border-red-200 bg-red-50 px-3 py-2 text-xs text-red-700">Robot model parse failed: {modelError}</div>}
    {allObjects.some((item)=>item.header?.frame_id&&item.header.frame_id!==model?.base_frame&&!frameTransformFor(item.header.frame_id))&&
      <div className="absolute bottom-10 right-4 z-10 rounded-md border border-amber-200 bg-amber-50 px-2.5 py-1.5 text-[10px] text-amber-800">
        Objects outside {model?.base_frame??"base frame"} are listed but not drawn without TF transforms.
      </div>}
    {poseMarkers.some((item)=>item.frame_id&&item.frame_id!==model?.base_frame&&!frameTransformFor(item.frame_id))&&
      <div className="absolute bottom-10 left-4 z-10 max-w-xs rounded-md border border-amber-200 bg-amber-50 px-2.5 py-1.5 text-[10px] text-amber-800">
        Reference poses in {poseMarkers.find(item=>item.frame_id&&item.frame_id!==model?.base_frame&&!frameTransformFor(item.frame_id))?.frame_id} are hidden: no TF to {model?.base_frame??"the scene base"}.
      </div>}
    {hasTrajectory&&<div className="absolute bottom-3 left-3 right-3 z-20 rounded-lg border border-zinc-200 bg-white/95 px-3 py-2.5 shadow-lg backdrop-blur-sm">
      <div className="mb-1.5 flex items-center justify-between gap-3">
        <div className="flex min-w-0 items-center gap-2">
          <span className="text-[11px] font-semibold text-zinc-800">Trajectory preview</span>
          <span className="font-mono text-[10px] tabular-nums text-zinc-500">
            {selectedPosition===null?"LIVE POSE":"POINT "+(Math.round(selectedPosition)+1)+" / "+trajectoryPoints.length}
          </span>
          <span className="font-mono text-[10px] tabular-nums text-zinc-400">
            {selectedPosition===null?"":selectedTime.toFixed(2)+" / "+totalTime.toFixed(2)+" s"}
          </span>
        </div>
        <button type="button" onClick={()=>setTrajectoryPosition(null)} disabled={selectedPosition===null}
          className="shrink-0 rounded border border-zinc-200 px-2 py-1 text-[10px] font-medium text-zinc-600 hover:bg-zinc-50 disabled:cursor-default disabled:opacity-40">
          Live pose
        </button>
      </div>
      <input type="range" min={0} max={trajectoryPoints.length-1} step={0.01}
        value={selectedPosition??0} aria-label="Scrub planned robot trajectory"
        onChange={(event)=>setTrajectoryPosition(Number(event.target.value))}
        className="range-control w-full"/>
      <div className="mt-0.5 flex justify-between text-[9px] text-zinc-400">
        <span>Start · 0.00 s</span><span>{trajectoryPoints.length} points</span><span>End · {totalTime.toFixed(2)} s</span>
      </div>
    </div>}
    <Canvas shadows camera={{position:viewPreferences.cameraPosition,fov:42,near:0.01,far:100}} dpr={[1,1.75]} gl={{antialias:true,alpha:false}}>
      <color attach="background" args={["#fafafa"]}/>
      <ambientLight intensity={1.1}/>
      <directionalLight position={[2,4,2]} intensity={2.1} castShadow/>
      <directionalLight position={[-2,2,-3]} intensity={0.65}/>
      <Suspense fallback={null}>
        <group ref={rootRef} rotation={[-Math.PI/2,0,0]}>
          {model&&<RobotModelNode model={model} jointState={jointState} onLoaded={onLoaded}
            showRods={showRods} showMeshes={showMeshes} showGripperMeshes={showGripperMeshes}
            previewPositions={previewPositions} gripperPositionMm={gripperPositionMm}/>}
          <TcpFrame robot={robot} linkName={model?.pgea_urdf?"tcp":model?.end_effector_link??""} visible={showTcp}/>
          {showTargetTcp&&targetTcp&&[targetTcp.x,targetTcp.y,targetTcp.z,targetTcp.rx,targetTcp.ry,targetTcp.rz].every(isFiniteNumber)&&(()=>{
            const [qx,qy,qz,qw]=rpyDegreesToQuaternion(targetTcp.rx,targetTcp.ry,targetTcp.rz);
            return <group position={[targetTcp.x/1000,targetTcp.y/1000,targetTcp.z/1000]} quaternion={new THREE.Quaternion(qx,qy,qz,qw)}>
              <axesHelper args={[0.12]}/>
              <mesh renderOrder={32}>
                <sphereGeometry args={[0.012,18,14]}/>
                <meshBasicMaterial color="#f97316" depthTest={false}/>
              </mesh>
              <mesh renderOrder={32}>
                <ringGeometry args={[0.018,0.023,32]}/>
                <meshBasicMaterial color="#f97316" side={THREE.DoubleSide} depthTest={false}/>
              </mesh>
              <Billboard position={[0,0,0.045]}>
                <Text fontSize={0.018} color="#c2410c" outlineWidth={0.001} outlineColor="#ffffff">TARGET TCP</Text>
              </Billboard>
            </group>;
          })()}
          {showSceneObjects&&renderObjects.filter((object)=>hiddenObjects[object.id]!==true).map((object)=><ScenePrimitive key={object.id} object={object}
            frameTransform={frameTransformFor(object.header?.frame_id)}
            showLabel={showLabels&&objectLabels[object.id]!==false}
            collisionEnabled={activeObjects.some((active)=>active.id===object.id)}/>)}
          {poseMarkers.filter((item)=>item.position.every(isFiniteNumber)&&(!item.frame_id||item.frame_id===model?.base_frame||Boolean(frameTransformFor(item.frame_id)))).map((item,index)=>{
            const frameTransform=frameTransformFor(item.frame_id);
            const position=new THREE.Vector3(...item.position);
            const orientation=item.orientation?new THREE.Quaternion(...item.orientation):undefined;
            if(frameTransform){
              position.applyQuaternion(frameTransform.rotation).add(frameTransform.translation);
              orientation?.premultiply(frameTransform.rotation);
            }
            return <group key={item.id} position={position.toArray()} quaternion={orientation}>
              {item.orientation&&<axesHelper args={[0.065]}/> }
              <mesh renderOrder={35}><sphereGeometry args={[item.selected?0.018:0.012,16,12]}/><meshStandardMaterial color={item.color??(item.selected?"#f97316":"#0d9488")} emissive={item.color??(item.selected?"#c2410c":"#0f766e")} emissiveIntensity={0.35}/></mesh>
              <Billboard position={[0,0,0.035]}><Text fontSize={0.018} color={item.selected?"#c2410c":item.color??"#0f766e"} outlineWidth={0.0015} outlineColor="#ffffff">{item.label||("POSE "+(index+1))}</Text></Billboard>
            </group>;
          })}

          {showPath&&robot&&model&&comparisonPaths.length>0
            ?comparisonPaths.map((item)=><TrajectoryPreview key={item.key} robot={robot} model={model} plan={item.plan} rootRef={rootRef} color={item.color}/>)
            :showPath&&hasTrajectory&&robot&&model&&<TrajectoryPreview robot={robot} model={model} plan={plan} rootRef={rootRef} color="#2563eb"/>}
          {showAxes&&<BaseAxes/>}
        </group>
      </Suspense>
      {showGrid&&<Grid position={[0,-0.002,0]} args={[3,3]} cellSize={0.1} cellThickness={0.5} sectionSize={0.5} sectionThickness={0.9} fadeDistance={4} fadeStrength={1.3} infiniteGrid/>}
      <OrbitControls makeDefault target={viewPreferences.cameraTarget} minDistance={0.45} maxDistance={4}
        onEnd={(event:any)=>{
          const controls=event.target;
          if(!controls?.object?.position||!controls?.target)return;
          const cameraPosition=controls.object.position.toArray() as [number,number,number];
          const cameraTarget=controls.target.toArray() as [number,number,number];
          onViewPreferencesChange(current=>({...current,cameraPosition,cameraTarget}));
        }}/>
    </Canvas>
    <div className={hasTrajectory?"absolute bottom-24 right-4 z-10 rounded-md border border-zinc-200 bg-white/80 px-2.5 py-1.5 text-[10px] text-zinc-500":"absolute bottom-4 right-4 z-10 rounded-md border border-zinc-200 bg-white/80 px-2.5 py-1.5 text-[10px] text-zinc-500"}>
      {model?.base_frame??"dummy_link"} · metres
    </div>
  </div>;
}
