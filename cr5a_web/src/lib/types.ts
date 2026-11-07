export type BridgeState = "connecting" | "connected" | "reconnecting" | "disconnected";
export interface JointStateMessage { name?: string[]; position?: number[]; velocity?: number[]; effort?: number[]; }
export interface TcpMessage { x?: number; y?: number; z?: number; rx?: number; ry?: number; rz?: number; }
export interface RobotModeMessage { value?: string; res?: number; }
export interface RobotStatusMessage { is_connected?: boolean; is_enable?: boolean; }
export interface RobotCapabilities {
  robot_commands_enabled?: boolean;
  error_clear_enabled?: boolean;
  motion_execution_enabled?: boolean;
}
export interface RobotModel {
  urdf: string; srdf?: string; mesh_prefix: string; mesh_base_url: string;
  base_frame: string; end_effector_link: string; group_name: string;
}
export interface CollisionObject {
  id: string; header?: { frame_id?: string };
  primitives?: Array<{ type: number; dimensions: number[] }>;
  primitive_poses?: Array<{
    position: { x: number; y: number; z: number };
    orientation: { x: number; y: number; z: number; w: number };
  }>;
}
export interface PlanningScene {
  world?: { collision_objects?: CollisionObject[] };
  robot_state?: { attached_collision_objects?: Array<{ link_name?: string; object?: CollisionObject }> };
}
export interface MotionPlan {
  plan_id?: string; status?: string; error_code?: number; planning_time?: number; trajectory?: any;
}
