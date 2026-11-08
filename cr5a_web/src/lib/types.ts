export type BridgeState = "connecting" | "connected" | "reconnecting" | "disconnected";
export type TelemetryNumber = number | null;
export interface JointStateMessage {
  name?: string[];
  position?: TelemetryNumber[];
  velocity?: TelemetryNumber[];
  effort?: TelemetryNumber[];
}
export interface TcpMessage {
  x?: TelemetryNumber; y?: TelemetryNumber; z?: TelemetryNumber;
  rx?: TelemetryNumber; ry?: TelemetryNumber; rz?: TelemetryNumber;
}
export interface RobotModeMessage { value?: string; res?: number; }
export interface RobotStatusMessage { is_connected?: boolean; is_enable?: boolean; }
export interface RobotCapabilities {
  robot_commands_enabled?: boolean;
  error_clear_enabled?: boolean;
  motion_execution_enabled?: boolean;
  gripper_commands_enabled?: boolean;
  direct_movement_enabled?: boolean;
  movl_frame_confirmed?: boolean;
}
export interface RobotModel {
  urdf: string; pgea_urdf?: string|null; srdf?: string;
  base_frame: string; end_effector_link: string; group_name: string;
}
export interface CollisionObject {
  id: string; header?: { frame_id?: string };
  pose?: {
    position: { x: number; y: number; z: number };
    orientation: { x: number; y: number; z: number; w: number };
  };
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
  plan_id?: string; status?: string; error_code?: number | null; planning_time?: number | null; trajectory?: any;
}
