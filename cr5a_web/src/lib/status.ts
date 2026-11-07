import type { BridgeState, RobotModeMessage } from "./types";

export type RobotStatusKey = "error" | "idle" | "paused" | "moving" | "planning" | "connecting" | "disabled" | "disconnected" | "unknown";
export interface RobotStatusInfo { key: RobotStatusKey; label: string; hint: string; }

export const ROBOT_STATUS: Record<RobotStatusKey, RobotStatusInfo> = {
  error:{key:"error",label:"Error",hint:"Controller reports an active error"},
  idle:{key:"idle",label:"Idle",hint:"Connected and enabled"},
  paused:{key:"paused",label:"Paused",hint:"Robot motion is paused"},
  moving:{key:"moving",label:"Moving",hint:"Robot motion is active"},
  planning:{key:"planning",label:"Planning",hint:"MoveIt is computing a trajectory"},
  connecting:{key:"connecting",label:"Connecting",hint:"Connecting to the ROS bridge"},
  disabled:{key:"disabled",label:"Disabled",hint:"Robot is connected but not enabled"},
  disconnected:{key:"disconnected",label:"Disconnected",hint:"Robot controller is not connected"},
  unknown:{key:"unknown",label:"Awaiting data",hint:"Waiting for reliable robot telemetry"}
};

function errorPresent(value: unknown): boolean {
  if (typeof value === "number") return Number.isFinite(value) && value !== 0;
  if (Array.isArray(value)) return value.some(errorPresent);
  if (value && typeof value === "object") return Object.values(value as Record<string,unknown>).some(errorPresent);
  if (typeof value !== "string") return false;
  const text = value.trim();
  if (!text || text === "[]" || text.toLowerCase() === "none" || text.toLowerCase().includes("no error")) return false;
  try { return errorPresent(JSON.parse(text)); }
  catch {
    const numbers = text.match(/-?\d+(?:\.\d+)?/g);
    return numbers ? numbers.some((item) => Number(item) !== 0) : false;
  }
}
interface StatusInput {
  bridgeState: BridgeState; connected?: boolean; enabled?: boolean;
  mode?: RobotModeMessage | null; errors?: Record<string,unknown> | null;
  lastCommand?: string | null; planningStatus?: string | null;
  executionStatus?: string | null; lastJointChangeAt?: number;
}
export function deriveRobotStatus(input: StatusInput): RobotStatusInfo {
  if (input.bridgeState !== "connected") {
    return input.bridgeState === "connecting" || input.bridgeState === "reconnecting" ? ROBOT_STATUS.connecting : ROBOT_STATUS.disconnected;
  }
  const payload = input.errors?.GetErrorID ?? input.errors?.robot_return ?? input.errors;
  if (errorPresent(payload)) return ROBOT_STATUS.error;
  const mode = String(input.mode?.value ?? "").toLowerCase();
  if (mode.includes("pause") || input.lastCommand === "pause") return ROBOT_STATUS.paused;
  const recentMotion = input.lastJointChangeAt !== undefined && Date.now() - input.lastJointChangeAt < 650;
  if (mode.includes("running") || mode.includes("moving") || mode.includes("jog") || input.executionStatus === "EXECUTING" || recentMotion) return ROBOT_STATUS.moving;
  if (input.planningStatus === "PLANNING") return ROBOT_STATUS.planning;
  if (input.connected === false) return ROBOT_STATUS.disconnected;
  if (input.connected === true && input.enabled === false) return ROBOT_STATUS.disabled;
  if (input.connected === true && input.enabled === true) return ROBOT_STATUS.idle;
  return ROBOT_STATUS.unknown;
}
export function hasActiveError(errors?: Record<string,unknown> | null): boolean {
  return errorPresent(errors?.GetErrorID ?? errors?.robot_return ?? errors);
}
export function statusClass(key: RobotStatusKey) { return "status-" + key; }
