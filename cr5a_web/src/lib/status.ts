import type { BridgeState, RobotModeMessage } from "./types";

export type RobotStatusKey = "error" | "idle" | "paused" | "moving" | "planning" | "connecting" | "initializing" | "disabled" | "disconnected" | "unknown";
export interface RobotStatusInfo { key: RobotStatusKey; label: string; hint: string; }

export const ROBOT_STATUS: Record<RobotStatusKey, RobotStatusInfo> = {
  error:{key:"error",label:"Error",hint:"Controller reports an active error"},
  idle:{key:"idle",label:"Idle",hint:"Connected and enabled"},
  paused:{key:"paused",label:"Paused",hint:"Robot motion is paused"},
  moving:{key:"moving",label:"Moving",hint:"Robot motion is active"},
  planning:{key:"planning",label:"Planning",hint:"MoveIt is computing a trajectory"},
  connecting:{key:"connecting",label:"Connecting",hint:"Connecting to the ROS bridge"},
  initializing:{key:"initializing",label:"Initializing",hint:"Controller is initializing"},
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
function extractRobotModeCode(value?: string): number | null {
  if (!value) return null;
  const text = String(value).trim();
  const braced = text.match(/\{([^}]+)\}/);
  const source = braced ? braced[1] : text;
  const numbers = source.match(/-?[0-9]+/g);
  if (!numbers?.length) return null;
  const code = Number(numbers[numbers.length - 1]);
  return Number.isFinite(code) ? code : null;
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
  const mode = String(input.mode?.value ?? "").toLowerCase();
  const modeCode = extractRobotModeCode(mode);

  if (errorPresent(payload) || mode.includes("error") || modeCode === 9) return ROBOT_STATUS.error;
  if (input.connected === false) return ROBOT_STATUS.disconnected;
  if (mode.includes("pause") || modeCode === 10 || (!mode && input.lastCommand === "pause")) return ROBOT_STATUS.paused;

  if (modeCode === 1) return ROBOT_STATUS.initializing;
  if (
    [6, 7, 8, 11].includes(modeCode ?? -1) ||
    mode.includes("running") || mode.includes("moving") || mode.includes("jog") ||
    mode.includes("backdrive") || mode.includes("recording") ||
    input.executionStatus === "EXECUTING" ||
    (input.lastJointChangeAt !== undefined && Date.now() - input.lastJointChangeAt < 650)
  ) return ROBOT_STATUS.moving;

  if ([2, 3, 4].includes(modeCode ?? -1)) return ROBOT_STATUS.disabled;
  if (input.planningStatus === "PLANNING") return ROBOT_STATUS.planning;
  if (input.connected === true && input.enabled === false) return ROBOT_STATUS.disabled;
  if (modeCode === 5 || (input.connected === true && input.enabled === true)) return ROBOT_STATUS.idle;
  return ROBOT_STATUS.unknown;
}

export function formatRobotMode(value?: string): string {
  if (!value) return "Unavailable";
  const trimmed = value.trim();
  const code = extractRobotModeCode(trimmed);
  if (code === null) return trimmed.length > 32 ? trimmed.slice(0, 32) + "…" : trimmed;
  const labels: Record<number, string> = {
    1: "Initializing", 2: "Brake open", 3: "Power off", 4: "Disabled",
    5: "Enabled / idle", 6: "Backdrive", 7: "Running",
    8: "Recording / single move", 9: "Error", 10: "Paused", 11: "Jog"
  };
  return labels[code] ? labels[code] + " (" + code + ")" : trimmed;
}

export function hasActiveError(errors?: Record<string,unknown> | null): boolean {
  return errorPresent(errors?.GetErrorID ?? errors?.robot_return ?? errors);
}
export function statusClass(key: RobotStatusKey) { return "status-" + key; }

export function operationStatus(value?: string | null): { key: RobotStatusKey; label: string } {
  const normalized = String(value ?? "").toUpperCase();
  if (normalized === "PLANNING") return { key: "planning", label: "Planning" };
  if (normalized === "EXECUTING") return { key: "moving", label: "Executing" };
  if (normalized === "SUCCEEDED") return { key: "idle", label: "Ready" };
  if (normalized === "FAILED" || normalized === "TIMED_OUT_OR_FAILED") return { key: "error", label: "Failed" };
  if (normalized === "CANCELED" || normalized === "CANCELLED" || normalized === "ABORTED") return { key: "disabled", label: "Cancelled" };
  return { key: "unknown", label: value || "Not planned" };
}
