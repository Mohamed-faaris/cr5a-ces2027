import { clsx, type ClassValue } from "clsx";
import { twMerge } from "tailwind-merge";

export function cn(...inputs: ClassValue[]) { return twMerge(clsx(inputs)); }
export function finite(value: unknown): number | null {
  if (value === null || value === undefined || typeof value === "boolean") return null;
  if (typeof value === "string" && value.trim() === "") return null;
  const parsed = typeof value === "number" ? value : Number(value);
  return Number.isFinite(parsed) ? parsed : null;
}
export function formatNumber(value: unknown, digits = 3): string {
  const parsed = finite(value);
  return parsed === null ? "—" : parsed.toFixed(digits);
}
export function rpyDegreesToQuaternion(rollDeg: number, pitchDeg: number, yawDeg: number) {
  const roll = rollDeg * Math.PI / 180;
  const pitch = pitchDeg * Math.PI / 180;
  const yaw = yawDeg * Math.PI / 180;
  const cr = Math.cos(roll / 2), sr = Math.sin(roll / 2);
  const cp = Math.cos(pitch / 2), sp = Math.sin(pitch / 2);
  const cy = Math.cos(yaw / 2), sy = Math.sin(yaw / 2);
  return [
    sr * cp * cy - cr * sp * sy,
    cr * sp * cy + sr * cp * sy,
    cr * cp * sy - sr * sp * cy,
    cr * cp * cy + sr * sp * sy
  ];
}
export interface JointPositionLimitConfig {
  has_position_limits?: boolean;
  min_position?: unknown;
  max_position?: unknown;
}

export function parseJointBounds(
  urdf?: string,
  configured?: Record<string,JointPositionLimitConfig>
): Record<string,{lower:number;upper:number}> {
  const result: Record<string,{lower:number;upper:number}> = {};
  if (urdf && typeof DOMParser !== "undefined") {
    const doc = new DOMParser().parseFromString(urdf, "application/xml");
    if (doc.querySelector("parsererror") === null) {
      doc.querySelectorAll("robot > joint").forEach((joint) => {
        const name = joint.getAttribute("name");
        const limit = joint.querySelector(":scope > limit");
        const lower = finite(limit?.getAttribute("lower"));
        const upper = finite(limit?.getAttribute("upper"));
        if (name && lower !== null && upper !== null && lower < upper) {
          result[name] = {lower,upper};
        }
      });
    }
  }

  // MoveIt config explicitly overrides URDF values when position limits are enabled.
  Object.entries(configured ?? {}).forEach(([name,limit]) => {
    if (limit.has_position_limits === false) {
      delete result[name];
      return;
    }
    const lower = finite(limit.min_position);
    const upper = finite(limit.max_position);
    if (lower !== null && upper !== null && lower < upper) {
      result[name] = {lower,upper};
    }
  });

  return result;
}
export function safeDate(value?: string): string {
  if (!value) return "—";
  const date = new Date(value);
  return Number.isNaN(date.getTime()) ? "—" : date.toLocaleTimeString();
}
