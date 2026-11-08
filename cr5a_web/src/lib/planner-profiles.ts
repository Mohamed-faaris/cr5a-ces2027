export type PlanningPipeline = "ompl" | "pilz_industrial_motion_planner" | "chomp";

export interface PlannerProfile {
  id: string;
  name: string;
  description: string;
  pipeline_id: PlanningPipeline;
  planner_id: string;
  allowed_planning_time: number;
  num_planning_attempts: number;
  max_velocity_scaling: number;
  max_acceleration_scaling: number;
  max_planning_threads: number;
  planner_parameters: Record<string, number | string | boolean>;
}

export const DEFAULT_PLANNER_PROFILES: PlannerProfile[] = [
  {
    id: "safe",
    name: "Conservative",
    description: "Single OMPL worker and short planning budget. Use while system memory is constrained.",
    pipeline_id: "ompl",
    planner_id: "RRTConnect",
    allowed_planning_time: 3,
    num_planning_attempts: 2,
    max_velocity_scaling: 0.2,
    max_acceleration_scaling: 0.2,
    max_planning_threads: 1,
    planner_parameters: {},
  },
  {
    id: "balanced",
    name: "Balanced",
    description: "Recommended starting point for the 12-logical-CPU host with current memory headroom.",
    pipeline_id: "ompl",
    planner_id: "RRTConnect",
    allowed_planning_time: 5,
    num_planning_attempts: 5,
    max_velocity_scaling: 0.2,
    max_acceleration_scaling: 0.2,
    max_planning_threads: 2,
    planner_parameters: {},
  },
  {
    id: "quality",
    name: "Path quality",
    description: "RRTstar spends longer improving a path; measure it against RRTConnect.",
    pipeline_id: "ompl",
    planner_id: "RRTstar",
    allowed_planning_time: 8,
    num_planning_attempts: 3,
    max_velocity_scaling: 0.2,
    max_acceleration_scaling: 0.2,
    max_planning_threads: 2,
    planner_parameters: {},
  },
  {
    id: "roadmap",
    name: "Roadmap",
    description: "PRMstar is worth benchmarking for repeated queries in an unchanged scene.",
    pipeline_id: "ompl",
    planner_id: "PRMstar",
    allowed_planning_time: 8,
    num_planning_attempts: 3,
    max_velocity_scaling: 0.2,
    max_acceleration_scaling: 0.2,
    max_planning_threads: 2,
    planner_parameters: {},
  },
  {
    id: "pilz-lin",
    name: "Cartesian LIN",
    description: "Straight TCP approach or retreat; the entire line must be collision-free because LIN does not route around obstacles.",
    pipeline_id: "pilz_industrial_motion_planner",
    planner_id: "LIN",
    allowed_planning_time: 3,
    num_planning_attempts: 1,
    max_velocity_scaling: 0.1,
    max_acceleration_scaling: 0.1,
    max_planning_threads: 1,
    planner_parameters: {},
  },
  {
    id: "pilz-ptp",
    name: "Pilz PTP",
    description: "Synchronized point-to-point motion with trapezoidal joint profiles.",
    pipeline_id: "pilz_industrial_motion_planner",
    planner_id: "PTP",
    allowed_planning_time: 3,
    num_planning_attempts: 1,
    max_velocity_scaling: 0.1,
    max_acceleration_scaling: 0.1,
    max_planning_threads: 1,
    planner_parameters: {},
  },
  {
    id: "chomp",
    name: "CHOMP refine",
    description: "Trajectory optimization; benchmark with clutter and a valid initial trajectory.",
    pipeline_id: "chomp",
    planner_id: "",
    allowed_planning_time: 8,
    num_planning_attempts: 1,
    max_velocity_scaling: 0.2,
    max_acceleration_scaling: 0.2,
    max_planning_threads: 1,
    planner_parameters: {},
  },
  {
    id: "high-throughput",
    name: "High throughput",
    description: "Four OMPL workers. Only use when the host has adequate free memory.",
    pipeline_id: "ompl",
    planner_id: "RRTConnect",
    allowed_planning_time: 6,
    num_planning_attempts: 6,
    max_velocity_scaling: 0.2,
    max_acceleration_scaling: 0.2,
    max_planning_threads: 4,
    planner_parameters: {},
  },
];

export const DEFAULT_PLANNER_PROFILE = DEFAULT_PLANNER_PROFILES[1];

export function plannerRequestOptions(profile: PlannerProfile) {
  return {
    pipeline_id: profile.pipeline_id,
    planner_id: profile.planner_id,
    allowed_planning_time: profile.allowed_planning_time,
    num_planning_attempts: profile.num_planning_attempts,
    max_velocity_scaling: profile.max_velocity_scaling,
    max_acceleration_scaling: profile.max_acceleration_scaling,
    ...(Object.keys(profile.planner_parameters).length > 0
      ? { planner_parameters: profile.planner_parameters }
      : {}),
  };
}
