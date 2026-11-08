import type {PlanningPipeline} from "@/lib/planner-profiles";

export function describeMoveItError(
  code:number,
  context?:{pipeline?:PlanningPipeline;targetType?:"pose"|"joints"}
):string{
  if(code===-31){
    const pilzNote=context?.pipeline==="pilz_industrial_motion_planner"
      ?" The CR5A KDL solver timeout is now configured for 50 ms; restart MoveGroup and verify the loaded parameter before further tuning."
      :"";
    return "NO_IK_SOLUTION: MoveIt could not find joint angles for the requested pose. Check target reachability, orientation, end-effector link/frame, joint limits, and IK solver settings. A nearby pose can help distinguish an unreachable target from an IK timeout."+pilzNote;
  }
  if(code===-16&&context?.pipeline==="chomp"&&context.targetType==="pose"){
    return "INVALID_GOAL_CONSTRAINTS: CHOMP supports joint-space goals only; it rejects TCP position/orientation constraints. Switch to the Joints target or remove CHOMP from this TCP comparison.";
  }
  const reasons:Record<number,string>={
    [-1]:"PLANNING_FAILED: The planner did not find a valid trajectory. Check reachability, obstacles, constraints, and planning time.",
    [-2]:"INVALID_MOTION_PLAN: MoveIt returned a trajectory that failed validation.",
    [-3]:"MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE: The planning scene or robot state changed during planning.",
    [-4]:"CONTROL_FAILED: The controller failed while executing a trajectory. This should not occur during plan-only benchmarking.",
    [-6]:"TIMED_OUT: The planning request exceeded its time limit.",
    [-7]:"PREEMPTED: The planning request was cancelled or preempted.",
    [-10]:"START_STATE_IN_COLLISION: The current robot state is in collision. Check the start pose and collision scene.",
    [-11]:"START_STATE_VIOLATES_PATH_CONSTRAINTS: The current state violates the requested path constraints.",
    [-12]:"GOAL_IN_COLLISION: The requested goal state is in collision. Move the target or update the collision scene only after verifying the real workspace.",
    [-13]:"GOAL_VIOLATES_PATH_CONSTRAINTS: The goal violates the requested path constraints.",
    [-14]:"GOAL_CONSTRAINTS_VIOLATED: The planner could not satisfy the requested goal tolerances.",
    [-15]:"INVALID_GROUP_NAME: The MoveIt planning group does not exist. Check the SRDF group name.",
    [-16]:"INVALID_GOAL_CONSTRAINTS: The request's goal constraints are malformed or unsupported by this pipeline.",
    [-17]:"INVALID_ROBOT_STATE: The requested robot state is invalid or outside configured joint limits.",
    [-18]:"INVALID_LINK_NAME: The end-effector or constrained link is not in the robot model.",
    [-21]:"FRAME_TRANSFORM_FAILURE: MoveIt could not transform the target into the planning frame. Check TF and frame IDs.",
    [-22]:"COLLISION_CHECKING_UNAVAILABLE: MoveIt could not perform collision checking. Check the planning scene monitor.",
    [-23]:"ROBOT_STATE_STALE: The robot state used for planning is stale. Refresh joint-state telemetry.",
    [-25]:"COMMUNICATION_FAILURE: MoveIt communication failed. Check move_group and ROS services/actions.",
    [-26]:"START_STATE_INVALID: The supplied start state is invalid. Refresh joints and verify joint limits.",
    [-27]:"GOAL_STATE_INVALID: The requested goal state is invalid or violates robot-model bounds.",
    [-28]:"UNRECOGNIZED_GOAL_TYPE: The selected planner does not understand this goal type.",
    [-29]:"CRASH: The planning pipeline reported an internal crash. Inspect move_group logs.",
    [-30]:"ABORT: The planning pipeline aborted the request. Inspect move_group logs.",
    [-31]:"NO_IK_SOLUTION: No inverse-kinematics solution was found for the requested pose. Check reachability, orientation, link/frame, joint limits, and IK solver settings.",
    [99999]:"FAILURE: MoveIt reported a generic failure without a more specific error code. Inspect move_group logs."
  };
  return reasons[code]??`MoveIt returned unclassified error code ${code}. Inspect move_group logs for the pipeline-specific reason.`;
}
