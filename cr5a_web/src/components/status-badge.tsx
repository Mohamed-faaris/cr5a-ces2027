import {Circle} from "lucide-react";
import {ROBOT_STATUS,statusClass,type RobotStatusKey} from "@/lib/status";
import {cn} from "@/lib/utils";

export function StatusBadge({state,compact=false}:{state:RobotStatusKey;compact?:boolean}) {
  const status=ROBOT_STATUS[state];
  return (
    <span className={cn("status-badge",statusClass(state),compact&&"status-badge-compact")} title={status.hint}>
      <Circle size={8} strokeWidth={0} fill="currentColor"/>
      <span>{status.label}</span>
    </span>
  );
}
