import {NavLink} from "react-router-dom";
import {Activity,Bot,Box} from "lucide-react";
import type {ReactNode} from "react";
import {StatusBadge} from "./status-badge";
import {deriveRobotStatus} from "@/lib/status";
import {useRobotStore} from "@/store/robot-store";
import {cn} from "@/lib/utils";

export function AppShell({children}:{children:ReactNode}) {
  const bridgeState=useRobotStore((state)=>state.bridgeState);
  const robotStatus=useRobotStore((state)=>state.robotStatus);
  const robotMode=useRobotStore((state)=>state.robotMode);
  const errors=useRobotStore((state)=>state.errors);
  const lastCommand=useRobotStore((state)=>state.lastCommand);
  const plan=useRobotStore((state)=>state.plan);
  const executionStatus=useRobotStore((state)=>state.executionStatus);
  const lastJointChangeAt=useRobotStore((state)=>state.lastJointChangeAt);
  const status=deriveRobotStatus({
    bridgeState,connected:robotStatus?.is_connected,enabled:robotStatus?.is_enable,
    mode:robotMode,errors,lastCommand,planningStatus:plan?.status,
    executionStatus,lastJointChangeAt
  });
  const navItems=[
    {to:"/diagnostics",label:"Diagnostics",icon:Activity},
    {to:"/visualizer",label:"Visualizer",icon:Box}
  ];
  return (
    <div className="min-h-screen bg-zinc-50 text-zinc-950">
      <header className="sticky top-0 z-30 border-b border-zinc-200 bg-white/95 backdrop-blur">
        <div className="mx-auto flex h-16 max-w-[1500px] items-center justify-between px-4 sm:px-6 lg:px-8">
          <div className="flex min-w-0 items-center gap-3">
            <div className="flex h-9 w-9 items-center justify-center rounded-lg bg-zinc-950 text-white">
              <Bot size={19} strokeWidth={1.8}/>
            </div>
            <div className="leading-tight">
              <div className="text-sm font-semibold tracking-tight">CR5A Control</div>
              <div className="text-[11px] text-zinc-500">ROBOT OPERATIONS</div>
            </div>
          </div>
          <nav className="hidden items-center gap-1 sm:flex">
            {navItems.map((item)=>{
              const Icon=item.icon;
              return <NavLink key={item.to} to={item.to} className={({isActive})=>cn(
                "flex h-9 items-center gap-2 rounded-md px-3 text-sm font-medium transition",
                isActive?"bg-zinc-100 text-zinc-950":"text-zinc-500 hover:bg-zinc-50 hover:text-zinc-900"
              )}><Icon size={15}/>{item.label}</NavLink>;
            })}
          </nav>
          <div className="flex items-center gap-3">
            <span className="hidden text-[10px] font-medium tracking-[0.12em] text-zinc-400 md:inline">
              {bridgeState==="connected"?"LIVE TELEMETRY":"BRIDGE"}
            </span>
            <StatusBadge state={status.key}/>
          </div>
        </div>
        <nav className="flex border-t border-zinc-100 px-4 py-2 sm:hidden">
          {navItems.map((item)=>{
            const Icon=item.icon;
            return <NavLink key={item.to} to={item.to} className={({isActive})=>cn(
              "mr-2 flex items-center gap-2 rounded-md px-3 py-2 text-xs font-medium",
              isActive?"bg-zinc-100 text-zinc-950":"text-zinc-500"
            )}><Icon size={14}/>{item.label}</NavLink>;
          })}
        </nav>
      </header>
      <main className="mx-auto max-w-[1500px] px-4 py-6 sm:px-6 lg:px-8">{children}</main>
      <footer className="mx-auto max-w-[1500px] px-4 pb-5 text-[11px] text-zinc-400 sm:px-6 lg:px-8">
        CR5A · ROS 2 · MoveIt 2 · Application WebSocket
      </footer>
    </div>
  );
}
