import {Hand} from "lucide-react";
import {Link} from "react-router-dom";
import {GripperDiagnosticsCard,GripperPositionCard} from "@/components/gripper/gripper-panels";
import {useGripperController} from "@/components/gripper/use-gripper-controller";

export function GripperPage(){
  const gripper=useGripperController();
  return <div className="space-y-5">
    <div className="flex flex-wrap items-end justify-between gap-3">
      <div>
        <div className="eyebrow flex items-center gap-1.5"><Hand size={12}/> END EFFECTOR / PGEA</div>
        <h1 className="page-title">Full gripper panel</h1>
        <p className="mt-1 text-sm text-zinc-500">
          Direct-link view combining PGEA diagnostics, initialization and finger position controls.
        </p>
      </div>
      <div className="flex gap-3 text-xs">
        <Link to="/diagnostics" className="font-medium text-zinc-600 underline underline-offset-2">Diagnostics</Link>
        <Link to="/visualizer" className="font-medium text-zinc-600 underline underline-offset-2">Controls</Link>
      </div>
    </div>
    <div className="grid items-start gap-5 xl:grid-cols-[minmax(0,1.05fr)_minmax(360px,0.95fr)]">
      <GripperDiagnosticsCard controller={gripper}/>
      <GripperPositionCard controller={gripper}/>
    </div>
    <div className="text-[11px] text-zinc-400">
      Software cancellation is not a safety-rated protective stop. Keep the physical E-stop accessible and verify speed and force during commissioning.
    </div>
  </div>;
}
