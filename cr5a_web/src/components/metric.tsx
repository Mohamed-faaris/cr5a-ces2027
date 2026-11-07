export function Metric({label,value,suffix,hint}:{label:string;value:string;suffix?:string;hint?:string}) {
  return <div className="min-w-0">
    <div className="text-[11px] font-medium uppercase tracking-[0.12em] text-zinc-500">{label}</div>
    <div className="mt-1 flex items-baseline gap-1.5">
      <span className="truncate font-mono text-lg font-medium tabular-nums text-zinc-950">{value}</span>
      {suffix&&<span className="text-xs text-zinc-400">{suffix}</span>}
    </div>
    {hint&&<div className="mt-1 text-[11px] text-zinc-400">{hint}</div>}
  </div>;
}
