import * as TabsPrimitive from "@radix-ui/react-tabs";
import {cn} from "@/lib/utils";

export const Tabs=TabsPrimitive.Root;
export const TabsList=({className,...props}:React.ComponentProps<typeof TabsPrimitive.List>)=>(
  <TabsPrimitive.List className={cn("inline-flex h-9 items-center gap-1 rounded-lg bg-zinc-100 p-1",className)} {...props}/>
);
export const TabsTrigger=({className,...props}:React.ComponentProps<typeof TabsPrimitive.Trigger>)=>(
  <TabsPrimitive.Trigger className={cn("inline-flex items-center justify-center rounded-md px-3 py-1.5 text-sm font-medium text-zinc-600 transition data-[state=active]:bg-white data-[state=active]:text-zinc-950 data-[state=active]:shadow-sm",className)} {...props}/>
);
export const TabsContent=({className,...props}:React.ComponentProps<typeof TabsPrimitive.Content>)=>(
  <TabsPrimitive.Content className={cn("mt-4 outline-none",className)} {...props}/>
);
