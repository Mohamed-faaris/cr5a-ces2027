import {forwardRef,type ButtonHTMLAttributes} from "react";
import {cva,type VariantProps} from "class-variance-authority";
import {cn} from "@/lib/utils";

const variants=cva(
  "inline-flex items-center justify-center gap-2 whitespace-nowrap rounded-md text-sm font-medium transition-colors focus-visible:outline-none focus-visible:ring-2 focus-visible:ring-zinc-950 focus-visible:ring-offset-2 disabled:pointer-events-none disabled:opacity-40",
  {variants:{
    variant:{
      default:"bg-zinc-950 text-white hover:bg-zinc-800",
      secondary:"bg-zinc-100 text-zinc-900 hover:bg-zinc-200",
      outline:"border border-zinc-300 bg-white text-zinc-900 hover:bg-zinc-50",
      destructive:"bg-red-600 text-white hover:bg-red-700",
      ghost:"text-zinc-700 hover:bg-zinc-100"
    },
    size:{default:"h-9 px-3",sm:"h-8 px-2.5 text-xs",lg:"h-10 px-4",icon:"h-9 w-9"}
  },defaultVariants:{variant:"default",size:"default"}}
);
export interface ButtonProps extends ButtonHTMLAttributes<HTMLButtonElement>,VariantProps<typeof variants>{}
export const Button=forwardRef<HTMLButtonElement,ButtonProps>(({className,variant,size,...props},ref)=>(
  <button ref={ref} className={cn(variants({variant,size}),className)} {...props}/>
));
Button.displayName="Button";
