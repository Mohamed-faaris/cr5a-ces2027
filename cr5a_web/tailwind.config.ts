import type { Config } from "tailwindcss";
export default {
  darkMode:["class"],
  content:["./index.html","./src/**/*.{ts,tsx}"],
  theme:{extend:{fontFamily:{
    sans:["Inter","ui-sans-serif","system-ui","sans-serif"],
    mono:["ui-monospace","SFMono-Regular","Menlo","monospace"]
  }}},
  plugins:[]
} satisfies Config;
