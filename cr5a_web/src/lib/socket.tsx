import { createContext,useContext,useEffect,type ReactNode } from "react";
import { request,startSocket } from "./socket-client";

const SocketContext=createContext<{request:typeof request}>({request});
export function SocketProvider({children}:{children:ReactNode}) {
  useEffect(()=>startSocket(),[]);
  return <SocketContext.Provider value={{request}}>{children}</SocketContext.Provider>;
}
export function useRobotSocket(){return useContext(SocketContext);}
