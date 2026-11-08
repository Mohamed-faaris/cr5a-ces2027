import { createContext,useContext,useEffect,type ReactNode } from "react";
import { request,startSocket,subscribeChannels } from "./socket-client";

const SocketContext=createContext<{request:typeof request;subscribeChannels:typeof subscribeChannels}>({request,subscribeChannels});
export function SocketProvider({children}:{children:ReactNode}) {
  useEffect(()=>startSocket(),[]);
  return <SocketContext.Provider value={{request,subscribeChannels}}>{children}</SocketContext.Provider>;
}
export function useRobotSocket(){return useContext(SocketContext);}
