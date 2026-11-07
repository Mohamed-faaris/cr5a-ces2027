declare module "urdf-loader" {
  import type { Object3D } from "three";
  export default class URDFLoader {
    packages:Record<string,string>;
    load(url:string,onComplete:(robot:Object3D)=>void,onProgress?:(event:ProgressEvent)=>void,onError?:(error:unknown)=>void):void;
  }
}
