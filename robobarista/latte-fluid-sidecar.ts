/** Replay RoboBarista 3D camera AprilTag tip path through actual MIT 2D fluid core.
 *
 * Upstream must be checked out at vendor/latte-fluid-sim:
 *   https://github.com/p-sun/typescript-fluid-simulator
 *   MIT Copyright (c) 2023 Paige Sun
 * Pure upstream code is used via imports, NOT reproduced or patched.
 *
 * The output is illustrative WHITE MILK surface pigment, not physical
 * hydrodynamics, measured flow or a validated autonomous pour.
 */
import fs from "node:fs";
import path from "node:path";
import { makeScene, setObstacle } from "../vendor/latte-fluid-sim/src/FluidScene.ts";
import Vec2 from "../vendor/latte-fluid-sim/src/Utils/Vec2.ts";

const input=process.argv[2];
const output=process.argv[3];
if (!input || !output) throw new Error("usage: tsx latte-fluid-sidecar.ts trace.json output/");
const trace=JSON.parse(fs.readFileSync(input,"utf8"));
if (trace.schema!=="robobarista-fluid2d-input-v1" || trace.events.length<3)
  throw new Error("invalid camera-derived trajectory");
fs.mkdirSync(output,{recursive:true});
const scene=makeScene("Latte Scene",new Vec2(360,360),{
  resolution:64,
  milkStartSpeed:0.8,
  timeToZeroMilkSpeed:45.0,
  timeToMinObstacleRadius:8.0,
});
let simulated_s=0;
let hit=0;
let index=0;
const events=trace.events;
for (const e of events) {
  const target=e.t_s-events[0].t_s;
  if(!Number.isFinite(target) || target<simulated_s-.0001)
    throw new Error("bad trajectory timestamps");
  while (simulated_s+scene.dt*0.5<target) {
    scene.fluid.simulate(scene,scene.dt);
    scene.frameNr+=1;
    simulated_s+=scene.dt;
  }
  const x=e.norm_x,y=e.norm_y;
  if (![x,y].every((v:number)=>Number.isFinite(v)&&v>=0&&v<=1))
     throw new Error("bad 2D projected spout");
  // Public upstream physics method; pressed milk tool.
  setObstacle(scene,x,y,index===0,true);
  scene.fluid.simulate(scene,scene.dt);
  scene.frameNr+=1;
  simulated_s+=scene.dt;
  const f=scene.fluid;
  const n=f.numY;
  const pgm=Buffer.alloc(f.numX*f.numY);
  let whitened=0;
  for(let j=0;j<f.numY;j++){
    for(let i=0;i<f.numX;i++){
      const m=Math.min(1,Math.max(0,Number(f.m[i*n+j])));
      if (m>0.25)whitened++;
      const intensity=Math.round(32+218*m);
      pgm[(f.numY-1-j)*f.numX+i]=intensity;
    }
  }
  const header=Buffer.from(`P5\n${f.numX} ${f.numY}\n255\n`);
  const file=path.join(output,`milk_${String(index).padStart(4,"0")}.pgm`);
  fs.writeFileSync(file,Buffer.concat([header,pgm]));
  hit+=whitened;
  index++;
}
setObstacle(scene,0,0,true);
const summary={
  engine:"p-sun/typescript-fluid-simulator imported actual MIT source",
  linked_to_real_camera_apriltag_positions:true,
  frames:events.length,
  virtual_video_duration_s:events.at(-1).t_s-events[0].t_s,
  normalized_spout_path:true,
  separate_2d_surface_only:true,
  rendered_milk_proxy_png:false,
  real_milk_flow_simulated:false,
  valid_3d_steaming_physics:false,
  true_robot_controller_feedback:false,
  white_proxy_cell_hits:hit
};
fs.writeFileSync(path.join(output,"summary.json"),JSON.stringify(summary,null,2));
console.log(JSON.stringify(summary,null,2));
