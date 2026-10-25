import assert from 'node:assert/strict';
import {MissionStore} from '../slam_dunk_bridge/web/model.mjs';
import {createDemo} from '../slam_dunk_bridge/web/demo.mjs';
import {STATIC,segmentFree} from '../slam_dunk_bridge/web/planner.mjs';
import {rayBox,createPov} from '../slam_dunk_bridge/web/pov.mjs';
export function runArenaTests(){
  const store=new MissionStore();store.reset('demo');const demo=createDemo(store);let now=1700000000000;demo.step(now);
  const initial=demo.snapshot();
  assert.equal(demo.addObstacle(initial.position).ok,false,'cannot spawn on drone');
  assert.equal(demo.addObstacle(initial.goal).ok,false,'cannot cover goal');
  assert.equal(demo.addObstacle([-3,5]).ok,false,'cannot overlap wall');
  assert.equal(demo.addObstacle([99,0]).ok,false);
  assert.equal(demo.addObstacle([NaN,0]).ok,false);
  assert.equal(demo.addObstacle([-8.5,-2],'crate').ok,true);
  assert.equal(demo.snapshot().placed.length,1);
  assert.equal(demo.setGoal([-8.5,-2,1.8]).ok,false,'added obstacle affects goal validation');
  assert.equal(segmentFree(initial.position,initial.goal,.45,0,0,false,.1,()=>[],demo.snapshot().obstacles),false,'crate blocks original route');
  const plans=demo.snapshot().plans;let min=Infinity;
  for(let i=0;i<1800;i++){now+=100;demo.step(now);const frame=demo.snapshot();min=Math.min(min,frame.clearance);assert.ok(frame.clearance>=.45-1e-6);}
  assert.ok(demo.snapshot().plans>plans);assert.ok(demo.snapshot().completed>0,'drone makes progress around judge obstacle');
  // Each simulation owns its own obstacle collection.
  const otherStore=new MissionStore();otherStore.reset('demo');const other=createDemo(otherStore);assert.equal(other.snapshot().obstacles.length,STATIC.length);
  assert.equal(demo.removeObstacle([-8.5,-2]).ok,true);assert.equal(demo.snapshot().placed.length,0);
  assert.equal(demo.removeObstacle([-8.5,-2]).ok,false);
  demo.clearObstacles();assert.equal(demo.snapshot().obstacles.length,STATIC.length);
  // Ray geometry: parallel, behind, front, and inside cases.
  assert.equal(rayBox([0,0],[1,0],[2,-1,3,1]),2);
  assert.equal(rayBox([0,0],[1,0],[-3,-1,-2,1]),null);
  assert.equal(rayBox([0,0],[0,1],[2,-1,3,1]),null);
  assert.equal(rayBox([2.5,0],[1,0],[2,-1,3,1]),.5);
  let paints=0;const ctx=new Proxy({}, {get(target,key){return target[key]??((...args)=>{paints++;for(const a of args)if(typeof a==='number')assert.ok(Number.isFinite(a));});},set(target,key,value){target[key]=value;return true;}});
  const canvas={width:0,height:0,getContext:()=>ctx,getBoundingClientRect:()=>({width:640,height:360})},pov=createPov(canvas);
  pov.setData(initial);assert.ok(paints>0);const before=paints;pov.setData(initial);assert.equal(paints,before,'unchanged camera frame skips redraw');
  pov.setData({...initial,yaw:initial.yaw+Math.PI/2});assert.ok(paints>before);pov.destroy();
  return {simulationSteps:1800,goalsReached:demo.snapshot().completed,minimumClearance:min,rayCases:4};
}
