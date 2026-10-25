import assert from 'node:assert/strict';
import {MissionStore} from '../slam_dunk_bridge/web/model.mjs';
import {createDemo} from '../slam_dunk_bridge/web/demo.mjs';
import {sweptDistance} from '../slam_dunk_bridge/web/planner.mjs';

export function runCollisionTests(){
  // Safe endpoints alone cannot rule out a collision between frames.
  assert.equal(sweptDistance([-1,0],[1,0],[1,0],[-1,0]),0);
  assert.equal(sweptDistance([-1,0],[1,0],[0,-1],[0,1]),0);
  assert.equal(sweptDistance([0,0],[0,0],[2,0],[1,0]),1);
  let checks=0,minimum=Infinity;
  function check(before,after){
    assert.ok(after.clearance>=.45-1e-6,`body clearance ${after.clearance}`);
    assert.equal(after.bodyRadius,.62,'collision footprint includes rotor span');
    const previous=new Map(before.movers.map(o=>[o.id,o]));
    for(const object of after.movers){const old=previous.get(object.id);if(!old)continue;
      const gap=sweptDistance(before.position,after.position,old.position,object.position)-object.radius-after.bodyRadius;
      minimum=Math.min(minimum,gap);assert.ok(gap>=.45-1e-6,`swept collision with ${object.id}: ${gap}`);checks++;
    }
  }
  const make=()=>{const store=new MissionStore();store.reset('demo');store.setSpeedMode('standard');const demo=createDemo(store,26,{room:true});demo.step(1700000000000);return {store,demo};};
  // An object drives directly toward the stationary drone with no destination.
  {const {demo}=make();assert.equal(demo.addObstacle([-10,-5],'moving',{axis:'x'}).ok,true);demo.start();
    for(let i=1;i<=600;i++){const before=demo.snapshot();demo.step(1700000000000+i*100);check(before,demo.snapshot());}
    assert.deepEqual(demo.snapshot().position,[-8,-5,1.8]);
  }
  // Crossing, head-on, axis changes, manual pushing and irregular frame times.
  for(let seed=1;seed<=32;seed++){
    const {store,demo}=make();let random=seed,now=1700000000000;
    const rand=()=>{random=(Math.imul(random,1664525)+1013904223)>>>0;return random/4294967296;};
    assert.equal(demo.setGoal([8,5,1.8]).ok,true);
    for(let i=0;i<12;i++)demo.addObstacle([-10+rand()*20,-6+rand()*12],i%4===0?'crate':'moving',{axis:i%2?'x':'y'});
    demo.start();
    for(let i=0;i<600;i++){
      if(i%83===0){const actor=demo.snapshot().movers[i%demo.snapshot().movers.length];if(actor)demo.setObjectMotion({kind:'moving',id:actor.id},actor.axis==='x'?'y':'x');}
      if(i%60===0){demo.clearManual();store.startSteering(['forward','left','backward','right'][Math.floor(i/60)%4],now);}
      if(i%60===20){store.releaseSteering();demo.clearManual();}
      if(store.getState().steering)store.renewSteering(now);
      if(store.getState().explorationMode==='paused'){demo.setGoal([i%2?-8:8,i%3?5:-5,1.8]);store.resumeExploration();}
      const before=demo.snapshot();now+=[16,40,100,250,900][i%5];demo.step(now);check(before,demo.snapshot());
    }
  }
  return {scenarios:33,sweptChecks:checks,minimumRotorClearance:minimum};
}
