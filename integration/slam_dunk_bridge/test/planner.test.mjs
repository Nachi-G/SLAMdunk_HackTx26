import assert from 'node:assert/strict';
import {planPath,MISSIONS,segmentFree,cubicSegment,free,moversAt} from '../slam_dunk_bridge/web/planner.mjs';
import {MissionStore} from '../slam_dunk_bridge/web/model.mjs';
import {createDemo} from '../slam_dunk_bridge/web/demo.mjs';

export function runPlannerTests(){
  // Every consecutive patrol destination needs a route through the furnished site.
  for(let i=0;i<MISSIONS.length;i++){
    const route=planPath(MISSIONS[i],MISSIONS[(i+1)%MISSIONS.length],.45);
    assert.ok(route.length>1,`No route for mission ${i}`);
    for(let j=1;j<route.length;j++)assert.ok(segmentFree(route[j-1],route[j],.45));
  }
  assert.deepEqual(planPath(MISSIONS[0],[-3,5,1.8],.45),[],'wall goal rejected');
  assert.equal(free([-8,-5.1],.45),true);
  assert.equal(free([-8,-5.1],.9),false,'increased inflation closes near-furniture space');
  // A valid pair of endpoints may still have a blocked connecting segment.
  assert.equal(segmentFree([-4,5,1.8],[-2,5,1.8],.45),false);
  const actor=moversAt(0)[0];
  assert.equal(segmentFree([actor.position[0]-2,-.8],[actor.position[0]+2,-.8],.45,0,0,true),false);
  // Analytic cubic extrema obey the selected duration bounds.
  for(const d of [.05,.5,2,10])for(const speed of [.4,1,2]){
    const c=cubicSegment([0,0,0],[d,0,0],speed);
    assert.ok(1.5*d/c.duration<=speed+1e-9);
    assert.ok(6*d/c.duration**2<=2+1e-9);
    assert.ok(12*d/c.duration**3<=4+1e-9);
    assert.deepEqual(c.sample(0).velocity,[0,0,0]);
    assert.deepEqual(c.sample(c.duration).position,[d,0,0]);
  }
  const store=new MissionStore();store.reset('demo');const demo=createDemo(store);
  let minimum=Infinity,yields=0;
  // Ten simulated minutes catches patrol deadlocks and collisions beyond one route.
  for(let i=0;i<6000;i++){
    demo.step(1700000000000+i*100);const snapshot=demo.snapshot();
    minimum=Math.min(minimum,snapshot.clearance);if(snapshot.status.startsWith('Yielding'))yields++;
    assert.ok(snapshot.clearance>=.45-1e-6,'vehicle crossed inflated geometry');
    assert.ok(store.getState().pose.position.every(Number.isFinite));
  }
  assert.ok(demo.snapshot().completed>=6,'autonomous course must complete a full loop');
  assert.ok(yields>0,'course must exercise dynamic obstacle prediction');
  assert.equal(demo.setGoal([-3,5,1.8]).ok,false);
  assert.equal(demo.setGoal([NaN,0,1.8]).ok,false);
  const before=demo.snapshot();store.pauseExploration();demo.step(1700000601000);
  assert.deepEqual(demo.snapshot().movers,before.movers,'pause freezes patrols');
  assert.deepEqual(demo.snapshot().path,before.path,'pause freezes vehicle');
  return {missions:6,trajectoryBounds:12,simulationSteps:6000,goalsReached:before.completed,minimumClearance:minimum,yieldFrames:yields};
}
