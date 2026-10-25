import assert from 'node:assert/strict';
import {MissionStore} from '../slam_dunk_bridge/web/model.mjs';
import {createUnknownDemo,UNKNOWN_ENVIRONMENTS} from '../slam_dunk_bridge/web/unknown.mjs';
import {mountDashboard} from '../slam_dunk_bridge/web/app.mjs';
import {createTestEnvironment,createTestMarkup} from './ui_integration.test.mjs';

export function runUnknownTests(){
  const results=[];
  assert.deepEqual(UNKNOWN_ENVIRONMENTS.map(w=>w.id),['A','B','C']);
  for(const id of ['A','B','C']){
    const store=new MissionStore(),demo=createUnknownDemo(store,id);let now=1700000000000;
    demo.step(now);assert.equal(demo.snapshot().observedCells,0);assert.deepEqual(store.getState().mapPoints,[]);
    assert.deepEqual(demo.snapshot().movers,[],'unseen actors are not disclosed');
    assert.equal(demo.setGoal([-9,6,1.8]).ok,false);
    demo.start();now+=250;demo.step(now);const first=demo.snapshot(),initial=first.observedCells;
    assert.ok(initial>0&&initial<first.totalCells*.4,'first scan reveals a bounded neighborhood');
    assert.ok(store.getState().mapPoints.every(p=>Math.hypot(p[0]-first.position[0],p[1]-first.position[1])<=first.sensorRange+.75),'no remote geometry is published');
    assert.equal(demo.setGoal([9,7,1.8]).ok,false,'unknown destination is rejected without consulting truth');
    let previous=initial;
    for(let i=0;i<600;i++){
      now+=250;demo.step(now);
      if(i%20)continue;
      const s=demo.snapshot();assert.ok(s.observedCells>=previous);previous=s.observedCells;
      assert.deepEqual(s.obstacles,[]);assert.deepEqual(s.fixtures,[]);assert.deepEqual(s.placed,[]);
      assert.equal('walls' in s,false);assert.equal('truth' in s,false);
      const free=new Set(s.freeCells.map(p=>`${Math.floor((p[0]+12)/.5)},${Math.floor((p[1]+9)/.5)}`));
      for(const p of s.path)assert.ok(free.has(`${Math.floor((p[0]+12)/.5)},${Math.floor((p[1]+9)/.5)}`),'planned path stays on sensed free cells');
      assert.ok(s.movers.every(o=>Math.hypot(o.position[0]-s.position[0],o.position[1]-s.position[1])<s.sensorRange+.6));
    }
    const s=demo.snapshot();assert.ok(s.observedCells>initial*1.8,`${id} expands mapping while exploring`);assert.ok(s.completed>=3);
    store.pauseExploration();const paused=demo.snapshot();now+=1000;demo.step(now);
    assert.deepEqual(demo.snapshot().position,paused.position);assert.equal(demo.snapshot().scanSequence,paused.scanSequence);
    results.push({id,initialCells:initial,observedCells:s.observedCells,viewpoints:s.completed,coverage:s.coverage});
  }
  assert.equal(new Set(results.map(s=>s.observedCells)).size,3,'environments have distinct exploration outcomes');
  const env=createTestEnvironment(3000),doc=createTestMarkup(),get=id=>doc.getElementById(id);let app;
  try{
    app=mountDashboard({document:doc});get('environment').value='A';get('environment').fire('change');
    assert.equal(app.adapter.getSimulation().environment,'A');assert.equal(app.store.getState().mapPoints.length,0);
    assert.equal(get('unknown-controls').hidden,false);assert.equal(get('planner-panel').hidden,true);
    assert.equal(get('view-pov').hidden,true);get('view-pov').click();assert.equal(get('pov-canvas').hidden,true);
    assert.equal(app.adapter.editSimulation('add',[0,0],'crate').ok,false);
    get('begin-exploration').click();env.advance(400);
    assert.ok(app.adapter.getSimulation().observedCells>0);assert.ok(get('survey-area').textContent!=='0.0 m²');
    assert.equal(get('begin-exploration').disabled,true);
    get('stop').click();const p=app.adapter.getSimulation().position;env.advance(1000);assert.deepEqual(app.adapter.getSimulation().position,p);
    get('reset-survey').click();assert.equal(app.adapter.getSimulation().observedCells,0);assert.deepEqual(app.store.getState().combinedPoints,[]);
    get('environment').value='C';get('environment').fire('change');assert.equal(app.adapter.getSimulation().environment,'C');assert.equal(app.adapter.getSimulation().started,false);
    get('environment').value='sandbox';get('environment').fire('change');assert.equal(app.adapter.getSimulation().hidden,undefined);assert.equal(get('planner-panel').hidden,false);
    get('source').value='live';get('source').fire('change');assert.equal(get('environment-panel').hidden,true);assert.equal(app.adapter.startExploration().ok,false);
  }finally{app?.destroy();assert.equal(env.intervals.size,0);env.restore();}
  return {environments:results,dashboardSwitchResetAndLiveIsolation:true};
}
