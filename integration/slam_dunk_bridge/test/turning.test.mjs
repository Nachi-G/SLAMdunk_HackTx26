import assert from 'node:assert/strict';
import {mountDashboard} from '../slam_dunk_bridge/web/app.mjs';
import {createUnknownDemo} from '../slam_dunk_bridge/web/unknown.mjs';
import {MissionStore} from '../slam_dunk_bridge/web/model.mjs';
import {createTestEnvironment,createTestMarkup} from './ui_integration.test.mjs';

export function runTurningTests(){
  for(const environment of ['sandbox','A']){
    const env=createTestEnvironment(4100),doc=createTestMarkup(),get=id=>doc.getElementById(id);let app;
    try{
      app=mountDashboard({document:doc});
      if(environment==='A'){get('environment').value='A';get('environment').fire('change');get('begin-exploration').click();}
      env.advance(100);
      const click=(scene,canvas,p)=>{const q=scene.project([p[0],p[1],0]),event={clientX:q[0],clientY:q[1],pointerId:81,button:0};get(canvas).fire('pointerdown',event);get(canvas).fire('pointerup',event);};
      const targets=environment==='A'?[[-8,-6,1.8],[-8,-5,1.8],[-9,-5,1.8]]:[[-9,-3,1.8],[-8,-3,1.8],[-9,-2.5,1.8]];
      for(const target of targets){
        click(app.scenes.main,'main-canvas',target);
        assert.ok(Math.hypot(...app.adapter.getSimulation().goal.map((v,i)=>v-target[i]))<1e-7,'3D click changes the selected goal');
      }
      click(app.scenes.top,'top-canvas',targets[0]);
      const goal=app.adapter.getSimulation().goal;
      const left=doc.directions.find(b=>b.dataset.direction==='turn-left');assert.ok(left);
      left.fire('pointerdown',{pointerId:44,button:0});
      const before=app.store.getState().pose;env.advance(100);
      assert.deepEqual(app.store.getState().pose.position,before.position,'turning is in place');
      assert.ok(app.store.getState().pose.yaw>before.yaw,'left turn changes heading');
      left.fire('pointerup',{pointerId:44});
      const held=app.store.getState().pose.position;
      assert.equal(app.adapter.getSimulation().handoffRemainingMs,500);
      env.advance(499);assert.deepEqual(app.store.getState().pose.position,held,'autonomy waits the full 500 ms');
      assert.deepEqual(app.adapter.getSimulation().goal,goal,'selected goal survives turning');
      env.advance(1);assert.equal(app.adapter.getSimulation().handoffRemainingMs,0);
      assert.notDeepEqual(app.store.getState().pose.position,held,'autonomy resumes at the deadline');
      doc.fire('keydown',{key:'e',repeat:false});const heading=app.store.getState().pose.yaw;env.advance(100);
      assert.ok(app.store.getState().pose.yaw<heading,'E turns right');
      doc.fire('keyup',{key:'e'});assert.equal(app.adapter.getSimulation().handoffRemainingMs,500);
      env.advance(300);doc.fire('keydown',{key:'q',repeat:false});env.advance(100);doc.fire('keyup',{key:'q'});
      const repeated=app.store.getState().pose.position;env.advance(499);assert.deepEqual(app.store.getState().pose.position,repeated,'another turn restarts the delay');
      env.advance(1);assert.equal(app.adapter.getSimulation().handoffRemainingMs,0);
      get('source').value='live';get('source').fire('change');
      assert.equal(doc.directions.find(b=>b.dataset.direction==='turn-left').disabled,true,'unsupported live yaw stays disabled');
    }finally{app?.destroy();assert.equal(env.intervals.size,0);env.restore();}
  }
  // A missing release expires the turn lease, then waits another 500 ms.
  const store=new MissionStore(),demo=createUnknownDemo(store,'A');let now=1700000000000;
  demo.step(now);demo.start();now+=100;demo.step(now);store.startSteering('turn-left',now);now+=100;demo.step(now);
  const position=demo.snapshot().position;now+=799;demo.step(now);assert.deepEqual(demo.snapshot().position,position);
  assert.equal(demo.snapshot().handoffRemainingMs,101);now+=101;demo.step(now);assert.equal(demo.snapshot().handoffRemainingMs,0);
  return {perspectiveAndTopGoals:true,inPlaceLeftRight:true,releaseDelayMs:500,repeatAndLeaseExpiry:true};
}
