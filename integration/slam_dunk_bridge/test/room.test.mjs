import assert from 'node:assert/strict';
import {createTestMarkup,createTestEnvironment} from './ui_integration.test.mjs';
import {mountRoom} from '../slam_dunk_bridge/web/room.mjs';
import {MissionStore} from '../slam_dunk_bridge/web/model.mjs';
import {createDemo} from '../slam_dunk_bridge/web/demo.mjs';

export function runRoomTests(){
  const env=createTestEnvironment(900),doc=createTestMarkup('index.html'),get=id=>doc.getElementById(id);let app;
  try{
    // The legacy mock supplies a dashboard source selector; room has none.
    app=mountRoom({document:doc});env.advance(100);
    assert.equal(app.adapter.getSimulation().room,true);
    const start=app.adapter.getSimulation().position;env.advance(1000);
    assert.deepEqual(app.adapter.getSimulation().position,start,'startup waits for a destination');
    assert.equal(app.adapter.getSimulation().goal,null);assert.equal(app.adapter.getSimulation().plans,0);
    assert.equal(get('push-forward').disabled,true);assert.equal(get('pause-room').disabled,true);
    doc.fire('keydown',{key:'w'});doc.fire('keyup',{key:'w'});doc.fire('keydown',{key:' '});
    env.advance(300);assert.deepEqual(app.adapter.getSimulation().position,start,'keyboard cannot bypass Start');


    assert.equal(app.adapter.getSimulation().placed.length,0);assert.equal(app.adapter.getSimulation().movers.length,0);
    assert.deepEqual(app.adapter.getSimulation().bounds,[-16,-12,16,12]);
    assert.equal(app.adapter.getSimulation().fixtures.length,10);assert.equal(app.adapter.getSimulation().obstacles.length,14,'furniture and partitions are collision geometry');
    const clickWorld=(p)=>{const screen=app.scene.project(p);assert.ok(screen);const event={clientX:screen[0],clientY:screen[1],pointerId:1,button:0};get('room-canvas').fire('pointerdown',event);get('room-canvas').fire('pointerup',event);};
    get('room-static').click();clickWorld([0,0,0]);assert.equal(app.adapter.getSimulation().placed.length,1);
    const rect=app.adapter.getSimulation().placed[0];assert.ok(Math.abs((rect[0]+rect[2])/2)<1e-8,'perspective screen picking recovers world X');assert.ok(Math.abs((rect[1]+rect[3])/2)<1e-8);
    get('room-moving').click();assert.equal(get('movement-settings').hidden,false);get('movement-axis').value='y';get('movement-axis').fire('change');clickWorld([2,-3,0]);assert.equal(app.adapter.getSimulation().movers.length,1);
    get('room-goal').click();clickWorld([6,2,0]);
    const prepared=app.adapter.getSimulation();env.advance(1000);
    assert.equal(prepared.started,false);assert.deepEqual(app.adapter.getSimulation().position,prepared.position,'choosing a goal does not start flight');
    assert.deepEqual(app.adapter.getSimulation().movers,prepared.movers,'objects wait for Start too');
    assert.equal(app.adapter.startSteering('forward').ok,false,'adapter also gates pushes');
    get('start-room').click();assert.equal(app.adapter.getSimulation().started,true);assert.equal(get('start-room').hidden,true);assert.equal(get('push-forward').disabled,false);
    get('room-moving').click();
    const before=app.adapter.getSimulation().movers[0].position;env.advance(500);const after=app.adapter.getSimulation().movers[0].position;
    assert.equal(after[0],before[0]);assert.notEqual(after[1],before[1],'Y orientation moves up–down across floor');assert.equal(after[2],before[2]);
    get('movement-axis').value='x';get('movement-axis').fire('change');clickWorld([-5,3,0]);
    const xBefore=app.adapter.getSimulation().movers[1].position;env.advance(200);const movers=app.adapter.getSimulation().movers;
    assert.equal(movers[0].axis,'y','changing selector does not change existing object');assert.equal(movers[1].axis,'x');assert.notEqual(movers[1].position[0],xBefore[0]);assert.equal(movers[1].position[1],xBefore[1]);
    const prediction=app.adapter.getSimulation().predictions;assert.ok(prediction[0].every(p=>p[0]===movers[0].position[0]));assert.ok(prediction[1].every(p=>p[1]===movers[1].position[1]));
    assert.equal(app.adapter.editSimulation('remove',movers[1].position).ok,true);

    get('pause-room').click();const paused=app.adapter.getSimulation();env.advance(300);assert.deepEqual(app.adapter.getSimulation().position,paused.position);assert.deepEqual(app.adapter.getSimulation().movers,paused.movers);
    get('room-goal').click();assert.equal(get('movement-settings').hidden,true);get('goal-height').value='2.1';get('goal-height').fire('input');clickWorld([6,2,0]);
    assert.ok(Math.hypot(...app.adapter.getSimulation().goal.map((v,i)=>v-[6,2,2.1][i]))<1e-8);assert.equal(app.store.getState().explorationMode,'autonomous','setting a destination resumes paused flight');
    const goal=app.adapter.getSimulation().goal;
    get('push-left').fire('pointerdown',{pointerId:5,button:0});assert.equal(app.store.getState().explorationMode,'steering');env.advance(150);
    get('push-left').fire('pointerup',{pointerId:5});assert.equal(app.store.getState().explorationMode,'autonomous');assert.deepEqual(app.adapter.getSimulation().goal,goal,'release returns to selected destination');
    doc.fire('keydown',{key:'w'});assert.equal(app.store.getState().explorationMode,'steering');doc.defaultView.fire('blur');assert.equal(app.store.getState().explorationMode,'autonomous');
    get('room-remove').click();clickWorld([0,0,1.1]);assert.equal(app.adapter.getSimulation().placed.length,0,'visible box can be removed without clicking through to floor');
    const moving=app.adapter.getSimulation().movers[0];clickWorld(moving.position);assert.equal(app.adapter.getSimulation().movers.length,0);
    get('room-heat').checked=false;get('room-heat').fire('change');get('room-trail').checked=false;get('room-trail').fire('change');env.flush();
    get('reset-room').click();assert.equal(app.adapter.getSimulation().objectCount,0);assert.equal(get('room-object-count').textContent,'0 / 24');
    const reset=app.adapter.getSimulation().position;env.advance(1000);assert.deepEqual(app.adapter.getSimulation().position,reset,'reset also waits');assert.equal(app.adapter.getSimulation().goal,null);assert.equal(app.adapter.getSimulation().started,false);assert.equal(get('start-room').hidden,false);
    get('room-static').click();clickWorld([0,0,0]);clickWorld([0,0,1.1]);
    assert.equal(get('selected-object').hidden,false,'clicking a visible static object selects it');
    get('selected-motion-y').click();assert.equal(app.adapter.getSimulation().placed.length,0);assert.equal(app.adapter.getSimulation().movers[0].axis,'y','selected static object becomes a Y-moving object');
    assert.equal(app.adapter.getSimulation().started,false);const waiting=app.adapter.getSimulation().movers;env.advance(500);assert.deepEqual(app.adapter.getSimulation().movers,waiting,'editing selected movement cannot bypass Start');
    get('start-room').click();get('push-forward').fire('pointerdown',{pointerId:7,button:0});env.advance(200);get('push-forward').fire('pointerup',{pointerId:7});
    const released=app.adapter.getSimulation().position;env.advance(300);assert.deepEqual(app.adapter.getSimulation().position,released,'pushing without a goal holds on release after Start');
    const selected=app.adapter.getSimulation().movers[0];clickWorld(selected.position);assert.equal(get('selected-motion-y').getAttribute('aria-pressed'),'true');
    get('selected-motion-x').click();const switched=app.adapter.getSimulation().movers[0];assert.deepEqual(switched.position,selected.position,'changing direction keeps current position');
    env.advance(200);const changed=app.adapter.getSimulation().movers[0];assert.notEqual(changed.position[0],switched.position[0]);assert.equal(changed.position[1],switched.position[1]);
    get('clear-room').click();assert.equal(get('selected-object').hidden,true,'clearing removes stale selection');

  }finally{app?.destroy();assert.equal(env.intervals.size,0);env.restore();}
  // A moving obstacle across the empty room's diagonal must trigger avoidance.
  const store=new MissionStore();store.reset('demo');const demo=createDemo(store,26,{room:true});let now=1700000000000;demo.step(now);
  assert.equal(demo.setGoal([14,0,1.8]).ok,true,'newly added room area is navigable');
  assert.equal(demo.setGoal([4.15,-7,1.8]).ok,false,'interior partition blocks destination');
  assert.equal(demo.addObstacle([-11,5.8],'crate').ok,false,'built-in sofa blocks object placement');
  assert.equal(demo.addObstacle([0,0],'moving',{axis:'z'}).ok,false);
  assert.equal(demo.addObstacle([0,10.5],'moving',{axis:'y'}).ok,false,'Y travel checks room Y bounds');
  assert.equal(demo.addObstacle([0,0],'moving').ok,true);assert.equal(demo.addObstacle([3,1],'crate').ok,true);
  const goals=[[8,5,1.8],[-8,5,1.8],[8,-5,1.8],[-8,-5,1.8]];let nextGoal=0;
  assert.equal(demo.setGoal(goals[nextGoal++]).ok,true);
  const prepared=demo.snapshot();now+=1000;demo.step(now);assert.deepEqual(demo.snapshot().position,prepared.position);assert.deepEqual(demo.snapshot().movers,prepared.movers);assert.equal(demo.snapshot().plans,0);
  demo.start();
  let min=Infinity,yields=0;
  for(let i=0;i<3600;i++){if(store.getState().explorationMode==='paused'){const result=demo.setGoal(goals[nextGoal++%goals.length]);if(result.ok)store.resumeExploration();}now+=100;demo.step(now);const s=demo.snapshot();min=Math.min(min,s.clearance);if(s.status.startsWith('Yielding'))yields++;assert.ok(s.clearance>=.45-1e-6);}
  assert.ok(demo.snapshot().completed>=3,'room continues navigating around user obstacles');
  demo.clearObstacles();assert.equal(demo.snapshot().movers.length,0);assert.equal(demo.snapshot().placed.length,0);
  return {simulationSteps:3600,goalsReached:demo.snapshot().completed,minimumClearance:min,yieldFrames:yields};
}
