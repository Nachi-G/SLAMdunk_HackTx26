import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import {mountDashboard} from '../slam_dunk_bridge/web/app.mjs';
import {createTestEnvironment,createTestMarkup} from './ui_integration.test.mjs';

export function runBackupTests(){
  const env=createTestEnvironment(2000),doc=createTestMarkup(),get=id=>doc.getElementById(id);let app;
  try{
    app=mountDashboard({document:doc});env.advance(100);
    assert.equal(get('main-canvas').hidden,false,'Nachi map is the default view');
    assert.equal(get('pov-canvas').hidden,false,'Drone POV runs in the camera panel');
    get('view-arena').click();
    assert.equal(get('pov-canvas').hidden,false,'editing the map keeps the camera running');
    get('view-rgb').click();assert.equal(get('pov-canvas').hidden,true);
    get('view-pov').click();assert.equal(get('pov-canvas').hidden,false);
    assert.equal(get('main-canvas').hidden,false,'camera modes never replace the map');
    assert.equal(get('camera-state').textContent,'DRONE POV · SIMULATED');
    get('camera-url').value='http://robot:8081/stream';get('camera-form').fire('submit');
    assert.equal(get('pov-canvas').hidden,true,'connecting RGB selects the stream');
    assert.equal(get('camera-image').hidden,false);
    get('view-pov').click();assert.equal(get('camera-image').hidden,true);
    get('view-rgb').click();assert.equal(get('camera-image').src,'http://robot:8081/stream');
    assert.equal(get('main-canvas').hidden,false);
    get('view-pov').click();
    get('tool-add').click();get('obstacle-kind').value='moving';get('movement-axis').value='x';
    const click=p=>{const q=app.scenes.main.project(p),e={clientX:q[0],clientY:q[1],pointerId:91,button:0};get('main-canvas').fire('pointerdown',e);get('main-canvas').fire('pointerup',e);};
    click([-8.5,-2,0]);
    let sim=app.adapter.getSimulation(),actor=sim.movers.find(o=>o.id==='M1');
    assert.ok(actor,'dashboard places a moving object through its event handlers');
    assert.equal(actor.axis,'x');assert.equal(get('obstacle-count').textContent,'1 / 24');
    get('tool-goal').click();click(actor.position);
    assert.equal(get('selected-object').hidden,false);
    get('selected-motion-y').click();
    assert.equal(app.adapter.getSimulation().movers.find(o=>o.id==='M1').axis,'y');
    const start=app.adapter.getSimulation().movers.find(o=>o.id==='M1').position;
    for(let i=0;i<30;i++)env.advance(100);
    actor=app.adapter.getSimulation().movers.find(o=>o.id==='M1');
    assert.notDeepEqual(actor.position,start,'added object actually moves');
    assert.ok(app.adapter.getSimulation().clearance>=.45-1e-6);
    get('tool-remove').click();click(actor.position);
    assert.equal(app.adapter.getSimulation().movers.length,3,'built-in patrols survive removal');
    get('tool-add').click();get('obstacle-kind').value='crate';click([-8.5,-2,0]);
    assert.equal(app.adapter.getSimulation().placed.length,1);
    get('tool-goal').click();click([-8.5,-2,0]);get('selected-motion-x').click();
    assert.equal(app.adapter.getSimulation().placed.length,0,'static object converts to motion');
    assert.equal(app.adapter.getSimulation().movers.find(o=>o.id==='M2').axis,'x');
    get('clear-obstacles').click();assert.equal(app.adapter.getSimulation().movers.length,3);
    get('stop').click();get('mission-goal').value='2';get('send-goal').click();
    assert.equal(app.store.getState().explorationMode,'autonomous','new target resumes a paused simulation');
    assert.deepEqual(app.adapter.getSimulation().goal,[1,7,1.8]);
    get('source').value='live';get('source').fire('change');
    const exploration={mode:'frontier',state:'exploring',reason:'Following a reachable frontier',map_revision:3,scan_sequence:8,frontier_cells:12,reachable_cells:90,unknown_voxels:100,free_voxels:90,occupied_voxels:20,failed_viewpoints:0,goal:[2,3,2],completion_scope:'configured bounds, horizontal slice'};
    const status={type:'control_status',protocol_version:1,epoch:1,mode:'autonomous',ready:true,lease_id:null,reason:'Exploring',ttl_ms:500,capabilities:{steering:true,pause:false},exploration};
    assert.equal(app.store.ingestControl(status),true);
    assert.equal(get('exploration-panel').hidden,false);assert.equal(get('exploration-frontiers').textContent,'12');
    exploration.goal[0]=999;assert.equal(app.store.getState().exploration.goal[0],2,'telemetry snapshots are immutable');
    app.store.ingestControl({...status,exploration:{...exploration,unknown_voxels:-1}});
    assert.equal(app.store.getState().exploration,null,'malformed optional metrics are ignored');
    app.store.ingestControl(status);app.store.setControlConnection('disconnected');
    assert.equal(get('exploration-panel').hidden,true,'disconnected metrics cannot appear current');
    assert.equal(app.adapter.editSimulation('add',[0,0],'moving').ok,false);
    assert.equal(app.adapter.setSimulationGoal([1,2,2]).ok,false);
    for(const page of ['index.html','dashboard.html']){
      const html=readFileSync(new URL(`../slam_dunk_bridge/web/${page}`,import.meta.url),'utf8');
      assert.match(html,/href="\/flight-room.html"/);assert.match(html,/id="camera-image"/);assert.match(html,/id="history-screen"/);
    }
    return {dashboardDynamicObjects:1,targetResume:1,liveExploration:1,entryRoutes:2};
  }finally{app?.destroy();assert.equal(env.intervals.size,0);env.restore();}
}
