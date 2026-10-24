import assert from 'node:assert/strict';
import {MissionStore} from '../slam_dunk_bridge/web/model.mjs';
import {createAdapter} from '../slam_dunk_bridge/web/adapter.mjs';
import {createDemo} from '../slam_dunk_bridge/web/demo.mjs';

const stamp=t=>({sec:Math.floor(t/1000),nanosec:Math.floor(t%1000*1e6)});
const pose=(t,yaw=0)=>({type:'pose',protocol_version:1,frame_id:'map',stamp:stamp(t),position:[0,0,2],orientation:[0,0,Math.sin(yaw/2),Math.cos(yaw/2)]});
const map=t=>({type:'map',protocol_version:1,frame_id:'map',stamp:stamp(t),operation:'replace',revision:1,points:[[1,1,1]]});
const ready=(epoch=0,extra={})=>({type:'control_status',protocol_version:1,epoch,mode:'autonomous',ready:true,lease_id:null,last_request_id:null,active_request_id:null,accepted:null,reason:'Exploring waypoints',ttl_ms:500,capabilities:{steering:true,pause:false},...extra});

function harness(seed) {
  let now=1700000000000+seed*10000,callback=null;
  const sockets=[];
  class Socket {
    constructor(url){this.url=url;this.readyState=0;this.sent=[];sockets.push(this);}
    open(){this.readyState=1;this.onopen?.();}
    close(){this.readyState=3;this.onclose?.();}
    send(value){assert.equal(this.readyState,1);this.sent.push(JSON.parse(value));}
    message(value){this.onmessage?.({data:typeof value==='string'?value:JSON.stringify(value)});}
  }
  const store=new MissionStore(),adapter=createAdapter({store,WebSocketImpl:Socket,clock:()=>now,schedule:fn=>{callback=fn;return 1;},cancel:()=>{callback=null;}});
  return {store,adapter,sockets,get now(){return now;},advance(ms){now+=ms;callback?.();},live(){adapter.connectLive('ws://127.0.0.1:8765/stream');sockets[0].open();sockets[0].message(pose(now,seed/10));sockets[0].message(map(now));adapter.connectControl('ws://127.0.0.1:8766/control');sockets[1].open();sockets[1].message(ready());return sockets[1];}};
}

export function runHoldStateTests() {
  const counts={},test=(name,fn)=>{fn();counts[name]=(counts[name]??0)+1;};
  for(let i=0;i<300;i++) {
    const t=1700000000000+i*10000,direction=['forward','backward','left','right','up','down'][i%6],yaw=i/17;
    test('model.held-steering',()=>{
      const s=new MissionStore();s.ingest(pose(t,yaw),t);s.ingest(map(t),t);
      assert.equal(s.startSteering(direction,t).ok,true);const first=s.getState().steering;
      assert.equal(first.direction,direction);assert.equal(first.expiresAt,t+500);assert.ok(Object.isFrozen(first.worldDirection));
      assert.ok(Math.abs(Math.hypot(...first.worldDirection)-1)<1e-12);
      s.ingest({...pose(t+1,yaw+1),position:[1,2,2]},t+1);assert.deepEqual(s.getState().steering.worldDirection,first.worldDirection);
      assert.equal(s.startSteering('left',t+2).ok,false);assert.equal(s.renewSteering(t+50+i%400).ok,true);
      assert.equal(s.releaseSteering().ok,true);assert.equal(s.getState().explorationMode,'autonomous');assert.equal(s.getState().steering,null);
      assert.equal(s.releaseSteering().ok,true);assert.equal(s.renewSteering(t+1000).ok,false);
    });
    test('model.lease-expiry-and-pause',()=>{
      const s=new MissionStore();s.ingest(pose(t,yaw),t);s.ingest(map(t),t);s.startSteering(direction,t);
      s.tick(t+499);assert.equal(s.getState().explorationMode,'steering');s.tick(t+500);assert.equal(s.getState().explorationMode,'autonomous');
      assert.equal(s.renewSteering(t+501).ok,false);assert.equal(s.pauseExploration().ok,true);assert.equal(s.getState().explorationMode,'paused');
      assert.equal(s.startSteering(direction,t+502).ok,false);s.releaseSteering();assert.equal(s.getState().explorationMode,'paused');
      s.resumeExploration();assert.equal(s.getState().explorationMode,'autonomous');s.startSteering(direction,t+503);s.setView('history');assert.equal(s.getState().steering,null);
      s.reset('live');assert.equal(s.startSteering(direction,t+504).ok,false);assert.equal(s.pauseExploration().ok,false);assert.equal(s.resumeExploration().ok,false);
    });
    test('model.invalid-control-status',()=>{
      const s=new MissionStore();s.reset('live');assert.equal(s.setControlConnection('connecting'),true);assert.equal(s.ingestControl(ready(i)),true);
      const before=s.getState();const bad=[null,{...ready(i),epoch:-1},{...ready(i),mode:'teleport'},{...ready(i),ttl_ms:10000},{...ready(i),capabilities:{}},{...ready(i),mode:'steering',lease_id:null},{...ready(i),reason:123},{...ready(i),protocol_version:2}][i%8];
      assert.equal(s.ingestControl(bad),false);assert.strictEqual(s.getState(),before);assert.equal(s.ingestControl(ready(i-1)),false);
      assert.equal(s.setControlConnection('disconnected'),true);assert.equal(s.getState().controlAvailable,false);assert.equal(s.getState().explorationMode,'waiting');
    });
    test('adapter.demo-held-release-expiry',()=>{
      const h=harness(i);h.adapter.startDemo();const p=h.store.getState().pose.position;
      assert.equal(h.adapter.startSteering(direction).ok,true);h.advance(100);assert.notDeepEqual(h.store.getState().pose.position,p);
      assert.equal(h.adapter.renewSteering().ok,true);h.advance(100);assert.equal(h.store.getState().explorationMode,'steering');
      assert.equal(h.adapter.releaseSteering('pointerup').ok,true);assert.equal(h.store.getState().explorationMode,'autonomous');
      assert.equal(h.adapter.startSteering('forward').ok,true);h.advance(500);assert.equal(h.store.getState().steering,null);assert.equal(h.store.getState().explorationMode,'autonomous');
      assert.equal(h.adapter.startSteering('forward').ok,true);h.adapter.pauseExploration();assert.equal(h.store.getState().explorationMode,'paused');
      const held=h.store.getState().pose.position;h.advance(100);assert.deepEqual(h.store.getState().pose.position,held);h.adapter.resumeExploration();h.advance(100);assert.equal(h.store.getState().explorationMode,'autonomous');h.adapter.dispose();
    });
    test('adapter.live-lease-correlation',()=>{
      const h=harness(i),control=h.live();assert.equal(h.adapter.startSteering(direction).ok,true);
      const start=control.sent.at(-1);assert.equal(start.type,'steer_start');assert.equal(start.direction,direction);
      assert.equal(h.adapter.renewSteering().pending,true);assert.equal(control.sent.length,1);
      control.message(ready(1,{mode:'steering',lease_id:`lease-${i}`,active_request_id:start.request_id,last_request_id:start.request_id,accepted:true}));
      assert.equal(h.store.getState().explorationMode,'steering');assert.equal(h.adapter.renewSteering().ok,true);assert.equal(control.sent.at(-1).type,'steer_renew');assert.equal(control.sent.at(-1).epoch,1);
      h.adapter.releaseSteering('pointerup');assert.equal(control.sent.at(-1).type,'steer_release');assert.equal(h.adapter.startSteering(direction).ok,false);
      control.message(ready(2));assert.equal(h.adapter.startSteering(direction).ok,true);h.adapter.dispose();
    });
    test('adapter.pending-start-cancel-race',()=>{
      const h=harness(i),control=h.live();h.adapter.startSteering(direction);const start=control.sent.at(-1);
      h.adapter.releaseSteering('keyup');const cancel=control.sent.at(-1);assert.equal(cancel.type,'steer_cancel');assert.equal(cancel.request_id,start.request_id);assert.equal(cancel.epoch,start.epoch);
      control.message(ready(1,{mode:'steering',lease_id:'late',active_request_id:start.request_id,last_request_id:start.request_id,accepted:true}));
      const sent=control.sent.length;assert.equal(h.adapter.renewSteering().ok,false);assert.equal(control.sent.length,sent);assert.equal(h.adapter.startSteering(direction).ok,false);
      control.message(ready(2,{last_request_id:start.request_id,accepted:true}));assert.equal(h.adapter.startSteering(direction).ok,true);h.adapter.dispose();
    });
    test('adapter.readonly-stale-and-source-isolation',()=>{
      const h=harness(i);h.adapter.connectLive('ws://127.0.0.1:8765/stream');h.sockets[0].open();h.sockets[0].message(pose(h.now));h.sockets[0].message(map(h.now));
      assert.equal(h.adapter.startSteering(direction).ok,false);assert.equal(h.adapter.connectControl('https://bad/control'),false);
      h.adapter.connectControl('ws://127.0.0.1:8766/control');const c=h.sockets.at(-1);c.open();c.message(ready());h.adapter.startSteering(direction);
      const start=c.sent.at(-1);c.message(ready(1,{mode:'steering',lease_id:'owned',active_request_id:start.request_id,last_request_id:start.request_id,accepted:true}));
      assert.equal(h.adapter.pauseExploration().ok,false);h.adapter.releaseSteering();c.message(ready(2));
      h.adapter.startSteering(direction);h.advance(1600);assert.equal(h.store.getState().controlAvailable,false);assert.equal(h.adapter.renewSteering().ok,false);
      const late=c.onmessage;h.adapter.startDemo();late?.({data:JSON.stringify(ready(3))});assert.equal(h.store.getState().source,'demo');assert.equal(h.store.getState().controlConnection,'demo');h.adapter.dispose();
    });
    test('adapter.gateway-restart-and-old-callback',()=>{
      const h=harness(i),old=h.live();old.message(ready(50+i));assert.equal(h.store.getState().controlEpoch,50+i);
      const oldCallback=old.onmessage;h.adapter.connectControl('ws://127.0.0.1:8766/control');const fresh=h.sockets.at(-1);fresh.open();fresh.message(ready());
      assert.equal(h.store.getState().controlEpoch,0);assert.equal(h.store.getState().controlAvailable,true);
      oldCallback({data:JSON.stringify(ready(999999,{mode:'fault'}))});assert.equal(h.store.getState().controlEpoch,0);assert.equal(h.store.getState().explorationMode,'autonomous');
      assert.equal(h.adapter.startSteering(direction).ok,true);h.adapter.releaseSteering();h.adapter.dispose();
    });
    test('adapter.foreign-lease-and-unsupported-pause',()=>{
      const h=harness(i),control=h.live();h.adapter.startSteering(direction);const start=control.sent.at(-1);
      control.message(ready(0,{mode:'steering',lease_id:'foreign',active_request_id:'another-operator',last_request_id:start.request_id,accepted:false}));
      assert.equal(h.adapter.renewSteering().ok,false);
      control.message(ready(1));h.adapter.startSteering(direction);const own=control.sent.at(-1);
      control.message(ready(1,{mode:'steering',lease_id:'own',active_request_id:own.request_id,last_request_id:own.request_id,accepted:true}));
      const before=control.sent.length;assert.equal(h.adapter.pauseExploration().ok,false);assert.equal(h.adapter.resumeExploration().ok,false);assert.equal(control.sent.length,before);
      assert.equal(h.adapter.renewSteering().ok,true);assert.equal(control.sent.at(-1).lease_id,'own');h.adapter.dispose();
    });
    test('demo.heading-fixed-autonomy-restored',()=>{
      const s=new MissionStore(),d=createDemo(s,i);d.step(t);const p=s.getState().pose.position;
      s.startSteering('right',t);const world=s.getState().steering.worldDirection;d.step(t+100);const q=s.getState().pose.position;
      assert.ok((q[0]-p[0])*world[0]+(q[1]-p[1])*world[1]>0);
      s.renewSteering(t+200);d.step(t+200);s.releaseSteering();d.step(t+300);assert.equal(s.getState().explorationMode,'autonomous');assert.equal(s.getState().steering,null);
      s.pauseExploration();const held=s.getState().pose.position;d.step(t+400);assert.deepEqual(s.getState().pose.position,held);s.resumeExploration();d.step(t+500);assert.notDeepEqual(s.getState().pose.position,held);
    });
  }
  return {passed:Object.values(counts).reduce((a,b)=>a+b,0),cases:counts};
}

if(globalThis.process?.argv[1]?.endsWith('hold_state.test.mjs'))console.log(JSON.stringify(runHoldStateTests(),null,2));
