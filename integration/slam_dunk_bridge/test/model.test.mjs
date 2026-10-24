import assert from 'node:assert/strict';
import {MissionStore} from '../slam_dunk_bridge/web/model.mjs';
import {createAdapter} from '../slam_dunk_bridge/web/adapter.mjs';
import {createDemo} from '../slam_dunk_bridge/web/demo.mjs';

const stamp=t=>({sec:Math.floor(t/1000),nanosec:Math.round((t%1000)*1e6)});
const pose=(t,p=[0,0,2],q=[0,0,0,1])=>({protocol_version:1,type:'pose',frame_id:'map',stamp:stamp(t),position:p,orientation:q});
const map=(t,points=[[1,2,3]])=>({protocol_version:1,type:'map',frame_id:'map',stamp:stamp(t),revision:1,operation:'replace',points});

export async function runModelTests() {
  const counts={};
  const test=(name,fn)=>{fn();counts[name]=(counts[name]??0)+1;};
  const loops=300;
  for(let i=0;i<loops;i++) {
    const t=1700000000000+i*10000,s=new MissionStore();
    test('MissionStore.constructor',()=>{assert.equal(s.getState().source,'demo');assert.equal(s.getState().pose,null);assert.ok(Object.isFrozen(s.getState()));});
    test('MissionStore.getState',()=>{assert.strictEqual(s.getState(),s.getState());assert.equal(s.getState().speed,null);assert.equal(s.getState().profile.inflation_hgp,.45);});
    test('MissionStore.subscribe',()=>{let n=0;const un=s.subscribe(()=>n++);assert.equal(n,1);s.setProfile(i%101);assert.equal(n,2);un();s.setProfile(0);assert.equal(n,2);});
    test('MissionStore.reset',()=>{s.ingest(map(t),t);s.ingest(pose(t),t);s.reset(i%2?'demo':'live');assert.equal(s.getState().history.length,0);assert.equal(s.getState().combinedPoints.length,0);assert.equal(s.getState().pose,null);assert.equal(s.getState().command,null);assert.equal(s.reset('invalid'),false);s.reset('demo');});
    test('MissionStore.setConnection',()=>{assert.equal(s.setConnection('connected'),true);assert.equal(s.setConnection('disconnected'),true);assert.equal(s.getState().stale.pose,true);assert.equal(s.setConnection('other'),false);s.setConnection('demo');});
    test('MissionStore.ingest.valid',()=>{const points=[[i/10,-i,2],[i+.1,i/4,0]];assert.equal(s.ingest(map(t,points),t),true);points[0][0]=999;assert.equal(s.getState().mapPoints[0][0],i/10);const q=[0,0,Math.sin(i/20)*3,Math.cos(i/20)*3];assert.equal(s.ingest(pose(t,[i/10,0,2],q),t),true);assert.ok(Math.abs(Math.hypot(...s.getState().pose.orientation)-1)<1e-12);});
    test('MissionStore.ingest.invalid',()=>{const before=s.getState().pose;const bad=[null,{}, {...pose(t+1),protocol_version:2},{...pose(t+1),frame_id:'wrong'},pose(t+1,[NaN,0,2]),pose(t+1,[0,0,2],[0,0,0,0]),{...map(t+1),operation:'append'},map(t+1,[[Infinity,0,0]]),{...pose(t+1),velocity:[0,NaN,0]},pose(t),{...pose(t+1),velocity:[1.79e308,1.79e308,0]},{...pose(t+1),stamp:{sec:Number.MAX_SAFE_INTEGER,nanosec:0}}];assert.equal(s.ingest(bad[i%bad.length],t+1),false);assert.strictEqual(s.getState().pose,before);});
    test('MissionStore.tick',()=>{s.tick(t+2999);assert.equal(s.getState().stale.pose,false);s.tick(t+3001);assert.equal(s.getState().stale.pose,true);s.ingest(pose(t+5000),t+5000);assert.equal(s.getState().stale.pose,false);assert.equal(s.tick(NaN),false);});
    test('MissionStore.getDisplay',()=>{const display=s.getDisplay();assert.strictEqual(display.pose,s.getState().pose);assert.strictEqual(display.points,s.getState().mapPoints);assert.ok(Object.isFrozen(display));});
    test('MissionStore.command',()=>{const direction=['forward','backward','left','right','up','down'][i%6],p=s.getState().pose.position;const result=s.command(direction);assert.equal(result.ok,true);assert.ok(result.command.target.every(Number.isFinite));assert.ok(Math.hypot(...result.command.target.map((v,j)=>v-p[j]))<=1.000001);assert.equal(s.command(['bad',null,undefined,{},'__proto__'][i%5]).ok,false);});
    test('MissionStore.stop',()=>{assert.equal(s.stop().ok,true);assert.equal(s.getState().command.phase,'holding');});
    test('MissionStore.completeDemoCommand',()=>{const r=s.command('forward',.1+(i%20)/10);assert.equal(r.ok,true);assert.equal(s.completeDemoCommand(r.command.id+1),false);assert.equal(s.completeDemoCommand(r.command.id),true);assert.equal(s.getState().command.phase,'reached');assert.equal(s.completeDemoCommand(r.command.id),false);assert.equal(s.command('forward',NaN).ok,false);});
    test('MissionStore.setProfile',()=>{const value=i/299*100;assert.equal(s.setProfile(value),true);const p=s.getState().profile;assert.ok(p.inflation_hgp>=.45&&p.inflation_hgp<=.9);assert.ok(p.dyn_base_inflation_m>=.1&&p.dyn_base_inflation_m<=.3);assert.equal(p.applied,null);assert.equal(s.setProfile([NaN,Infinity,-1,101,'50'][i%5]),false);});
    test('MissionStore.setSpeedMode',()=>{const mode=i%2?'precision':'standard';assert.equal(s.setSpeedMode(mode),true);assert.equal(s.getState().speedLimit,mode==='precision'?1:2);assert.equal(s.setSpeedMode('unsafe'),false);});
    test('MissionStore.setView',()=>{assert.equal(s.setView('history'),true);assert.equal(s.command('forward').ok,false);assert.equal(s.setView('other'),false);assert.equal(s.setView('live'),true);});
    test('MissionStore.seek',()=>{s.setView('history');const history=s.getState().history;assert.equal(s.seek(t+i*100),true);assert.ok(s.getState().selectedTime>=history[0].timestamp&&s.getState().selectedTime<=history.at(-1).timestamp);assert.ok(s.getDisplay().timestamp<=s.getState().selectedTime);assert.equal(s.seek(NaN),false);});
    test('MissionStore.setPlaying',()=>{assert.equal(s.setPlaying(true),true);assert.equal(s.getState().playing,true);assert.equal(s.setPlaying(false),true);assert.equal(s.setPlaying(1),false);s.setView('live');assert.equal(s.setPlaying(true),false);});
    test('MissionStore.live-gating',()=>{s.reset('live');s.setConnection('connected');s.ingest(pose(t),t);assert.equal(s.command('forward').ok,false);assert.equal(s.stop().ok,false);assert.equal(s.setProfile(i%101),true);assert.equal(s.getState().profile.state,'proposal');assert.equal(s.getState().battery,null);assert.equal(s.getState().flightSeconds,null);assert.equal(s.setConnection('demo'),false);});

    test('createDemo',()=>{const dstore=new MissionStore(),d=createDemo(dstore,i);assert.equal(d.step(t),true);assert.ok(dstore.getState().mapPoints.length>100);assert.ok(dstore.getState().mapPoints.length<20000);assert.equal(dstore.getState().battery,87);assert.equal(d.step(t),false);assert.equal(d.step(NaN),false);const p=dstore.getState().pose.position;assert.equal(d.move({type:'move',target:[p[0]+1,p[1],p[2]]}),true);d.step(t+100);assert.ok(dstore.getState().pose.position[0]>p[0]);assert.equal(d.stop(),true);const held=dstore.getState().pose.position;d.step(t+200);assert.deepEqual(dstore.getState().pose.position,held);assert.equal(d.move({type:'move',target:[NaN,0,2]}),false);});

    const astore=new MissionStore();let now=t,scheduled=null,clears=0;const sockets=[];
    class FakeSocket {constructor(url){this.url=url;sockets.push(this);}close(){this.onclose?.();}}
    const adapter=createAdapter({store:astore,WebSocketImpl:FakeSocket,clock:()=>now,schedule:fn=>{scheduled=fn;return 7;},cancel:()=>{clears++;scheduled=null;}});
    test('createAdapter',()=>{assert.ok(adapter);assert.ok(Object.isFrozen(adapter));});
    test('adapter.startDemo',()=>{assert.equal(adapter.startDemo(),true);assert.equal(astore.getState().source,'demo');assert.ok(astore.getState().pose);now+=100;scheduled();});
    test('adapter.command',()=>{assert.equal(adapter.command('forward').ok,true);now+=100;scheduled();assert.equal(adapter.command('invalid').ok,false);});
    test('adapter.stop',()=>{assert.equal(adapter.stop().ok,true);now+=100;scheduled();assert.equal(astore.getState().speed,0);});
    test('adapter.setProfile',()=>{assert.equal(adapter.setProfile(i%101),true);assert.equal(astore.getState().profile.value,i%101);});
    test('adapter.setSpeedMode',()=>{assert.equal(adapter.setSpeedMode(i%2?'precision':'standard'),true);assert.equal(adapter.setSpeedMode(null),false);});
    test('adapter.connectLive',()=>{assert.equal(adapter.connectLive('ws://127.0.0.1:8765/stream'),true);assert.equal(astore.getState().source,'live');assert.equal(astore.getState().pose,null);const socket=sockets.at(-1);socket.onopen();assert.equal(astore.getState().connection,'connected');socket.onmessage({data:JSON.stringify(pose(now))});assert.ok(astore.getState().pose);assert.equal(adapter.command('forward').ok,false);socket.onmessage({data:'bad json'});assert.ok(astore.getState().rejected>0);socket.close();assert.equal(astore.getState().connection,'disconnected');now+=1600;scheduled();assert.equal(astore.getState().connection,'connecting');assert.equal(adapter.connectLive('https://bad/stream'),false);assert.equal(adapter.connectLive('ws://bad/control'),false);});
    test('adapter.dispose',()=>{assert.equal(adapter.dispose(),true);assert.equal(scheduled,null);assert.ok(clears>=2);assert.equal(adapter.dispose(),true);});

    test('program-sequence',()=>{
      const program=new MissionStore(),d=createDemo(program,i+1000);
      for(let k=0;k<25;k++) {d.step(t+k*150);if(k%7===0)program.setProfile((i+k)%101);if(k%5===0){const r=program.command(['forward','left','right'][k%3]);if(r.ok)d.move(r.command);}}
      const livePose=program.getState().pose;
      program.setView('history');program.seek(t+800);assert.ok(program.getDisplay().timestamp<=program.getState().selectedTime);assert.equal(program.command('left').ok,false);
      d.step(t+5000);assert.notStrictEqual(program.getState().pose,livePose);assert.ok(program.getDisplay().timestamp<=program.getState().selectedTime);
      program.setView('live');assert.strictEqual(program.getDisplay().pose,program.getState().pose);
      program.reset('live');assert.equal(program.getState().combinedPoints.length,0);assert.equal(program.getState().history.length,0);assert.equal(program.stop().ok,false);
    });
  }
  test('bounds-and-empty',()=>{
    const s=new MissionStore(),t=1700000000000;
    const points=Array.from({length:20000},(_,i)=>[i*.2,0,1]);
    assert.equal(s.ingest(map(t,points),t),true);assert.equal(s.getState().combinedPoints.length,12000);
    assert.equal(s.ingest(map(t+1000,[]),t+1000),true);assert.equal(s.getState().mapPoints.length,0);assert.equal(s.getState().combinedPoints.length,12000);
    assert.equal(s.ingest(map(t+2000,Array.from({length:20001},()=>[0,0,0])),t+2000),false);
    for(let i=0;i<650;i++)s.ingest(pose(t+3000+i*600,[i,0,2]),t+3000+i*600);
    assert.equal(s.getState().trail.length,500);assert.ok(s.getState().history.length<=240);assert.ok(s.getState().history.length>=100);assert.ok(s.getState().history.reduce((n,f)=>n+f.points.length+f.combinedPoints.length,0)<=240000);
  });
  test('freshness-source-age-regression',()=>{
    const s=new MissionStore(),t=1700000000000;
    s.ingest(pose(t),t);s.ingest(map(t),t);
    s.ingest({protocol_version:1,type:'status',mode:'map_snapshot',age_seconds:{pose:0,map:0}},t);
    s.tick(t+5000);assert.equal(s.getState().stale.pose,true);
    s.ingest(pose(t+6000),t+6000);assert.equal(s.getState().stale.pose,false);assert.equal(s.getState().stale.map,true);
  });
  test('cross-source-history-regression',()=>{
    const s=new MissionStore(),t=1700000000000;
    s.ingest(pose(t+2000,[8,0,2]),t);s.ingest(map(t,[[0,0,0]]),t+1);
    s.setView('history');s.seek(t);assert.ok(s.getDisplay().timestamp>=t+2000);
    for(const frame of s.getState().history)assert.ok(frame.timestamp>=t+2000);
  });
  return {passed:Object.values(counts).reduce((a,b)=>a+b,0),cases:counts};
}

if(globalThis.process?.argv[1]?.endsWith('model.test.mjs')) console.log(JSON.stringify(await runModelTests(),null,2));
