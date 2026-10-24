import assert from 'node:assert/strict';
import {spawn} from 'node:child_process';
import {createInterface} from 'node:readline';
import {fileURLToPath} from 'node:url';
import {MissionStore} from '../slam_dunk_bridge/web/model.mjs';
import {createAdapter} from '../slam_dunk_bridge/web/adapter.mjs';

// Real sockets and real JS/Python protocol implementations; the destination
// consumer and source pose/map are deliberately synthetic, without ROS imports.
const python=process.argv[2]??'python';
const child=spawn(python,[fileURLToPath(new URL('./control_fixture.py',import.meta.url))],{stdio:['pipe','pipe','pipe']});
let errors='';child.stderr.on('data',chunk=>{errors+=chunk;});
const lines=createInterface({input:child.stdout});
const messages=[];lines.on('line',line=>messages.push(JSON.parse(line)));
const waitFor=async(predicate,label)=>{
  const deadline=Date.now()+5000;
  while(!predicate()){
    if(Date.now()>deadline)throw new Error(`Timed out: ${label}. ${errors}`);
    await new Promise(resolve=>setTimeout(resolve,20));
  }
};
let adapter;
try{
  await waitFor(()=>messages[0]?.port,'sidecar startup');
  const store=new MissionStore();store.reset('live');store.setConnection('connected');
  const now=Date.now(),stamp={sec:Math.floor(now/1000),nanosec:now%1000*1e6};
  store.ingest({type:'pose',protocol_version:1,frame_id:'map',stamp,position:[0,0,2],orientation:[0,0,0,1]},now);
  store.ingest({type:'map',protocol_version:1,frame_id:'map',stamp,operation:'replace',revision:1,points:[[0,0,0]]},now);
  class OriginSocket extends WebSocket {
    constructor(url){super(url,{headers:{Origin:'http://127.0.0.1:8080'}});}
  }
  adapter=createAdapter({store,WebSocketImpl:OriginSocket});
  assert.equal(adapter.connectControl(`ws://127.0.0.1:${messages[0].port}/control`),true);
  await waitFor(()=>store.getState().controlAvailable,'ready control handshake');
  assert.equal(adapter.startSteering('forward').ok,true);
  await waitFor(()=>store.getState().explorationMode==='steering','steering acknowledgment');
  assert.equal(adapter.renewSteering().ok,true);
  assert.equal(adapter.releaseSteering('test release').ok,true);
  await waitFor(()=>store.getState().explorationMode==='autonomous','release restores autonomy');
  assert.equal(adapter.startSteering('left').ok,true);
  await waitFor(()=>store.getState().explorationMode==='steering','second steering acknowledgment');
  // No renewal: browser crash semantics are determined by the Python watchdog.
  await waitFor(()=>store.getState().explorationMode==='autonomous','lease expiry restores autonomy');
  adapter.dispose();adapter=null;child.stdin.end('\n');
  await waitFor(()=>messages[1]?.goals,'fake destination sink summary');
  const goals=messages[1].goals;
  assert.ok(goals.some(p=>p[0]===3&&p[1]===0));
  assert.ok(goals.some(p=>p[0]===0&&p[1]===3));
  assert.deepEqual(goals.at(-1),[10,10,2]);
  console.log('PASS: real JavaScript adapter -> Python sidecar, steering/renew/release/expiry -> autonomous destination');
}finally{
  adapter?.dispose();child.stdin.end();
  if(child.exitCode===null)child.kill();
  lines.close();
}
