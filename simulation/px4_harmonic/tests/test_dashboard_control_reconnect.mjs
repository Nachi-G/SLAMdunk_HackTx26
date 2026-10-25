import assert from 'node:assert/strict';
import {MissionStore} from '../../../integration/slam_dunk_bridge/slam_dunk_bridge/web/model.mjs';
import {createAdapter} from '../../../integration/slam_dunk_bridge/slam_dunk_bridge/web/adapter.mjs';
let now=1700000000000,tick;const sockets=[];
class Socket {
  constructor(url){this.url=url;this.readyState=0;this.sent=[];sockets.push(this);}
  open(){this.readyState=1;this.onopen?.();}
  close(){this.readyState=3;this.onclose?.();}
  send(data){this.sent.push(JSON.parse(data));}
  message(data){this.onmessage?.({data:JSON.stringify(data)});}
}
const store=new MissionStore(),adapter=createAdapter({store,WebSocketImpl:Socket,clock:()=>now,schedule:fn=>(tick=fn,1),cancel:()=>{tick=null;}});
adapter.connectLive('ws://localhost:8765/stream');sockets[0].open();
adapter.connectControl('ws://localhost:8766/control');sockets[1].close();
now+=1600;tick();assert.equal(sockets.length,3,'retry after server unavailable');
const control=sockets[2];control.open();
const stamp={sec:Math.floor(now/1000),nanosec:0};
sockets[0].message({protocol_version:1,type:'pose',stamp,frame_id:'gazebo_world',position:[0,0,2],orientation:[0,0,0,1]});
sockets[0].message({protocol_version:1,type:'map',stamp,frame_id:'gazebo_world',operation:'replace',revision:1,points:[[3,0,2]]});
control.message({type:'control_status',protocol_version:1,epoch:0,mode:'autonomous',ready:true,lease_id:null,reason:'Navigation',ttl_ms:500,capabilities:{steering:true,pause:false}});
assert.equal(adapter.startSteering('forward').ok,true);
assert.equal(control.sent.at(-1).type,'steer_start');
control.close();assert.equal(adapter.renewSteering().ok,false);
now+=1600;tick();assert.equal(sockets.length,4,'retry after dropped connection');
assert.equal(sockets[3].sent.length,0,'never revive a held press on reconnect');
adapter.startDemo();const count=sockets.length;now+=1600;tick();assert.equal(sockets.length,count,'source change cancels retries');
adapter.dispose();console.log('Control startup retry, reconnect, command delivery and no held-command revival: PASS');
