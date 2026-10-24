/** Deterministic illustrative indoor scan. No physics or planner validation. */
function environment(seed) {
  const points=[];
  const wall=(a,b)=>{
    const n=Math.ceil(Math.hypot(b[0]-a[0],b[1]-a[1])/.24);
    for(let i=0;i<=n;i++) for(let k=0;k<10;k++) {
      const t=i/Math.max(1,n),jitter=Math.sin(i*17+k*31+seed)*.018;
      points.push([a[0]+(b[0]-a[0])*t+jitter,a[1]+(b[1]-a[1])*t+jitter,k*.3]);
    }
  };
  [[[-13,-9],[13,-9]],[[13,-9],[13,9]],[[13,9],[-13,9]],[[-13,9],[-13,-9]],
   [[-3,-9],[-3,-3]],[[-3,0],[-3,9]],[[4,-9],[4,0]],[[4,3],[4,9]],
   [[-13,2],[-9,2]],[[-6,2],[-3,2]],[[4,-2],[9,-2]],[[11,-2],[13,-2]],
   [[-1,5],[1.5,5]],[[1.5,5],[1.5,7]],[[8,4],[10,4]],[[10,4],[10,6]]].forEach(([a,b])=>wall(a,b));
  return points;
}
const makeStamp=t=>({sec:Math.floor(t/1000),nanosec:Math.floor(t%1000*1e6)});
export function createDemo(store,seed=26) {
  if(!store||typeof store.ingest!=='function'||!Number.isSafeInteger(seed)) throw new TypeError('Demo requires a store and integer seed');
  const geometry=environment(seed),route=[[-9,-5,1.8],[-9,-1,1.8],[-7,-1,1.8],[-7,5,1.8],[-4,5,1.8],[-4,-1.5,1.8],[1,-1.5,1.8],[1,5,1.8],[6,5,1.8],[9,1,1.8],[9,-6,1.8],[1,-6,1.8]];
  let position=[...route[0]],index=1,target=null,activeCommand=null,paused=false,last=null,start=null,lastMap=-Infinity,revision=0,yaw=Math.PI/2,lastMessage=-Infinity;
  const step=now=>{
    if(typeof now!=='number'||!Number.isFinite(now)||now<=0||(last!==null&&now<=last))return false;
    if(store.getState().source!=='demo')return false;
    store.tick(now);
    if(start===null) start=now;
    const dt=last===null?0:Math.min(.25,(now-last)/1000);last=now;
    const state=store.getState(),held=state.explorationMode==='steering'?state.steering:null;
    const stopped=paused||state.explorationMode==='paused';
    const destination=held?position.map((v,i)=>v+held.worldDirection[i]*2):target??route[index],delta=destination.map((v,i)=>v-position[i]),distance=Math.hypot(...delta);
    const speed=stopped?0:Math.min(state.speedLimit*.65,distance/Math.max(.001,dt));
    let velocity=distance>.001?delta.map(v=>v/distance*speed):[0,0,0];
    if(!stopped&&distance>.001) {position=position.map((v,i)=>v+velocity[i]*dt);if(held)position=[Math.max(-12.5,Math.min(12.5,position[0])),Math.max(-8.5,Math.min(8.5,position[1])),Math.max(.5,Math.min(6,position[2]))];if(Math.hypot(velocity[0],velocity[1])>.01)yaw=Math.atan2(velocity[1],velocity[0]);}
    if(!stopped&&!held&&distance<.08) {if(target){position=[...target];velocity=[0,0,0];target=null;paused=true;store.completeDemoCommand(activeCommand);activeCommand=null;}else index=(index+1)%route.length;}
    // Strictly increasing source timestamps even when a test clock has fractions.
    const messageTime=Math.max(now,lastMessage+.001);lastMessage=messageTime;
    store.ingest({type:'pose',protocol_version:1,frame_id:'map',stamp:makeStamp(messageTime),position,orientation:[0,0,Math.sin(yaw/2),Math.cos(yaw/2)],velocity,battery:Math.max(0,87-(now-start)/180000),flight_seconds:145+(now-start)/1000},now);
    if(now-lastMap>=1000) {
      lastMap=now;
      const points=geometry.filter(p=>Math.hypot(p[0]-position[0],p[1]-position[1])<11);
      store.ingest({type:'map',protocol_version:1,frame_id:'map',stamp:makeStamp(messageTime),operation:'replace',revision:++revision,points},now);
    }
    return true;
  };
  return Object.freeze({
    step,
    move(command) {if(!command||command.type!=='move'||!Array.isArray(command.target)||command.target.length!==3||!command.target.every(Number.isFinite))return false;target=[...command.target];activeCommand=command.id;paused=false;return true;},
    clearManual() {target=null;activeCommand=null;paused=false;return true;},
    stop() {paused=true;target=null;activeCommand=null;return true;}
  });
}
