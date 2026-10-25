import {ROOM_OBSTACLES,ROOM_FIXTURES,ROOM_BOUNDS} from './room-layout.mjs';
import {geometryPoints,BODY_RADIUS,sweptDistance,STATIC,WALLS,rectDistance,MISSIONS,moversAt,free,segmentFree,clearance,planPath,cubicSegment} from './planner.mjs';
const makeStamp=t=>({sec:Math.floor(t/1000),nanosec:Math.floor(t%1000*1e6)});
export function createDemo(store,seed=26,{room=false}={}) {
  if(!store||typeof store.ingest!=='function'||!Number.isSafeInteger(seed))throw new TypeError('Demo requires a store and integer seed');
  const bodyRadius=room?.62:BODY_RADIUS,footprintPadding=bodyRadius-BODY_RADIUS;
  const base=room?ROOM_OBSTACLES:STATIC,course=room?[[-8,-5,1.8],[8,5,1.8],[-8,5,1.8],[8,-5,1.8]]:MISSIONS;
  let obstacles=[...base],placed=[],geometry=geometryPoints(seed,base),moving=[],nextObject=1;
  let position=[...course[0]],index=1,target=null,activeCommand=null,paused=false,last=null,start=null,lastMap=-Infinity,revision=0,yaw=Math.PI/2,lastMessage=-Infinity;
  let started=!room;
  let targetPinned=false,resumeAfter=0,wasTurning=false,turnExpiresAt=0;
  let path=[],segment=null,elapsed=0,simTime=0,lastPlan=-Infinity,plans=0,completed=0,status='Planning',lastProfile=null,wait=0;
  const bounds=room?ROOM_BOUNDS:[-13,-9,13,9];
  const amplitudeAt=(x,y,axis)=>Math.min(2.5,axis==='x'?Math.min(x-bounds[0],bounds[2]-x)-1.4:Math.min(y-bounds[1],bounds[3]-y)-1.4);
  const actorTimes=room?[]:[0,0,0],actorRates=room?[]:[1,1,1];
  const fixedActorCount=room?0:3;
  const rawActors=t=>[...(room?[]:moversAt(t)),...moving.map(o=>({id:o.id,position:[o.x+(o.axis==='x'?o.amplitude*Math.sin(t*.55):0),o.y+(o.axis==='y'?o.amplitude*Math.sin(t*.55):0),1.1],radius:.55,axis:o.axis}))];
  const actors=t=>actorTimes.map((time,i)=>rawActors(time+(t-simTime)*actorRates[i])[i]);
  const goal=()=>target??(room?null:course[index]);
  function replan(state){path=planPath(position,goal(),state.profile.inflation_hgp+footprintPadding,simTime,state.profile.dyn_base_inflation_m,actors,obstacles);segment=null;elapsed=0;lastPlan=last/1000;lastProfile=`${state.profile.value}/${state.speedLimit}`;plans++;status=path.length?'Following cubic trajectory':'No feasible route';}
  function movementFree(from,to,margin,base,dt){
    // Preserve the separate furnished dashboard demo's original timing model.
    if(!room&&!moving.length)return segmentFree(from,to,margin,simTime,dt,true,base,actors,obstacles);
    const startTime=simTime-dt;
    if(!segmentFree(from,to,margin,startTime,dt,true,base,actors,obstacles))return false;
    // Relative-motion sweeps catch crossings between the sample timestamps.
    // A small buffer covers the curvature between 20 ms sine/cubic samples.
    const n=Math.max(1,Math.ceil(dt/.02)),required=bodyRadius+margin-footprintPadding+base+.005;
    for(let k=0;k<n;k++){
      const u=k/n,v=(k+1)/n,a=from.map((p,i)=>p+(to[i]-p)*u),b=from.map((p,i)=>p+(to[i]-p)*v);
      const before=actors(startTime+dt*u),after=actors(startTime+dt*v);
      for(let j=0;j<before.length;j++)if(sweptDistance(a,b,before[j].position,after[j].position)<required+before[j].radius)return false;
    }
    return true;
  }
  const step=now=>{
    if(typeof now!=='number'||!Number.isFinite(now)||now<=0||(last!==null&&now<=last)||store.getState().source!=='demo')return false;
    store.tick(now);if(start===null)start=now;
    const dt=last===null?0:Math.min(.25,(now-last)/1000);last=now;
    const state=store.getState(),held=state.explorationMode==='steering'?state.steering:null,stopped=room&&!started||paused||state.explorationMode==='paused';
    if(wasTurning&&!held?.yawRate)resumeAfter=Math.max(resumeAfter,Math.min(now,turnExpiresAt)+500);
    wasTurning=!!held?.yawRate;if(wasTurning)turnExpiresAt=held.expiresAt;
    let velocity=[0,0,0];
    // Pause freezes the entire synthetic world, including patrols.
    if(!stopped){
      const margin=state.profile.inflation_hgp+footprintPadding,base=state.profile.dyn_base_inflation_m;
      // Synthetic patrols independently yield at the inflated vehicle boundary.
      // Other patrols continue moving while a close actor waits for the drone.
      for(let i=0;i<actorTimes.length;i++){
        actorRates[i]=Array.from({length:6},(_,k)=>rawActors(actorTimes[i]+dt*k/5)[i]).every(o=>Math.hypot(position[0]-o.position[0],position[1]-o.position[1])>=o.radius+.24+margin+base+.02&&(room||i>=fixedActorCount?obstacles:placed).every(r=>rectDistance(o.position,r)>o.radius+.1)&&(!room&&i<fixedActorCount||actors(simTime).every((other,j)=>j===i||Math.hypot(o.position[0]-other.position[0],o.position[1]-other.position[1])>o.radius+other.radius+.1)))?1:0;
        actorTimes[i]+=dt*actorRates[i];
      }
      simTime+=dt;
      if(held){
        path=[];segment=null;lastPlan=-Infinity;
        if(held.yawRate){yaw=Math.atan2(Math.sin(yaw+held.yawRate*dt),Math.cos(yaw+held.yawRate*dt));status='Manual turn · release for 0.5 s handoff';}
        else {
        const next=position.map((v,i)=>v+held.worldDirection[i]*state.speedLimit*.65*dt);next[2]=Math.max(.5,Math.min(2.5,next[2]));
        if(movementFree(position,next,margin,base,dt)){velocity=next.map((v,i)=>(v-position[i])/Math.max(dt,.001));position=next;status='Steering · collision guard active';}else status='Steering blocked by clearance';
        }
      }else if(now<resumeAfter){status='Manual turn released · autonomy resumes in 0.5 s';
      }else if(!room||target){
        if(lastProfile!==`${state.profile.value}/${state.speedLimit}`||!path.length&&now/1000-lastPlan>1||now/1000-lastPlan>(room?.35:1)&&wait>.2)replan(state);
        if(path.length>1&&!segment){segment=cubicSegment(position,path[1],state.speedLimit*.65);elapsed=0;}
        if(segment){
          const next=segment.sample(elapsed+dt);
          const look=segment.sample(elapsed+dt+.7).position;
          if(movementFree(position,next.position,margin,base,dt)&&segmentFree(next.position,look,margin,room?simTime:simTime+dt,.7,true,base,actors,obstacles)){
            position=next.position;velocity=next.velocity;elapsed+=dt;wait=0;status='Following cubic trajectory';
            if(elapsed>=segment.duration){path.shift();segment=null;}
          }else{wait+=dt;status='Yielding · predicted obstacle';}
        }
        if(Math.hypot(...goal().map((v,i)=>v-position[i]))<.08){completed++;if(target){paused=true;if(activeCommand)store.completeDemoCommand(activeCommand);else store.pauseExploration();activeCommand=null;status='Goal reached';}else{index=(index+1)%course.length;path=[];lastPlan=-Infinity;}}
      }else status='Ready · choose a destination';
    }else status=room&&!started?'Ready · click Start':'Simulation paused';
    if(Math.hypot(velocity[0],velocity[1])>.01)yaw=Math.atan2(velocity[1],velocity[0]);
    const messageTime=Math.max(now,lastMessage+.001);lastMessage=messageTime;
    store.ingest({type:'pose',protocol_version:1,frame_id:'map',stamp:makeStamp(messageTime),position,orientation:[0,0,Math.sin(yaw/2),Math.cos(yaw/2)],velocity,battery:Math.max(0,87-(now-start)/180000),flight_seconds:145+(now-start)/1000},now);
    if(now-lastMap>=1000){lastMap=now;store.ingest({type:'map',protocol_version:1,frame_id:'map',stamp:makeStamp(messageTime),operation:'replace',revision:++revision,points:geometry},now);}
    return true;
  };
  function changedScene(){obstacles=[...base,...placed];geometry=geometryPoints(seed,obstacles);path=[];segment=null;lastPlan=-Infinity;lastProfile=null;lastMap=-Infinity;status='Scene changed · replanning';}
  return Object.freeze({step,
    start(){started=true;paused=false;return {ok:true};},
    addObstacle(point,kind='crate',{axis='x'}={}){
      if(!Array.isArray(point)||point.length<2||!point.slice(0,2).every(Number.isFinite)||!['crate','barrier','moving'].includes(kind))return {ok:false,reason:'Choose a valid position and obstacle type'};
      if(kind==='moving'&&!['x','y'].includes(axis))return {ok:false,reason:'Choose left–right or up–down movement'};
      if(placed.length+moving.length>=24)return {ok:false,reason:'24 obstacle limit reached. Remove or clear some first.'};
      const [x,y]=point,hx=kind==='barrier'?1.5:.55,hy=kind==='barrier'?.3:.55,r=[x-hx,y-hy,x+hx,y+hy],s=store.getState();
      if(r[0]<bounds[0]+.5||r[2]>bounds[2]-.5||r[1]<bounds[1]+.5||r[3]>bounds[3]-.5)return {ok:false,reason:'Place obstacles inside the building'};
      if(obstacles.some(b=>r[0]<b[2]+.05&&r[2]>b[0]-.05&&r[1]<b[3]+.05&&r[3]>b[1]-.05))return {ok:false,reason:'That overlaps existing geometry'};
      const padding=bodyRadius+s.profile.inflation_hgp+s.profile.dyn_base_inflation_m+.05;
      if(rectDistance(position,r)<padding||goal()&&rectDistance(goal(),r)<padding)return {ok:false,reason:'Leave clearance around the drone and active goal'};
      if(actors(simTime).some(o=>rectDistance(o.position,r)<o.radius+.1))return {ok:false,reason:'A patrol occupies that position'};
      if(kind==='moving'){
        const amplitude=amplitudeAt(x,y,axis);
        if(amplitude<.3)return {ok:false,reason:'Place moving objects farther from the walls along their movement direction'};
        moving.push({id:`M${nextObject++}`,x,y,amplitude,axis});actorTimes.push(0);actorRates.push(1);
      }else placed.push(r);
      changedScene();return {ok:true};
    },
    setObjectMotion(selection,axis){
      if(!['x','y'].includes(axis))return {ok:false,reason:'Choose X or Y movement'};
      const i=moving.findIndex(o=>selection?.kind==='moving'&&o.id===selection.id);
      const boxIndex=selection?.kind==='static'&&Array.isArray(selection.box)?placed.findIndex(r=>r.every((v,k)=>v===selection.box[k])):-1;
      if(i<0&&boxIndex<0)return {ok:false,reason:'Select an object first'};
      const p=i>=0?actors(simTime)[fixedActorCount+i].position:[(placed[boxIndex][0]+placed[boxIndex][2])/2,(placed[boxIndex][1]+placed[boxIndex][3])/2];
      const amplitude=amplitudeAt(p[0],p[1],axis);
      if(amplitude<.3)return {ok:false,reason:'Not enough room to move along that axis'};
      if(boxIndex>=0){const s=store.getState();if(Math.hypot(p[0]-position[0],p[1]-position[1])<.55+bodyRadius+s.profile.inflation_hgp+s.profile.dyn_base_inflation_m+.02)return {ok:false,reason:'The drone is too close to start this object moving'};}
      const object={id:i>=0?moving[i].id:`M${nextObject++}`,x:p[0],y:p[1],axis,amplitude};
      if(i>=0){moving[i]=object;actorTimes[fixedActorCount+i]=0;actorRates[fixedActorCount+i]=1;}
      else{placed.splice(boxIndex,1);moving.push(object);actorTimes.push(0);actorRates.push(1);}
      changedScene();return {ok:true,id:object.id};
    },
    removeObstacle(point){if(!Array.isArray(point)||!point.slice(0,2).every(Number.isFinite))return {ok:false,reason:'Choose an obstacle'};const index=placed.findIndex(r=>rectDistance(point,r)<.35);if(index<0){const i=actors(simTime).slice(fixedActorCount).findIndex(o=>Math.hypot(point[0]-o.position[0],point[1]-o.position[1])<o.radius+.35);if(i<0)return {ok:false,reason:'Click an added object to remove it'};moving.splice(i,1);actorTimes.splice(fixedActorCount+i,1);actorRates.splice(fixedActorCount+i,1);}else placed.splice(index,1);changedScene();return {ok:true};},
    clearObstacles(){placed=[];moving=[];actorTimes.length=fixedActorCount;actorRates.length=fixedActorCount;changedScene();return {ok:true};},
    snapshot(){return {handoffRemainingMs:Math.max(0,resumeAfter-(last??0)),room,bounds:[...bounds],fixtures:room?ROOM_FIXTURES.map(f=>({...f,rect:[...f.rect]})):[],started,bodyRadius,predictions:actors(simTime).map((o,i)=>Array.from({length:7},(_,k)=>actors(simTime+k*.3)[i].position)),objectCount:placed.length+moving.length,obstacles:obstacles.map(r=>[...r]),placed:placed.map(r=>[...r]),yaw,position:[...position],path:[position,...path.slice(1)],goal:goal()?[...goal()]:null,movers:actors(simTime),status:room&&!started?'Ready · click Start':status,plans,completed,clearance:clearance(position,simTime,true,actors,obstacles)-footprintPadding,simTime};},
    setGoal(point){const s=store.getState();if(!Array.isArray(point)||point.length!==3||!point.every(Number.isFinite)||!free(point,s.profile.inflation_hgp+footprintPadding,simTime,false,0,actors,obstacles))return {ok:false,reason:'Goal is inside an obstacle or its clearance margin'};if(!planPath(position,point,s.profile.inflation_hgp+footprintPadding,simTime,s.profile.dyn_base_inflation_m,actors,obstacles).length)return {ok:false,reason:'No route at this clearance. Try another goal or reset.'};target=[...point];targetPinned=true;activeCommand=null;paused=false;path=[];lastPlan=-Infinity;return {ok:true};},
    move(command){if(!command||command.type!=='move'||!Array.isArray(command.target)||command.target.length!==3||!command.target.every(Number.isFinite))return false;target=[...command.target];targetPinned=false;activeCommand=command.id;paused=false;path=[];lastPlan=-Infinity;return true;},
    clearManual({phase,direction,now=last??0}={}){if(!room&&!targetPinned)target=null;activeCommand=null;paused=false;path=[];segment=null;lastPlan=-Infinity;if(phase==='start'){resumeAfter=0;wasTurning=false;}if(phase==='release'&&direction?.startsWith('turn-')){resumeAfter=now+500;wasTurning=false;}return true;},
    stop(){paused=true;target=null;targetPinned=false;activeCommand=null;return true;}
  });
}
