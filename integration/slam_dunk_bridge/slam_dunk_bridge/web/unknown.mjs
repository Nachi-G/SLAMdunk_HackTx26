import {rayBox} from './pov.mjs';
import {rectDistance,sweptDistance} from './planner.mjs';

// Simulator truth stays inside this module. Public snapshots contain sensor
// evidence only; navigation searches the observed grid, never these rectangles.
const BOUNDS=[-12,-9,12,9],RES=.5,NX=48,NY=36,RANGE=4.5;
const SHELL=[[-12,-9,12,-8.75],[11.75,-9,12,9],[-12,8.75,12,9],[-12,-9,-11.75,9]];
const WORLDS={
  A:{name:'Unknown A',challenge:'Staggered passages and occluded rooms',spawn:[-9,-6,1.8],
    walls:[[-4.25,-9,-4,1.5],[-4.25,4.5,-4,9],[3.75,-9,4,-4.5],[3.75,-1.5,4,9],[-12,1.5,-8,1.75],[-1,4,1.5,5.5],[7,2,9,4]],actors:[]},
  B:{name:'Unknown B',challenge:'Warehouse aisles, blind corners and dead ends',spawn:[-9,-6,1.8],
    walls:[[-5,-5,-3.5,5.5],[0,-9,1.5,1],[5,-4,6.5,6],[-10,3,-8,6],[8,-7,10,-5]],actors:[]},
  C:{name:'Unknown C',challenge:'Crossing obstacles and changing routes',spawn:[-9,-6,1.8],
    walls:[[-5,-4,-3,0],[2,2,4,6],[-9,3,-7,5],[7,-6,9,-3]],
    actors:[{id:'D1',x:0,y:-3,axis:'x',amplitude:2,phase:0},{id:'D2',x:6,y:2,axis:'y',amplitude:3,phase:1}]}
};
export const UNKNOWN_ENVIRONMENTS=Object.freeze(Object.entries(WORLDS).map(([id,w])=>Object.freeze({id,name:w.name,challenge:w.challenge})));
const stamp=t=>({sec:Math.floor(t/1000),nanosec:Math.floor(t%1000*1e6)});
const index=(x,y)=>y*NX+x;
const inside=(x,y)=>x>=0&&x<NX&&y>=0&&y<NY;
const cell=p=>[Math.floor((p[0]-BOUNDS[0])/RES),Math.floor((p[1]-BOUNDS[1])/RES)];
const center=k=>[BOUNDS[0]+(k%NX+.5)*RES,BOUNDS[1]+(Math.floor(k/NX)+.5)*RES,1.8];
const neighbors=k=>{const x=k%NX,y=Math.floor(k/NX);return [[x-1,y],[x+1,y],[x,y-1],[x,y+1]].filter(([a,b])=>inside(a,b)).map(([a,b])=>index(a,b));};

export function createUnknownDemo(store,id='A'){
  const world=WORLDS[id];if(!world)throw new TypeError('Choose unknown environment A, B or C');
  const truth=[...SHELL,...world.walls],grid=new Uint8Array(NX*NY); // 0 unknown, 1 free, 2 occupied
  let position=[...world.spawn],yaw=0,started=false,last=null,simTime=0,lastScan=-Infinity,lastPlan=-Infinity,lastMap=-Infinity,revision=0;
  let route=[],target=null,manualTarget=false,plans=0,completed=0,state='ready',status='Unknown environment · click Begin exploration';
  let visibleActors=[],hits=[],freeCells=[],observedPoints=[],frontiers=0,observed=0,scanSequence=0,emptyScans=0,bestDistance=Infinity,progressAt=0;
  const failed=new Map(),visits=new Uint16Array(grid.length);
  const actorClocks=world.actors.map(()=>0);
  const actorsAt=(offset=0)=>world.actors.map((o,i)=>({id:o.id,axis:o.axis,radius:.5,position:[o.x+(o.axis==='x'?Math.sin((actorClocks[i]+offset)*.4+o.phase)*o.amplitude:0),o.y+(o.axis==='y'?Math.sin((actorClocks[i]+offset)*.4+o.phase)*o.amplitude:0),1.1]}));
  let motionBefore=actorsAt();
  const radius=()=>.24+store.getState().profile.inflation_hgp;
  function knownSafe(p){
    if(!Array.isArray(p)||p.length!==3||!p.every(Number.isFinite)||p[2]<.5||p[2]>2.5)return false;
    const r=radius(),[cx,cy]=cell(p),n=Math.ceil(r/RES)+1;
    if(visibleActors.some(o=>Math.hypot(p[0]-o.position[0],p[1]-o.position[1])<r+o.radius+store.getState().profile.dyn_base_inflation_m+.15))return false;
    for(let dy=-n;dy<=n;dy++)for(let dx=-n;dx<=n;dx++){
      const x=cx+dx,y=cy+dy,box=[BOUNDS[0]+x*RES,BOUNDS[1]+y*RES,BOUNDS[0]+(x+1)*RES,BOUNDS[1]+(y+1)*RES];
      if(rectDistance(p,box)<r-1e-6&&(!inside(x,y)||grid[index(x,y)]!==1))return false;
    }
    return true;
  }
  function knownSegment(a,b){
    const n=Math.max(1,Math.ceil(Math.hypot(b[0]-a[0],b[1]-a[1])/.15));
    for(let i=0;i<=n;i++)if(!knownSafe(a.map((v,k)=>v+(b[k]-v)*i/n)))return false;
    return true;
  }
  function scan(){
    const free=new Set(),occupied=new Set(),seenActors=new Set(),scanHits=[],currentActors=actorsAt();
    for(let i=0;i<240;i++){
      const angle=i*Math.PI*2/240,dir=[Math.cos(angle),Math.sin(angle)];let distance=RANGE,kind=null;
      for(const box of truth){const d=rayBox(position,dir,box);if(d!==null&&d>=0&&d<distance){distance=d;kind='static';}}
      for(const actor of currentActors){const x=position[0]-actor.position[0],y=position[1]-actor.position[1],b=x*dir[0]+y*dir[1],c=x*x+y*y-actor.radius**2,disc=b*b-c;
        if(disc<0)continue;const d=-b-Math.sqrt(disc);if(d>=0&&d<distance){distance=d;kind=actor.id;}}
      // Free evidence stops before the first hit. Nothing behind it is revealed.
      for(let d=0;d<distance-.08;d+=.12){const [x,y]=cell([position[0]+dir[0]*d,position[1]+dir[1]*d]);if(inside(x,y))free.add(index(x,y));}
      if(kind){const hit=[position[0]+dir[0]*(distance+.015),position[1]+dir[1]*(distance+.015),1.2];scanHits.push(hit);
        if(kind==='static'){const [x,y]=cell(hit);if(inside(x,y))occupied.add(index(x,y));}else seenActors.add(kind);}
    }
    for(const k of free)grid[k]=1;
    for(const k of occupied)grid[k]=2; // Hits win over free evidence in one scan.
    visibleActors=currentActors.filter(o=>seenActors.has(o.id));hits=scanHits;
    freeCells=[];observedPoints=[];observed=0;frontiers=0;
    for(let k=0;k<grid.length;k++){
      if(!grid[k])continue;observed++;
      if(grid[k]===1){freeCells.push(center(k));if(neighbors(k).some(n=>grid[n]===0))frontiers++;}
      else {const p=center(k);for(let z=0;z<=2.4;z+=.4)observedPoints.push([p[0],p[1],z]);}
    }
    scanSequence++;lastScan=simTime;
  }
  function reachable(){
    const [x,y]=cell(position),start=index(x,y),parent=new Map([[start,null]]),distance=new Map([[start,0]]),queue=[start];
    if(!knownSafe(position))return {parent,distance,queue:[]};
    for(let at=0;at<queue.length;at++)for(const next of neighbors(queue[at])){
      if(parent.has(next)||!knownSafe(center(next))||!knownSegment(at===0?position:center(queue[at]),center(next)))continue;
      parent.set(next,queue[at]);distance.set(next,distance.get(queue[at])+RES);queue.push(next);
    }
    return {parent,distance,queue};
  }
  function routeTo(k,search){
    const result=[];for(let cursor=k;search.parent.get(cursor)!==null;cursor=search.parent.get(cursor))result.push(center(cursor));
    result.reverse();if(result.length&&Math.hypot(result[0][0]-position[0],result[0][1]-position[1])<.1)result.shift();return result;
  }
  function informationGain(p){
    const unknown=new Set();
    // Visibility through the observed grid only. A known wall occludes gain.
    for(let i=0;i<48;i++){const a=i*Math.PI*2/48;for(let d=.4;d<RANGE;d+=RES){const [x,y]=cell([p[0]+Math.cos(a)*d,p[1]+Math.sin(a)*d]);if(!inside(x,y))break;const k=index(x,y);if(grid[k]===2)break;if(!grid[k])unknown.add(k);}}
    return unknown.size;
  }
  function plan(){
    lastPlan=simTime;plans++;const search=reachable();
    if(manualTarget&&target){
      const [x,y]=cell(target),k=index(x,y);
      if(search.parent.has(k)){route=routeTo(k,search);if(knownSegment(route.at(-1)??position,target))route.push([...target]);state='exploring';status='Navigating through observed free space';return;}
      manualTarget=false;target=null;
    }
    let best=null,bestScore=-Infinity;
    for(const k of search.queue){
      if(search.distance.get(k)<.75||failed.get(k)>simTime)continue;
      const gain=informationGain(center(k));if(gain<2)continue;
      const score=gain/(1+search.distance.get(k)*.12)-visits[k]*12;
      if(score>bestScore){best=k;bestScore=score;}
    }
    if(best!==null){target=center(best);route=routeTo(best,search);visits[best]++;bestDistance=Infinity;progressAt=simTime;emptyScans=0;state='exploring';status='Exploring a frontier · mapping from range scans';}
    else {route=[];target=null;emptyScans++;state=frontiers?'blocked':emptyScans>=3?'complete':'waiting';status=frontiers?'Frontiers remain · no reachable viewpoint at this clearance':state==='complete'?'Reachable observed space surveyed · sealed areas may remain unknown':'Confirming reachable frontier exhaustion';}
  }
  function safeMotion(a,b,dt){
    if(!knownSegment(a,b))return false;
    const r=radius(),before=motionBefore,after=actorsAt();
    // Truth collision checking belongs only to simulated physics. This guard
    // rejects motion; it never exposes walls or chooses navigation destinations.
    for(let i=0;i<=4;i++){const p=a.map((v,k)=>v+(b[k]-v)*i/4);if(truth.some(box=>rectDistance(p,box)<r))return false;}
    return after.every((o,i)=>sweptDistance(a,b,before[i].position,o.position)>=r+o.radius+store.getState().profile.dyn_base_inflation_m);
  }
  function step(now){
    if(!Number.isFinite(now)||now<=0||last!==null&&now<=last||store.getState().source!=='demo')return false;
    const dt=last===null?0:Math.min(.25,(now-last)/1000);last=now;
    const s=store.getState(),paused=s.explorationMode==='paused';let velocity=[0,0,0];
    if(started&&!paused){
      motionBefore=actorsAt();const proposed=actorsAt(dt);
      // Match the existing synthetic demo: actors yield at the vehicle boundary.
      for(let i=0;i<actorClocks.length;i++)if(sweptDistance(position,position,motionBefore[i].position,proposed[i].position)>=radius()+.5+s.profile.dyn_base_inflation_m+.04&&!truth.some(box=>rectDistance(proposed[i].position,box)<.55))actorClocks[i]+=dt;
      simTime+=dt;if(simTime-lastScan>=.3)scan();
      if(s.steering){
        route=[];target=null;manualTarget=false;lastPlan=-Infinity;
        const next=position.map((v,i)=>v+s.steering.worldDirection[i]*s.speedLimit*.65*dt);
        if(safeMotion(position,next,dt)){velocity=next.map((v,i)=>(v-position[i])/Math.max(dt,.001));position=next;status='Manual guidance · observed free space only';}else status='Manual guidance blocked · unknown or occupied space';
      }else{
        if(route.some(p=>!knownSafe(p))){route=[];target=null;manualTarget=false;}
        if(!route.length&&simTime-lastPlan>=1)plan();
        if(route.length){
          const nextPoint=route[0],distance=Math.hypot(nextPoint[0]-position[0],nextPoint[1]-position[1]),travel=Math.min(distance,s.speedLimit*.65*dt);
          const next=distance<1e-9?[...nextPoint]:position.map((v,i)=>v+(nextPoint[i]-v)*travel/distance);
          if(safeMotion(position,next,dt)){velocity=next.map((v,i)=>(v-position[i])/Math.max(dt,.001));position=next;if(distance<=travel+.01)route.shift();}
          else status='Yielding · local obstacle or insufficient observed clearance';
          const remaining=target?Math.hypot(target[0]-position[0],target[1]-position[1]):0;
          if(remaining<bestDistance-.1){bestDistance=remaining;progressAt=simTime;}
          if(simTime-progressAt>8){if(target){const [x,y]=cell(target);failed.set(index(x,y),simTime+15);}route=[];target=null;manualTarget=false;lastPlan=-Infinity;}
          if(!route.length&&target){completed++;target=null;manualTarget=false;lastPlan=-Infinity;}
        }
      }
    }
    if(Math.hypot(velocity[0],velocity[1])>.001)yaw=Math.atan2(velocity[1],velocity[0]);
    store.ingest({type:'pose',protocol_version:1,frame_id:'map',stamp:stamp(now),position,orientation:[0,0,Math.sin(yaw/2),Math.cos(yaw/2)],velocity},now);
    if(now-lastMap>=300){lastMap=now;store.ingest({type:'map',protocol_version:1,frame_id:'map',stamp:stamp(now),operation:'replace',revision:++revision,points:observedPoints},now);}
    return true;
  }
  return Object.freeze({step,
    start(){started=true;state='exploring';status='Scanning surroundings · searching for frontiers';lastPlan=-Infinity;return {ok:true};},
    clearManual(){route=[];target=null;manualTarget=false;lastPlan=-Infinity;return true;},
    stop(){return store.pauseExploration();},
    setGoal(p){
      if(!started)return {ok:false,reason:'Begin exploration first'};
      if(!knownSafe(p))return {ok:false,reason:'Choose observed free space; unknown areas must be explored first'};
      const [x,y]=cell(p),search=reachable(),k=index(x,y);if(!search.parent.has(k))return {ok:false,reason:'No route through observed free space'};
      target=[...p];manualTarget=true;route=routeTo(k,search);route.push([...p]);bestDistance=Infinity;progressAt=simTime;state='exploring';status='Navigating to observed target';return {ok:true};
    },
    snapshot(){return {hidden:true,environment:id,name:world.name,challenge:world.challenge,room:false,bounds:[...BOUNDS],started,bodyRadius:.24,position:[...position],yaw,simTime,
      path:route.length?[[...position],...route.map(p=>[...p])]:[],goal:target?[...target]:null,movers:visibleActors.map(o=>({...o,position:[...o.position]})),
      predictions:[],placed:[],obstacles:[],fixtures:[],objectCount:0,status:store.getState().explorationMode==='paused'?'Exploration paused':status,state,
      plans,completed,clearance:null,freeCells:freeCells.map(p=>[...p]),cellSize:RES,observedCells:observed,totalCells:grid.length,observedArea:observed*RES*RES,
      coverage:observed/grid.length,frontiers,scanSequence,sensorRange:RANGE,hits:hits.map(p=>[...p])};}
  });
}
