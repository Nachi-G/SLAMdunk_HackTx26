import {rayBox} from './pov.mjs';
export const MISSION_KEY='slamdunk.flight-room.missions.v1';
const MAX_FRAMES=3600,MAX_EVENTS=600,MAX_POINTS=8000,MAX_BYTES=1800000;
const copy=value=>JSON.parse(JSON.stringify(value));
const finite=n=>typeof n==='number'&&Number.isFinite(n);
export function readMissions(storage){
  try{
    const raw=storage?.getItem(MISSION_KEY);if(!raw||raw.length>MAX_BYTES+1000)return null;
    const point=p=>Array.isArray(p)&&p.length===3&&p.every(finite),rect=r=>Array.isArray(r)&&r.length===4&&r.every(finite),list=(a,max,check)=>Array.isArray(a)&&a.length<=max&&a.every(check);
    const frame=f=>(f.bounds===undefined||rect(f.bounds))&&(f.fixtures===undefined||list(f.fixtures,20,o=>rect(o.rect)&&finite(o.height)&&typeof o.color==='string'&&/^#[a-fA-F0-9]{6}$/.test(o.color)&&typeof o.label==='string'))&&finite(f.t)&&point(f.position)&&finite(f.yaw)&&(f.goal===null||point(f.goal))&&list(f.planned,1000,point)&&list(f.placed,24,rect)&&list(f.obstacles,48,rect)&&list(f.movers,24,o=>typeof o.id==='string'&&point(o.position)&&finite(o.radius)&&o.radius>0&&o.radius<5)&&finite(f.bodyRadius)&&finite(f.speed)&&finite(f.clearance)&&finite(f.distance)&&typeof f.status==='string'&&f.status.length<256;
    const data=JSON.parse(raw),validRun=run=>run&&typeof run.id==='string'&&typeof run.name==='string'&&finite(run.created)&&Array.isArray(run.frames)&&run.frames.length<=MAX_FRAMES&&run.frames.every(frame)&&Array.isArray(run.events)&&run.events.length<=MAX_EVENTS&&run.events.every(e=>finite(e.t)&&typeof e.label==='string'&&e.label.length<256)&&Array.isArray(run.observations)&&run.observations.length<=MAX_POINTS&&run.observations.every(p=>Array.isArray(p)&&p.length===4&&p.every(finite));
    return data.version===1&&validRun(data.current)&&Array.isArray(data.archives)&&data.archives.length<=3&&data.archives.every(validRun)?data:null;
  }catch{return null;}
}
/** A synthetic 360° planar scan: nearest occluding hit, 8 m maximum range. */
export function scanRoom(sim){
  const origin=sim.position,hits=[];
  for(let i=0;i<144;i++){
    const angle=i/144*Math.PI*2,dir=[Math.cos(angle),Math.sin(angle)];let distance=8,hit=false;
    for(const box of sim.obstacles){const d=rayBox(origin,dir,box);if(d!==null&&d<distance){distance=d;hit=true;}}
    for(const actor of sim.movers){
      const x=origin[0]-actor.position[0],y=origin[1]-actor.position[1],b=x*dir[0]+y*dir[1],c=x*x+y*y-actor.radius**2,disc=b*b-c;
      if(disc<0)continue;const d=-b-Math.sqrt(disc);if(d>=0&&d<distance){distance=d;hit=true;}
    }
    if(hit)hits.push([Number((origin[0]+dir[0]*distance).toFixed(2)),Number((origin[1]+dir[1]*distance).toFixed(2)),.06]);
  }
  return hits;
}
export function createMissionLog({storage,clock=()=>Date.now()}={}){
  if(storage===undefined)try{storage=globalThis.localStorage;}catch{storage=null;}
  const previous=readMissions(storage);let archives=previous?[previous.current,...previous.archives].filter(r=>r.frames.length).slice(0,3):[];
  let current,seen,lastSignature=null,lastMotion=null,lastFrame=-Infinity,lastWrite=-Infinity,dirty=false,available=!!storage;
  function newRun(){const created=clock();return {id:`${created}-${Math.random().toString(36).slice(2)}`,name:`Flight ${new Date(created).toLocaleTimeString([], {hour:'2-digit',minute:'2-digit'})}`,created,source:'synthetic_room',frames:[],events:[],observations:[],truncated:false};}
  function reset(){if(current?.frames.length)archives=[copy(current),...archives].slice(0,3);current=newRun();seen=new Set();lastSignature=null;lastMotion=null;lastFrame=-Infinity;dirty=true;flush();}
  function flush(){
    if(!dirty)return available;
    const pack=()=>JSON.stringify({version:1,current,archives});let raw=pack();
    while(raw.length>MAX_BYTES&&archives.length){archives.pop();raw=pack();}
    while(raw.length>MAX_BYTES&&current.frames.length>1){current.frames.splice(0,Math.max(1,Math.ceil(current.frames.length*.15)));current.truncated=true;const first=current.frames[0].t;current.events=current.events.filter(e=>e.t>=first);raw=pack();}
    try{if(!storage)throw new Error('Storage unavailable');storage.setItem(MISSION_KEY,raw);available=true;dirty=false;}catch{available=false;}
    lastWrite=clock();return available;
  }
  function capture(sim,state,distance=0){
    if(!sim?.room||!state.pose)return false;
    const now=clock(),signature={started:sim.started,goal:JSON.stringify(sim.goal),objects:JSON.stringify([sim.placed,sim.movers.map(o=>[o.id,o.axis])]),mode:state.explorationMode,plans:sim.plans,completed:sim.completed,status:sim.status};
    const events=[];
    if(lastSignature){
      if(!lastSignature.started&&signature.started)events.push(['start','Simulation started']);
      if(signature.goal!==lastSignature.goal)events.push(['goal','Destination changed']);
      if(signature.objects!==lastSignature.objects)events.push(['objects',`Objects updated · ${sim.placed.length} static · ${sim.movers.length} moving`]);
      if(signature.completed>lastSignature.completed)events.push(['arrival','Destination reached']);
      if(signature.plans>lastSignature.plans)events.push(['plan',`Route planned · #${sim.plans}`]);
      if(signature.mode!==lastSignature.mode)events.push(['mode',({steering:'Operator push started',paused:'Simulation paused',autonomous:lastSignature.mode==='steering'?'Push released · navigation resumed':'Navigation resumed'})[signature.mode]??signature.mode]);
      if(signature.status!==lastSignature.status&&/Yielding|blocked|No feasible|Avoidance/i.test(signature.status))events.push(['avoidance',signature.status]);
    }else events.push(['setup','Room recording started']);
    lastSignature=signature;
    const motion=JSON.stringify([sim.position,sim.yaw,sim.movers.map(o=>o.position)]);
    if(motion===lastMotion&&!events.length)return false;
    if(now-lastFrame<500&&!events.length)return false;
    const t=Math.max(now,(current.frames.at(-1)?.t??now)+.001);lastFrame=now;lastMotion=motion;
    const frame={t,bounds:copy(sim.bounds),fixtures:copy(sim.fixtures),position:[...sim.position],yaw:sim.yaw,goal:sim.goal?[...sim.goal]:null,planned:copy(sim.path),placed:copy(sim.placed),movers:copy(sim.movers),obstacles:copy(sim.obstacles),bodyRadius:sim.bodyRadius,status:state.explorationMode==='paused'?'Simulation paused':sim.status,started:sim.started,mode:state.explorationMode,speed:state.speed??0,clearance:sim.clearance,distance,plans:sim.plans,completed:sim.completed};
    current.frames.push(frame);
    if(current.frames.length>MAX_FRAMES){current.frames.shift();current.truncated=true;}
    for(const [kind,label]of events)current.events.push({t,kind,label});
    if(current.events.length>MAX_EVENTS)current.events.splice(0,current.events.length-MAX_EVENTS);
    for(const p of scanRoom(sim)){const key=`${Math.floor(p[0]/.2)},${Math.floor(p[1]/.2)}`;if(seen.has(key))continue;seen.add(key);if(current.observations.length<MAX_POINTS)current.observations.push([...p,t]);}
    dirty=true;if(now-lastWrite>=1500)flush();return true;
  }
  reset();
  return Object.freeze({capture,flush,reset,getSnapshot:()=>copy({version:1,current,archives}),isPersistent:()=>available});
}
