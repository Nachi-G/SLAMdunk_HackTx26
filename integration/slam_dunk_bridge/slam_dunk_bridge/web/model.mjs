/** Mission state and protocol validation. All outward snapshots are immutable.
 * Occupied-history voxels are observations, never a current free-space map.
 */
const LIMITS = Object.freeze({points:20000, combined:12000, trail:500, history:240, historyPointBudget:240000, replayPoints:700, staleMs:3000});
const finite = n => typeof n === 'number' && Number.isFinite(n);
const vector = (v,n) => Array.isArray(v) && v.length===n && v.every(finite);
const frozenVector = v => Object.freeze([...v]);
const empty = () => Object.freeze([]);
function explorationStatus(value) {
  if(!value||value.mode!=='frontier'||typeof value.state!=='string'||value.state.length>64||typeof value.reason!=='string'||value.reason.length>1024||typeof value.completion_scope!=='string'||value.completion_scope.length>256)return null;
  const counts=['map_revision','scan_sequence','frontier_cells','reachable_cells','unknown_voxels','free_voxels','occupied_voxels','failed_viewpoints'];
  if(!counts.every(k=>Number.isSafeInteger(value[k])&&value[k]>=0)||!(value.goal===null||vector(value.goal,3)&&value.goal.every(v=>Math.abs(v)<=1e6)))return null;
  return Object.freeze({mode:'frontier',state:value.state,reason:value.reason,completion_scope:value.completion_scope,...Object.fromEntries(counts.map(k=>[k,value[k]])),goal:value.goal===null?null:frozenVector(value.goal)});
}
function timestamp(stamp) {
  if(!stamp||!Number.isSafeInteger(stamp.sec)||stamp.sec<0||!Number.isInteger(stamp.nanosec)||stamp.nanosec<0||stamp.nanosec>=1e9)return NaN;
  const value=stamp.sec*1000+stamp.nanosec/1e6;
  return finite(value)&&value<=8.64e15&&Number.isSafeInteger(Math.floor(value))?value:NaN;
}
function profile(value,source) {
  return Object.freeze({value,inflation_hgp:Number((.45+.45*value/100).toFixed(4)),dyn_base_inflation_m:Number((.1+.2*value/100).toFixed(4)),state:source==='demo'?'demo':'proposal',applied:null});
}
function initial(source) {
  return {source,connection:source==='demo'?'demo':'connecting',view:'live',frameId:null,mapPoints:empty(),combinedPoints:empty(),pose:null,trail:empty(),speed:null,battery:null,flightSeconds:null,profile:profile(0,source),speedMode:'precision',speedLimit:1,command:null,statusMessage:source==='demo'?'Synthetic mission · no vehicle connected':'Waiting for bridge data',stale:Object.freeze({map:true,pose:true}),history:empty(),selectedTime:null,playing:false,rejected:0,mode:source==='demo'?'synthetic_demo':'map_snapshot',controlAvailable:source==='demo',controlConnection:source==='demo'?'demo':'unavailable',controlCapabilities:Object.freeze({steering:source==='demo',pause:source==='demo'}),controlEpoch:0,controlReady:source==='demo',explorationMode:source==='demo'?'autonomous':'waiting',steering:null,exploration:null};
}

export class MissionStore {
  constructor() { this.listeners=new Set(); this.reset('demo'); }
  getState() { return this.state; }
  subscribe(fn) {
    if(typeof fn!=='function') throw new TypeError('Subscriber must be a function');
    this.listeners.add(fn); fn(this.state);
    return ()=>this.listeners.delete(fn);
  }
  _emit(patch={}) {
    this.state=Object.freeze({...this.state,...patch});
    for(const fn of this.listeners) { try { fn(this.state); } catch { /* One panel cannot stop telemetry. */ } }
  }
  reset(source='demo') {
    if(!['demo','live'].includes(source)) return false;
    this.lastStamp={map:-Infinity,pose:-Infinity}; this.received={map:null,pose:null};
    this.sourceAge={map:null,pose:null}; this.ageAt={map:null,pose:null}; this.voxels=new Map(); this.replayCache=new WeakMap(); this.lastHistory=-Infinity; this.sequence=0; this.lastTick=null;
    this.state=Object.freeze(initial(source)); this._emit(); return true;
  }
  setConnection(connection) {
    if(!['demo','connecting','connected','disconnected'].includes(connection)) return false;
    if(this.state.source==='live' && connection==='demo') return false;
    this._emit({connection,...(connection==='disconnected'?{stale:Object.freeze({map:true,pose:true}),playing:false}:{})}); return true;
  }
  _reject(message) { this._emit({rejected:this.state.rejected+1,statusMessage:message}); return false; }
  ingest(message,now=Date.now()) {
    if(!finite(now)||!message||typeof message!=='object'||message.protocol_version!==1) return this._reject('Rejected invalid protocol message');
    if(message.type==='status') {
      if(!['map_snapshot','synthetic_demo'].includes(message.mode)) return this._reject('Rejected invalid source mode');
      const ages=message.age_seconds;
      if(!ages||!['map','pose'].every(k=>ages[k]===null||(finite(ages[k])&&ages[k]>=0))) return this._reject('Rejected invalid source freshness');
      if(message.frame_id && this.state.frameId && message.frame_id!==this.state.frameId) return this._reject('Rejected frame mismatch');
      this.sourceAge={...ages}; this.ageAt={map:now,pose:now};
      this._emit({mode:message.mode}); this.tick(now); return true;
    }
    if(!['map','pose'].includes(message.type)) return this._reject('Unsupported message type');
    const kind=message.type,t=timestamp(message.stamp);
    if(!finite(t)||t<=0||t<=this.lastStamp[kind]) return this._reject('Rejected duplicate or out-of-order timestamp');
    if(typeof message.frame_id!=='string'||!message.frame_id.trim()||message.frame_id.length>128||(this.state.frameId&&message.frame_id!==this.state.frameId)) return this._reject('Rejected frame mismatch or missing frame');
    let patch={frameId:message.frame_id};
    if(kind==='map') {
      if(message.operation!=='replace'||!Number.isSafeInteger(message.revision)||message.revision<0||!Array.isArray(message.points)||message.points.length>LIMITS.points||!message.points.every(p=>vector(p,3)&&p.every(v=>Math.abs(v)<=1e6))) return this._reject('Rejected invalid map snapshot');
      const points=Object.freeze(message.points.map(frozenVector));
      for(const p of points) {
        const key=p.map(v=>Math.floor(v/.18)).join(',');
        if(this.voxels.has(key)) this.voxels.delete(key);
        this.voxels.set(key,p);
      }
      while(this.voxels.size>LIMITS.combined) this.voxels.delete(this.voxels.keys().next().value);
      patch={...patch,mapPoints:points,combinedPoints:Object.freeze([...this.voxels.values()])};
    } else {
      if(!vector(message.position,3)||message.position.some(v=>Math.abs(v)>1e6)||!vector(message.orientation,4)) return this._reject('Rejected invalid drone pose');
      const norm=Math.hypot(...message.orientation);
      if(norm<1e-9||!finite(norm)) return this._reject('Rejected invalid quaternion');
      if(message.velocity!==undefined&&!vector(message.velocity,3)) return this._reject('Rejected invalid velocity');
      if(message.velocity!==undefined&&(!finite(Math.hypot(...message.velocity))||Math.hypot(...message.velocity)>10000)) return this._reject('Rejected invalid velocity magnitude');
      if(message.battery!==undefined&&message.battery!==null&&(!finite(message.battery)||message.battery<0||message.battery>100)) return this._reject('Rejected invalid battery');
      if(message.flight_seconds!==undefined&&message.flight_seconds!==null&&(!finite(message.flight_seconds)||message.flight_seconds<0)) return this._reject('Rejected invalid flight duration');
      const q=message.orientation.map(v=>v/norm),[x,y,z,w]=q;
      const pose=Object.freeze({position:frozenVector(message.position),orientation:frozenVector(q),yaw:Math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))});
      patch={...patch,pose,trail:Object.freeze([...this.state.trail,pose.position].slice(-LIMITS.trail)),speed:message.velocity?Math.hypot(...message.velocity):null,battery:message.battery??null,flightSeconds:message.flight_seconds??null};
    }
    this.lastStamp[kind]=t; this.received[kind]=now;
    // A fresh accepted source sample supersedes an older status report for this type.
    this.sourceAge[kind]=0;
    this.ageAt[kind]=now;
    this._emit({...patch,statusMessage:this.state.source==='demo'?'Synthetic mission · no vehicle connected':'Receiving bridge telemetry'});
    this._record(Math.max(this.lastStamp.map,this.lastStamp.pose)); this.tick(now); return true;
  }
  _record(t) {
    // Aggregate at most two frames/sec; each frame references immutable map arrays.
    if(t<this.lastHistory || (t!==this.lastHistory&&t-this.lastHistory<500 && this.state.history.length)) return;
    this.lastHistory=t;
    const s=this.state;
    const sample=points=>{
      if(points.length<=LIMITS.replayPoints)return points;
      if(!this.replayCache.has(points))this.replayCache.set(points,Object.freeze(Array.from({length:LIMITS.replayPoints},(_,i)=>points[Math.floor(i*points.length/LIMITS.replayPoints)])));
      return this.replayCache.get(points);
    };
    const frame=Object.freeze({timestamp:t,points:sample(s.mapPoints),combinedPoints:sample(s.combinedPoints),pose:s.pose,trail:s.trail,speed:s.speed,battery:s.battery,flightSeconds:s.flightSeconds,sampled:s.mapPoints.length>LIMITS.replayPoints||s.combinedPoints.length>LIMITS.replayPoints});
    const frames=[...s.history];if(frames.at(-1)?.timestamp===t)frames.pop();frames.push(frame);
    let budget=frames.reduce((sum,f)=>sum+f.points.length+f.combinedPoints.length,0);
    while(frames.length>1&&(frames.length>LIMITS.history||budget>LIMITS.historyPointBudget)){const old=frames.shift();budget-=old.points.length+old.combinedPoints.length;}
    const history=Object.freeze(frames);
    const selectedTime=s.view==='live'?t:Math.max(history[0].timestamp,Math.min(history.at(-1).timestamp,s.selectedTime??t));
    this._emit({history,selectedTime});
  }
  tick(now=Date.now()) {
    if(!finite(now)) return false;
    const stale={};
    for(const kind of ['map','pose']) {
      const local=this.received[kind]===null?Infinity:Math.max(0,now-this.received[kind]);
      const source=this.sourceAge[kind]===null?Infinity:this.sourceAge[kind]*1000+Math.max(0,now-(this.ageAt[kind]??now));
      stale[kind]=this.state.connection==='disconnected'||local>LIMITS.staleMs||source>LIMITS.staleMs;
    }
    const patch={stale:Object.freeze(stale)};
    if(this.state.source==='demo'&&this.state.steering&&now>=this.state.steering.expiresAt) {
      patch.steering=null;patch.explorationMode='autonomous';patch.controlEpoch=this.state.controlEpoch+1;
      patch.command=Object.freeze({type:'steer',phase:'released',message:'Steering lease expired · autonomous exploration resumed'});
    }
    if(this.state.playing&&this.state.view==='history'&&this.state.history.length&&this.lastTick!==null) {
      const max=this.state.history.at(-1).timestamp;
      patch.selectedTime=Math.min(max,(this.state.selectedTime??max)+Math.max(0,Math.min(1000,now-this.lastTick)));
      if(patch.selectedTime>=max) patch.playing=false;
    }
    this.lastTick=now; this._emit(patch); return true;
  }
  getDisplay() {
    const s=this.state;
    if(s.view==='history'&&s.history.length) {
      for(let i=s.history.length-1;i>=0;i--) if(s.history[i].timestamp<=s.selectedTime) return s.history[i];
      return s.history[0];
    }
    return Object.freeze({timestamp:this.lastStamp.pose>0?this.lastStamp.pose:null,points:s.mapPoints,combinedPoints:s.combinedPoints,pose:s.pose,trail:s.trail,speed:s.speed,battery:s.battery,flightSeconds:s.flightSeconds});
  }
  setControlConnection(connection) {
    if(!['connecting','connected','disconnected','unavailable'].includes(connection)||this.state.source!=='live')return false;
    const disconnected=connection!=='connected';
    this._emit({controlConnection:connection,...(connection==='connecting'?{controlEpoch:0}:{}),...(disconnected?{controlAvailable:false,controlReady:false,steering:null,exploration:null,explorationMode:'waiting',controlCapabilities:Object.freeze({steering:false,pause:false})}:{})});
    return true;
  }
  ingestControl(message) {
    const modes=['autonomous','steering','paused','waiting','fault'];
    if(this.state.source!=='live'||!message||message.protocol_version!==1||message.type!=='control_status'||!Number.isSafeInteger(message.epoch)||message.epoch<0||message.epoch<this.state.controlEpoch||!modes.includes(message.mode)||typeof message.ready!=='boolean'||!message.capabilities||typeof message.capabilities.steering!=='boolean'||typeof message.capabilities.pause!=='boolean'||message.ttl_ms!==500||!(message.lease_id===null||typeof message.lease_id==='string'&&message.lease_id.length>0&&message.lease_id.length<=128)||typeof message.reason!=='string'||message.reason.length>1024)return false;
    if(message.mode==='steering'&&!message.lease_id)return false;
    const steering=message.mode==='steering'?Object.freeze({leaseId:message.lease_id,epoch:message.epoch,direction:message.direction??null}):null;
    this._emit({controlConnection:'connected',controlAvailable:message.ready&&message.capabilities.steering,controlReady:message.ready,controlCapabilities:Object.freeze({...message.capabilities}),controlEpoch:message.epoch,explorationMode:message.mode,exploration:explorationStatus(message.exploration),steering,command:Object.freeze({type:'control',phase:message.mode,message:message.reason||`Exploration ${message.mode}`})});
    return true;
  }
  startSteering(direction,now=Date.now()) {
    const directions={forward:[1,0,0],backward:[-1,0,0],left:[0,1,0],right:[0,-1,0],up:[0,0,1],down:[0,0,-1]};
    const s=this.state;
    const reason=typeof direction!=='string'||!Object.hasOwn(directions,direction)?'Unknown direction':!finite(now)?'Invalid command time':s.source!=='demo'?'Live steering requires the command gateway':s.view!=='live'?'Return to Live to steer':s.explorationMode==='paused'?'Resume exploration before steering':s.steering?'A steering direction is already held':!s.pose||s.stale.pose||s.stale.map?'Fresh map and pose required':null;
    if(reason)return {ok:false,reason};
    const [dx,dy,dz]=directions[direction],c=Math.cos(s.pose.yaw),sn=Math.sin(s.pose.yaw),epoch=s.controlEpoch+1;
    const steering=Object.freeze({leaseId:`demo-${++this.sequence}`,direction,epoch,expiresAt:now+500,worldDirection:frozenVector([c*dx-sn*dy,sn*dx+c*dy,dz])});
    this._emit({steering,controlEpoch:epoch,explorationMode:'steering',command:Object.freeze({type:'steer',direction,phase:'steering',message:`Steering ${direction} · release to explore`})});
    return {ok:true,steering};
  }
  renewSteering(now=Date.now()) {
    if(!finite(now)||this.state.source!=='demo'||!this.state.steering)return {ok:false,reason:'No held steering lease'};
    if(now>=this.state.steering.expiresAt){this.tick(now);return {ok:false,reason:'Steering lease expired'};}
    this._emit({steering:Object.freeze({...this.state.steering,expiresAt:now+500})});return {ok:true};
  }
  releaseSteering(reason='released') {
    if(this.state.source!=='demo')return {ok:false,reason:'Live steering requires the command gateway'};
    if(!this.state.steering)return {ok:true};
    this._emit({steering:null,controlEpoch:this.state.controlEpoch+1,explorationMode:'autonomous',command:Object.freeze({type:'steer',phase:'released',message:'Autonomous exploration resumed'})});
    return {ok:true};
  }
  pauseExploration() {
    if(this.state.source!=='demo')return {ok:false,reason:'Physical pause backend unavailable'};
    this._emit({steering:null,controlEpoch:this.state.controlEpoch+1,explorationMode:'paused',command:Object.freeze({type:'pause',phase:'paused',message:'Synthetic exploration paused'})});return {ok:true};
  }
  resumeExploration() {
    if(this.state.source!=='demo')return {ok:false,reason:'Live resume requires the command gateway'};
    this._emit({steering:null,controlEpoch:this.state.controlEpoch+1,explorationMode:'autonomous',command:Object.freeze({type:'resume',phase:'autonomous',message:'Autonomous exploration resumed'})});return {ok:true};
  }
  command(direction,distance=1) {
    const directions={forward:[1,0,0],backward:[-1,0,0],left:[0,1,0],right:[0,-1,0],up:[0,0,1],down:[0,0,-1]};
    const s=this.state;
    let reason=typeof direction!=='string'||!Object.hasOwn(directions,direction)?'Unknown direction':!finite(distance)||distance<.1||distance>2?'Movement distance must be 0.1–2 m':s.view!=='live'?'Return to Live to move':s.source!=='demo'?'Bridge is read-only; command transport required':!s.pose||s.stale.pose?'Fresh drone pose required':null;
    if(reason) {this._emit({command:Object.freeze({id:++this.sequence,type:'move',phase:'rejected',message:reason})});return {ok:false,reason};}
    const [dx,dy,dz]=directions[direction],c=Math.cos(s.pose.yaw),sn=Math.sin(s.pose.yaw),p=s.pose.position;
    const target=frozenVector([p[0]+(c*dx-sn*dy)*distance,p[1]+(sn*dx+c*dy)*distance,Math.max(.5,Math.min(6,p[2]+dz*distance))]);
    const command=Object.freeze({id:++this.sequence,type:'move',direction,distance,phase:'accepted',message:'Synthetic movement accepted',target});
    this._emit({command});return {ok:true,command};
  }
  stop() {
    if(this.state.source!=='demo') {const reason='Bridge is read-only; Stop/Hold backend unavailable';this._emit({command:Object.freeze({id:++this.sequence,type:'stop',phase:'unavailable',message:reason})});return {ok:false,reason};}
    const command=Object.freeze({id:++this.sequence,type:'stop',phase:'holding',message:'Synthetic mission paused'});
    this._emit({command});return {ok:true,command};
  }
  completeDemoCommand(id) {
    const command=this.state.command;
    if(this.state.source!=='demo'||!command||command.id!==id||command.type!=='move'||command.phase!=='accepted')return false;
    this._emit({command:Object.freeze({...command,phase:'reached',message:'Synthetic destination reached · holding'})});return true;
  }
  setProfile(value) {
    if(!finite(value)||value<0||value>100) return false;
    this._emit({profile:profile(value,this.state.source)});return true;
  }
  setSpeedMode(mode) {
    if(!['precision','standard'].includes(mode)) return false;
    this._emit({speedMode:mode,speedLimit:mode==='precision'?1:2});return true;
  }
  setView(view) {
    if(!['live','history'].includes(view)) return false;
    if(view==='history'&&this.state.source==='demo')this.releaseSteering('History opened');
    this._emit({view,playing:false,selectedTime:this.state.history.at(-1)?.timestamp??null});return true;
  }
  seek(time) {
    if(!finite(time)||this.state.view!=='history'||!this.state.history.length) return false;
    this._emit({selectedTime:Math.max(this.state.history[0].timestamp,Math.min(this.state.history.at(-1).timestamp,time)),playing:false});return true;
  }
  setPlaying(playing) {
    if(typeof playing!=='boolean'||this.state.view!=='history'||!this.state.history.length) return false;
    const patch={playing};
    if(playing&&this.state.selectedTime>=this.state.history.at(-1).timestamp) patch.selectedTime=this.state.history[0].timestamp;
    this.lastTick=Date.now();this._emit(patch);return true;
  }
}
