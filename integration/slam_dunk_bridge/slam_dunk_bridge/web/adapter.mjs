import {createDemo} from './demo.mjs';
import {createUnknownDemo,UNKNOWN_ENVIRONMENTS} from './unknown.mjs';

/** Telemetry and leased control use separate sockets. /stream stays read-only. */
export function createAdapter({store,room=false,WebSocketImpl=globalThis.WebSocket,clock=()=>Date.now(),schedule=globalThis.setInterval,cancel=globalThis.clearInterval}={}) {
  if(!store||typeof store.ingest!=='function'||typeof clock!=='function'||typeof schedule!=='function'||typeof cancel!=='function')throw new TypeError('Adapter requires store, clock and timer functions');
  let timer=null,socket=null,controlSocket=null,demo=null,generation=0,controlGeneration=0,retryAt=null,liveUrl=null;
  let held=null,sequence=0,controlStatusAt=null,lastControlStatus=null,releasePending=false;
  const prefix=globalThis.crypto?.randomUUID?.()??`${clock()}-${Math.random().toString(36).slice(2)}`;
  const requestId=()=>`${prefix}-${++sequence}`;
  const validUrl=(url,path)=>{
    let parsed;try{parsed=new URL(url);}catch{return null;}
    return ['ws:','wss:'].includes(parsed.protocol)&&parsed.pathname===path&&!parsed.username&&!parsed.password&&!parsed.hash&&!parsed.search?parsed.href:null;
  };
  const send=message=>{
    if(!controlSocket||controlSocket.readyState!==1)return false;
    try{controlSocket.send(JSON.stringify({protocol_version:1,...message}));return true;}catch{return false;}
  };
  function releaseSteering(reason='released') {
    const active=held;held=null;
    if(store.getState().source==='demo'){
      if(active)demo?.clearManual({phase:'release',direction:active.direction,now:clock()});return store.releaseSteering(reason);
    }
    if(!active)return {ok:true};
    // Cancel the original request when release occurs before its lease ack.
    // Server tombstones prevent delayed acknowledgments from resurrecting it.
    releasePending=true;
    const message=active.leaseId
      ?{type:'steer_release',request_id:requestId(),lease_id:active.leaseId,seq:++active.seq,epoch:active.epoch}
      :{type:'steer_cancel',request_id:active.requestId,epoch:active.startEpoch};
    const ok=send(message);
    if(!ok&&controlSocket)controlSocket.close();
    return {ok,reason:ok?reason:'Control connection lost; backend lease expires'};
  }
  const closeControl=()=>{
    releaseSteering('Control disconnected');controlGeneration++;
    if(controlSocket){const old=controlSocket;controlSocket=null;old.onopen=old.onmessage=old.onclose=old.onerror=null;old.close();}
    controlStatusAt=null;lastControlStatus=null;held=null;releasePending=false;
  };
  const teardown=()=>{
    closeControl();generation++;if(timer!==null)cancel(timer);timer=null;
    if(socket){const old=socket;socket=null;old.onclose=old.onmessage=old.onopen=old.onerror=null;old.close();}
    demo=null;retryAt=null;
  };
  const open=()=>{
    const current=generation;
    try{socket=new WebSocketImpl(liveUrl);}catch{store.setConnection('disconnected');retryAt=clock()+1500;return;}
    const active=socket;
    active.onopen=()=>{
      if(current!==generation||socket!==active)return;
      releaseSteering('Telemetry reconnected');store.reset('live');store.setConnection('connected');
      if(controlSocket?.readyState===1){store.setControlConnection('connected');if(lastControlStatus&&clock()-controlStatusAt<500&&lastControlStatus.mode!=='steering')store.ingestControl(lastControlStatus);}
    };
    active.onmessage=e=>{
      if(current!==generation||socket!==active)return;
      try{if(typeof e.data!=='string'||e.data.length>4000000)throw new Error('Oversized data');store.ingest(JSON.parse(e.data),clock());}catch{store.ingest(null,clock());}
    };
    active.onclose=()=>{if(current!==generation||socket!==active)return;releaseSteering('Telemetry disconnected');socket=null;store.setConnection('disconnected');retryAt=clock()+1500;};
    active.onerror=()=>{if(current===generation&&socket===active)active.close();};
  };
  const tick=()=>{
    const now=clock();demo?.step(now);store.tick(now);
    if(held&&store.getState().source==='demo'&&!store.getState().steering)held=null;
    if(retryAt!==null&&now>=retryAt){retryAt=null;store.setConnection('connecting');open();}
    if(controlStatusAt!==null&&now-controlStatusAt>1500){releaseSteering('Control status timed out');controlSocket?.close();store.setControlConnection('disconnected');controlStatusAt=null;}
    if(held&&store.getState().source==='live'&&(store.getState().view!=='live'||store.getState().stale.pose||store.getState().stale.map||store.getState().connection!=='connected'))releaseSteering('Live telemetry unavailable');
  };
  const adapter={
    startDemo({environment=null}={}){if(environment&&(!UNKNOWN_ENVIRONMENTS.some(w=>w.id===environment)||room))return false;teardown();store.reset('demo');if(room)store.setSpeedMode('standard');demo=environment?createUnknownDemo(store,environment):createDemo(store,26,{room});demo.step(clock());timer=schedule(tick,100);return true;},
    connectLive(url){
      const fallback=globalThis.location?`${location.protocol==='https:'?'wss':'ws'}://${location.host}/stream`:null;
      const parsed=validUrl(url??fallback,'/stream');if(!parsed||typeof WebSocketImpl!=='function')return false;
      teardown();liveUrl=parsed;store.reset('live');open();timer=schedule(tick,100);return true;
    },
    connectControl(url){
      const parsed=validUrl(url,'/control');if(!parsed||typeof WebSocketImpl!=='function'||store.getState().source!=='live')return false;
      closeControl();const current=controlGeneration;store.setControlConnection('connecting');
      try{controlSocket=new WebSocketImpl(parsed);}catch{store.setControlConnection('disconnected');return false;}
      const active=controlSocket;
      active.onopen=()=>{if(current===controlGeneration&&controlSocket===active){store.setControlConnection('connected');controlStatusAt=clock();}};
      active.onmessage=e=>{
        if(current!==controlGeneration||controlSocket!==active)return;
        let message;try{if(typeof e.data!=='string'||e.data.length>16384)return;message=JSON.parse(e.data);}catch{return;}
        if(!store.ingestControl(message))return;
        controlStatusAt=clock();lastControlStatus=message;
        if(releasePending&&message.mode!=='steering')releasePending=false;
        if(held&&message.active_request_id===held.requestId&&message.mode==='steering'&&message.lease_id){held.leaseId=message.lease_id;held.epoch=message.epoch;}
        if(held&&message.last_request_id===held.requestId&&message.accepted===false)held=null;
        if(held&&held.leaseId&&(message.epoch!==held.epoch||message.mode!=='steering'||message.lease_id!==held.leaseId))held=null;
      };
      active.onclose=()=>{if(current!==controlGeneration||controlSocket!==active)return;held=null;releasePending=false;controlSocket=null;controlStatusAt=null;lastControlStatus=null;store.setControlConnection('disconnected');};
      active.onerror=()=>{if(current===controlGeneration&&controlSocket===active)active.close();};return true;
    },
    startSteering(direction){
      if(held)return {ok:false,reason:'A steering direction is already held'};
      if(releasePending)return {ok:false,reason:'Waiting for steering release acknowledgment'};
      if(store.getState().source==='demo'){
        if((room||demo?.snapshot().hidden)&&!demo?.snapshot().started)return {ok:false,reason:'Click Start before pushing the drone'};
        const result=store.startSteering(direction,clock());if(result.ok){held={requestId:result.steering.leaseId,direction};demo?.clearManual({phase:'start',direction,now:clock()});}return result;
      }
      const s=store.getState();
      if(!['forward','backward','left','right','up','down'].includes(direction))return {ok:false,reason:'Unknown direction'};
      if(s.view!=='live'||s.connection!=='connected'||s.stale.map||s.stale.pose||s.explorationMode==='paused'||!s.controlReady||!s.controlCapabilities.steering||!s.controlAvailable)return {ok:false,reason:'Fresh live telemetry and a ready steering gateway are required'};
      const active={requestId:requestId(),direction,startEpoch:s.controlEpoch,epoch:s.controlEpoch,leaseId:null,seq:0};held=active;
      if(!send({type:'steer_start',request_id:active.requestId,direction,epoch:s.controlEpoch})){held=null;return {ok:false,reason:'Control gateway unavailable'};}
      return {ok:true,pending:true};
    },
    renewSteering(){
      if(!held)return {ok:false,reason:'No held steering request'};
      if(store.getState().source==='demo'){
        const result=store.renewSteering(clock());if(!result.ok)held=null;return result;
      }
      if(!held.leaseId)return {ok:true,pending:true};
      if(!send({type:'steer_renew',request_id:requestId(),lease_id:held.leaseId,seq:++held.seq,epoch:held.epoch})){releaseSteering('Renewal failed');return {ok:false,reason:'Renewal failed'};}
      return {ok:true};
    },
    releaseSteering,
    pauseExploration(){
      if(store.getState().source==='demo'){releaseSteering('Exploration paused');demo?.clearManual();return store.pauseExploration();}
      if(!store.getState().controlReady||!store.getState().controlCapabilities.pause)return {ok:false,reason:'Physical pause is not supported by this backend'};
      // An implemented physical Pause must atomically supersede the lease; do
      // not send a release first and then race its incremented control epoch.
      held=null;releasePending=true;
      return {ok:send({type:'pause',request_id:requestId(),epoch:store.getState().controlEpoch})};
    },
    resumeExploration(){
      if(store.getState().source==='demo'){releaseSteering('Exploration resumed');demo?.clearManual();return store.resumeExploration();}
      if(!store.getState().controlReady||!store.getState().controlCapabilities.pause)return {ok:false,reason:'Physical pause/resume is not supported by this backend'};
      if(store.getState().explorationMode!=='paused')return {ok:false,reason:'Exploration is not paused'};
      return {ok:send({type:'resume',request_id:requestId(),epoch:store.getState().controlEpoch})};
    },
    // Compatibility for older callers; dashboard buttons use leased steering.
    command(direction,distance=1){if(demo?.snapshot().hidden)return {ok:false,reason:'Use held guidance or choose an observed free-space target'};const result=store.command(direction,distance);if(result.ok)demo?.move(result.command);return result;},
    stop(){releaseSteering('Legacy stop');const result=store.stop();if(result.ok)demo?.stop();return result;},
    editSimulation(action,point,kind,options){
      if(store.getState().source!=='demo'||!demo)return {ok:false,reason:'Obstacle editing is only available in the synthetic demo'};
      if(demo.snapshot().hidden)return {ok:false,reason:'Hidden challenge environments cannot be edited'};
      if(action==='add')return demo.addObstacle(point,kind,options);
      if(action==='remove')return demo.removeObstacle(point);
      if(action==='motion')return demo.setObjectMotion(point,kind);
      if(action==='clear')return demo.clearObstacles();
      return {ok:false,reason:'Unknown scene action'};
    },
    startSimulation(){if(!room||store.getState().source!=='demo'||!demo)return {ok:false,reason:'Start is only available in the flight room'};const result=demo.start();store.resumeExploration();return result;},
    startExploration(){if(store.getState().source!=='demo'||!demo?.snapshot().hidden)return {ok:false,reason:'Choose an unknown environment first'};const result=demo.start();if(result.ok)store.resumeExploration();return result;},
    getSimulation(){return demo?.snapshot()??null;},
    setSimulationGoal(point){if(store.getState().source!=='demo')return {ok:false,reason:'Simulation goals are only available in the synthetic demo'};return demo?.setGoal(point)??{ok:false,reason:'Start the demo first'};},
    setProfile(value){return store.setProfile(value);},
    setSpeedMode(mode){return store.setSpeedMode(mode);},
    dispose(){teardown();return true;}
  };
  return Object.freeze(adapter);
}
