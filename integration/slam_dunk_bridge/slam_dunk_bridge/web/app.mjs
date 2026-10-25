import { MissionStore } from './model.mjs';
import { createAdapter } from './adapter.mjs';
import { createScene } from './scene.mjs';
import { createCameraFeed } from './camera.mjs';

/** Mount once; returning resources also supports independent UI integration tests. */
export function mountDashboard({ document: doc = globalThis.document, autoStart = true } = {}) {
  if (!doc || !doc.getElementById('main-canvas')) throw new TypeError('Dashboard markup is required');
  const $ = id => doc.getElementById(id);
  const store = new MissionStore(), adapter = createAdapter({ store });
  const removers = [], scenes = {};
  const pageEvents = doc.defaultView ?? globalThis;
  let topFollow = true, mainFollow = true, toastTimer = null, expanded = null;
  let activePress = null, renewalTimer = null;
  let pinArmed=false,pinnedGoal=null,pinQueued=false,pinSending=false,pinAttemptAt=0,pinVersion=0,pinSentVersion=0,pinAckHandled=null;
  function on(id,type,handler) { const el=typeof id==='string'?$(id):id;el.addEventListener(type,handler);removers.push(()=>el.removeEventListener(type,handler)); }
  function text(id,value) { $(id).textContent=String(value); }
  function pressed(id,value) { $(id).setAttribute('aria-pressed',String(value)); }
  function toast(message) { text('toast',message);$('toast').hidden=false;clearTimeout(toastTimer);toastTimer=setTimeout(()=>{$('toast').hidden=true;},3500); }
  function formatTime(seconds) { if(!Number.isFinite(seconds)||seconds<0)return '—';const value=Math.floor(seconds),m=Math.floor(value/60),s=value%60;return `${String(m).padStart(2,'0')}:${String(s).padStart(2,'0')}`; }
  function setTopFollow(value) { topFollow=value;scenes.top?.setFollow(value);pressed('top-follow',value);text('top-follow',`Follow ${value?'on':'off'} ◎`); }
  function setMainFollow(value) { mainFollow=value;scenes.main?.setFollow(value);pressed('main-follow',value);$('main-follow').classList.toggle('active',value); }
  const pickOptions={isPicking:()=>pinArmed,onPick:position=>{
    if(!pinArmed)return;
    pinnedGoal=position;pinVersion++;pinArmed=false;pressed('pin-mode',false);
    $('top-canvas').style.cursor='';$('main-canvas').style.cursor='';
    text('pin-status',`Selected X ${position[0].toFixed(1)}, Y ${position[1].toFixed(1)} m`);
    pinQueued=true;store._emit({navigationAck:null});sendPinnedGoal();
  },onPickError:reason=>text('pin-status',reason)};
  scenes.top=createScene($('top-canvas'),{mode:'top',local:true,follow:true,onFollowChange:value=>setTopFollow(value),...pickOptions});
  scenes.main=createScene($('main-canvas'),{mode:'perspective',follow:true,onFollowChange:value=>setMainFollow(value),...pickOptions});
  scenes.history=createScene($('history-canvas'),{mode:'top',follow:false});
  const camera=createCameraFeed({image:$('camera-image'),empty:$('camera-empty'),status:$('camera-state'),note:$('camera-note')});

  function sendPinnedGoal(){
    if(!pinnedGoal||pinSending)return;
    const state=store.getState();
    if(!state.navigationReady||state.steering||state.connection!=='connected'){
      pinQueued=true;text('pin-status','Destination queued · waiting for flight controller readiness');return;
    }
    pinSending=true;pinQueued=false;pinSentVersion=pinVersion;pinAttemptAt=Date.now();
    releasePress('pin destination');const result=adapter.navigate(pinnedGoal.slice(0,2));
    if(!result.ok){pinSending=false;pinQueued=true;text('pin-status',result.reason??'Control connection unavailable; retrying');}
    else text('pin-status','Destination sent · waiting for PX4 controller acceptance');
  }
  function releasePress(reason='released') {
    const press=activePress;
    activePress=null;
    clearInterval(renewalTimer);renewalTimer=null;
    if(press){
      press.button.classList.remove('held');
      press.button.setAttribute('aria-pressed','false');
      try{if(press.kind==='pointer')press.button.releasePointerCapture?.(press.id);}catch{/* Already released by the browser. */}
    }
    // Also cancels an acquisition whose acknowledgement has not arrived yet.
    adapter.releaseSteering(reason);
  }
  function beginPress(button,kind,id) {
    if(activePress||button.disabled||doc.hidden)return;
    activePress={button,kind,id};
    const result=adapter.startSteering(button.dataset.direction);
    if(!result.ok){releasePress('rejected');toast(result.reason);return;}
    button.classList.add('held');button.setAttribute('aria-pressed','true');
    renewalTimer=setInterval(()=>{
      if(!activePress)return;
      if(doc.hidden){releasePress('page hidden');return;}
      const result=adapter.renewSteering();
      if(!result.ok)releasePress('steering lease ended');
    },100);
  }
  function changeView(view){releasePress('view changed');store.setView(view);if(view==='history')scenes.history.fit();}

  const unsubscribe=store.subscribe(state=>{
    const history=state.view==='history',demo=state.source==='demo',synthetic=demo||state.mode==='synthetic_demo',display=store.getDisplay();
    $('live-screen').hidden=history;$('history-screen').hidden=!history;
    pressed('live-tab',!history);pressed('history-tab',history);$('live-tab').classList.toggle('active',!history);$('history-tab').classList.toggle('active',history);
    text('screen-title',history?'A mission worth revisiting.':'Eyes on the unknown.');
    text('screen-subtitle',history?'Trace the session. See how the picture came together.':'Your live picture. One connected environment.');
    $('source').value=state.source;
    text('source-badge',synthetic?'SYNTHETIC DEMO':state.frameId==='gazebo_world'?(state.controlAvailable?'GAZEBO GROUND TRUTH · MANUAL OVERRIDE':'GAZEBO GROUND TRUTH · READ ONLY'):state.controlAvailable?'LIVE BRIDGE · STEERING':'LIVE BRIDGE · READ ONLY');
    $('source-badge').classList.toggle('demo-badge',synthetic);
    const connectionLabels={demo:'Demo online',connecting:'Connecting to bridge',connected:synthetic?'Synthetic bridge online':'Bridge connected',disconnected:'Bridge disconnected'};
    // Keep the status dot as a real DOM element rather than replacing it via innerHTML.
    const connection=$('connection');connection.replaceChildren();const dot=doc.createElement('i');connection.appendChild(dot);connection.appendChild(doc.createTextNode(connectionLabels[state.connection]??'Waiting for data'));
    connection.classList.toggle('stale',state.connection==='disconnected'||state.connection==='connecting'||state.stale.pose||state.stale.map);
    text('scene-state',state.connection==='disconnected'?'DISCONNECTED · LAST FRAME':state.stale.map?'MAP WAITING / STALE':synthetic?'LIVE DEMO FEED':'LIVE BRIDGE FEED');
    $('scene-state').parentElement.classList.toggle('stale',state.stale.map||state.stale.pose);
    text('frame',(state.frameId??'—').toUpperCase());
    const p=state.pose?.position;
    text('coordinates',p?`X ${p[0].toFixed(2)}  ·  Y ${p[1].toFixed(2)}  ·  Z ${p[2].toFixed(2)} m`:'X —  ·  Y —  ·  Z —');
    text('point-count',state.mapPoints.length.toLocaleString());
    text('map-freshness',state.stale.map?'Map stale / waiting':state.stale.pose?'Pose stale / waiting':'Map & pose current');
    $('map-freshness').classList.toggle('stale',state.stale.map||state.stale.pose);
    $('empty-state').hidden=state.mapPoints.length>0;
    text('speed',state.stale.pose||state.speed===null?'—':state.speed.toFixed(2));
    text('speed-source',synthetic?'SYNTHETIC TELEMETRY':state.frameId==='gazebo_world'?'PX4 VELOCITY':'MEASURED TELEMETRY');
    $('mission-start').disabled=history||state.missionPending||!state.missionStartReady||state.controlConnection!=='connected';
    $('mission-home').disabled=history||state.missionPending||!state.missionReturnReady||state.controlConnection!=='connected';
    $('mission-start').title=state.startReason??'Waiting for standby controller';
    text('mission-status',!state.missionStartReady&&['PREFLIGHT','DONE','ABORT'].includes(state.missionPhase)&&state.startReason?state.startReason:state.missionAck?.reason??(state.missionStartReady?(state.startReason??'Ready · Start takes off and follows the course'):state.missionPhase?`Flight: ${state.missionPhase}`:'Waiting for standby controller'));
    text('battery','97%');
    text('battery-source','BATTERY · FIXED');
    $('battery').title='Fixed display value; not live battery telemetry';
    text('flight-source',state.flightSource==='controller_last'?'FLIGHT TIME · LAST RECEIVED':'FLIGHT TIME');
    $('battery-fill').style.width='97%';
    text('flight-time',state.flightSource==='placeholder'?'—':formatTime(state.flightSeconds));
    text('vehicle-status',state.vehicleEvent?.startsWith('COLLISION')?'COLLISION / FAILED':synthetic?'DEMO':state.controlAvailable?'CONNECTED':'READ ONLY');
    text('vehicle-note',synthetic?'Synthetic telemetry · no vehicle connected':state.frameId==='gazebo_world'?'Live PX4 simulation telemetry.':'Unavailable fields show — · no estimate inferred');
    $('margin').value=String(state.profile.value);
    text('margin-value',state.profile.value===0?'Default':state.profile.value===100?'More clearance':`${state.profile.value}% range`);
    text('static-margin',`${state.profile.inflation_hgp.toFixed(2)} m`);text('dynamic-margin',`${state.profile.dyn_base_inflation_m.toFixed(2)} m`);
    text('profile-status',demo?'Synthetic preference · not ROS applied':'Configuration proposal · backend apply required');
    $('speed-slow').classList.toggle('selected',state.speedMode==='precision');$('speed-standard').classList.toggle('selected',state.speedMode==='standard');
    pressed('speed-slow',state.speedMode==='precision');pressed('speed-standard',state.speedMode==='standard');
    $('speed-slow').disabled=!demo;$('speed-standard').disabled=!demo;
    const canSteer=(demo||state.controlAvailable)&&!history&&!!state.pose&&!state.stale.pose&&!state.stale.map&&state.explorationMode!=='paused';
    for(const button of doc.querySelectorAll('[data-direction]'))button.disabled=!canSteer;
    $('stop').disabled=history||!(demo||state.controlConnection==='connected'&&state.controlCapabilities?.pause);
    const paused=state.explorationMode==='paused';
    text('pause-label',paused?'RESUME':'PAUSE');$('stop').setAttribute('aria-label',paused?'Resume exploration':'Pause exploration');
    text('exploration-mode',({autonomous:'Autonomous exploration',steering:'Operator steering',paused:'Exploration paused',waiting:'Waiting for exploration',fault:'Control fault'})[state.explorationMode]??'Waiting for exploration');
    text('control-help',state.frameId==='gazebo_world'&&state.controlAvailable?'Hold to steer WITHOUT avoidance. Release to rejoin course.':demo||state.controlAvailable?'Hold a direction to steer. Release to explore.':(state.controlConnection==='connecting'?'Connecting to PX4 controls…':state.frameId==='gazebo_world'?'Waiting for PX4 manual-control run; reconnecting automatically':'Read-only telemetry · connect the command supervisor'));
    text('command-status',state.vehicleEvent?.startsWith('COLLISION')?state.vehicleEvent:state.command?.message??(demo?'Autonomous exploration active':'Command supervisor not connected'));
    $('stop').title=!demo&&!state.controlCapabilities?.pause?'Planner braking/hold service is not available':'';
    if(activePress&&!canSteer)releasePress('steering unavailable');
    text('data-title',demo?'A space taking shape.':'Your backend. Your live view.');
    text('data-note',demo?'The demo reveals an indoor site as the drone explores. Connect the bridge when your backend is ready.':state.statusMessage);
    text('footer-status',demo?'Browser demo · hold to steer':state.controlAvailable?(state.frameId==='gazebo_world'?'PX4 manual override · release to rejoin course':'SANDO supervisor connected · leased steering'):'Read-only telemetry · no command transport');
    const plan=state.plan,planCurrent=plan&&!state.stale.pose&&state.connection==='connected';
    if(planCurrent&&plan.buffer_recovery&&state.missionPhase==='NAVIGATE')text('mission-status','Clearing obstacle buffer · up to 0.15 m/s');
    if(state.missionPhase==='GOAL_HOLD')text('mission-status','Aligning over clear landing site');
    if(pinSending&&state.controlConnection==='disconnected'){pinSending=false;pinQueued=true;text('pin-status','Control disconnected · destination queued for reconnect');}
    if(state.navigationAck&&state.navigationAck.requestId!==pinAckHandled){
      pinAckHandled=state.navigationAck.requestId;pinSending=false;
      if(pinVersion===pinSentVersion){
        if(!state.navigationAck.accepted&&state.navigationAck.retryable){pinQueued=true;text('pin-status','Queued · waiting for fresh control state');}
        else text('pin-status',(state.navigationAck.accepted?'Accepted: ':'Rejected: ')+state.navigationAck.reason);
        if(state.navigationAck.accepted&&state.navigationAck.destination)pinnedGoal=[...state.navigationAck.destination,pinnedGoal?.[2]??2];
      }else pinQueued=true;
    }
    if(pinQueued&&state.navigationReady&&!state.steering&&Date.now()-pinAttemptAt>1000)queueMicrotask(()=>{if(pinQueued)sendPinnedGoal();});
    $('pin-mode').disabled=history||state.source!=='live'||state.frameId!=='gazebo_world';
    $('pin-go').disabled=!pinnedGoal||pinSending||!state.navigationReady||!!state.steering||history;
    $('pin-go').title=state.navigationReady?'Send selected destination':'Start a connected manual-control flight run to enable destinations';
    const plannedTarget=pinnedGoal??(planCurrent?plan.target:null);
    const keepouts=planCurrent?plan.keepouts??[]:[],bounds=planCurrent?plan.bounds:null;
    const route=planCurrent?plan.route:[],prediction=planCurrent&&plan.prediction_valid?plan.prediction:[];
    const target=plannedTarget??(state.command?.type==='move'&&['accepted','moving'].includes(state.command.phase)?state.command.target:null);
    scenes.top.setData({points:state.mapPoints,pose:state.pose,trail:state.trail,target,route,prediction,keepouts,bounds});
    scenes.main.setData({points:state.mapPoints,pose:state.pose,trail:state.trail,target,route,prediction,keepouts,bounds});
    scenes.history.setData({points:display.combinedPoints,pose:display.pose,trail:display.trail,target:null});
    camera.setVisible(!history&&!doc.hidden);
    const frames=state.history,min=frames[0]?.timestamp??0,max=frames.at(-1)?.timestamp??1,selected=state.selectedTime??max;
    $('timeline').min=String(min);$('timeline').max=String(Math.max(min+1,max));$('timeline').value=String(selected);$('timeline').disabled=frames.length<2;
    text('replay-time',formatTime((selected-min)/1000));text('history-duration',`${formatTime((max-min)/1000)} recorded`);
    text('recording-count',`${frames.length} recorded frames`);
    text('history-chip',selected>=max?'LATEST RECORDED FRAME':'HISTORICAL FRAME · CONTROLS DISABLED');
    text('playback',state.playing?'Ⅱ  Pause recording':'▶  Play recording');$('playback').setAttribute('aria-label',state.playing?'Pause recorded mission':'Play recorded mission');$('playback').disabled=frames.length<2;
    text('detail-time',display.timestamp?new Date(display.timestamp).toLocaleTimeString([], {hour12:false}):'—');
    text('detail-position',display.pose?display.pose.position.map(n=>n.toFixed(2)).join(' / '):'—');
    text('detail-points',display.combinedPoints.length.toLocaleString());text('detail-frame',state.frameId??'—');text('detail-source',synthetic?'Synthetic demo':'Live bridge');
  });

  const settings=new URLSearchParams(globalThis.location?.search??'');
  function connectSource(source){
    pinQueued=false;pinSending=false;pinnedGoal=null;pinArmed=false;pinAckHandled=null;pinVersion++;pinSentVersion=pinVersion;
    releasePress('source changed');
    if(source==='demo')adapter.startDemo();
    else{adapter.connectLive();const control=settings.get('control')??(globalThis.location?.hostname&&globalThis.location.port==='8765'?`${globalThis.location.protocol==='https:'?'wss':'ws'}://${globalThis.location.host}/control`:null);if(control&&!adapter.connectControl(control))toast('Invalid command supervisor URL');}
    setTopFollow(true);setMainFollow(true);
  }
  on('live-tab','click',()=>changeView('live'));on('history-tab','click',()=>changeView('history'));
  on('open-history','click',()=>{changeView('history');$('history-tab').focus();});on('return-live','click',()=>{changeView('live');$('live-tab').focus();});
  on('source','change',()=>connectSource($('source').value));
  for(const [id,action] of [['mission-start','start'],['mission-home','return_home']])on(id,'click',()=>{
    pinQueued=false;pinSending=false;pinnedGoal=null;pinArmed=false;pinVersion++;
    const result=adapter.mission(action);store._emit({missionAck:null,navigationAck:null});
    text('mission-status',result.ok?'Request sent · awaiting controller acceptance':result.reason);
  });
  on('pin-mode','click',()=>{pinArmed=!pinArmed;pressed('pin-mode',pinArmed);text('pin-status',pinArmed?'Click either map to place your destination':'Pin placement canceled');$('top-canvas').style.cursor=pinArmed?'crosshair':'';$('main-canvas').style.cursor=pinArmed?'crosshair':'';});
  on('pin-go','click',()=>{if(!pinnedGoal)return;pinSending=false;pinQueued=true;store._emit({navigationAck:null});sendPinnedGoal();});
  on('top-follow','click',()=>setTopFollow(!topFollow));on('main-follow','click',()=>setMainFollow(!mainFollow));on('main-fit','click',()=>scenes.main.fit());on('history-fit','click',()=>scenes.history.fit());
  for(const button of doc.querySelectorAll('[data-direction]')){
    button.setAttribute('aria-pressed','false');
    on(button,'pointerdown',event=>{
      if(event.isPrimary===false||event.button!==undefined&&event.button!==0)return;
      event.preventDefault();beginPress(button,'pointer',event.pointerId);
      if(activePress)try{button.setPointerCapture?.(event.pointerId);}catch{releasePress('pointer capture failed');}
    });
    for(const type of ['pointerup','pointercancel','lostpointercapture'])on(button,type,event=>{
      if(activePress?.kind==='pointer'&&activePress.id===event.pointerId)releasePress(type);
    });
    on(button,'keydown',event=>{
      if(![' ','Enter'].includes(event.key))return;
      event.preventDefault();if(!event.repeat)beginPress(button,'keyboard',event.key);
    });
    on(button,'keyup',event=>{
      if(activePress?.kind==='keyboard'&&activePress.id===event.key){event.preventDefault();releasePress('key released');}
    });
    on(button,'blur',()=>{if(activePress?.button===button)releasePress('button lost focus');});
  }
  const visibility=()=>{if(doc.hidden)releasePress('page hidden');camera.setVisible(store.getState().view==='live'&&!doc.hidden);};
  on(doc,'visibilitychange',visibility);
  if(typeof pageEvents.addEventListener==='function'){
    on(pageEvents,'blur',()=>releasePress('window lost focus'));
    on(pageEvents,'pagehide',()=>releasePress('page closed'));
    on(pageEvents,'pointerup',event=>{if(activePress?.kind==='pointer'&&activePress.id===event.pointerId)releasePress('pointer released');});
  }
  on('stop','click',()=>{
    releasePress('pause toggled');
    const result=store.getState().explorationMode==='paused'?adapter.resumeExploration():adapter.pauseExploration();
    if(!result.ok)toast(result.reason);
  });
  on('camera-form','submit',event=>{event.preventDefault();if(!camera.connect($('camera-url').value))toast('Use an HTTP or HTTPS camera stream URL without embedded credentials.');});
  on('camera-disconnect','click',()=>camera.disconnect());
  on('margin','input',()=>adapter.setProfile(Number($('margin').value)));
  on('speed-slow','click',()=>adapter.setSpeedMode('precision'));on('speed-standard','click',()=>adapter.setSpeedMode('standard'));
  on('timeline','input',()=>store.seek(Number($('timeline').value)));
  on('playback','click',()=>{const state=store.getState();if(!state.playing&&state.history.length&&state.selectedTime>=state.history.at(-1).timestamp)store.seek(state.history[0].timestamp);store.setPlaying(!state.playing);});
  function historyMode(mode) { scenes.history.setMode(mode);$('history-2d').classList.toggle('selected',mode==='top');$('history-3d').classList.toggle('selected',mode==='perspective');pressed('history-2d',mode==='top');pressed('history-3d',mode==='perspective'); }
  on('history-2d','click',()=>historyMode('top'));on('history-3d','click',()=>historyMode('perspective'));
  async function expand(id) {
    const panel=$(id);
    if(doc.fullscreenElement){await doc.exitFullscreen?.();return;}
    if(expanded){expanded.classList.remove('expanded');expanded=null;return;}
    if(typeof panel.requestFullscreen==='function'){try{await panel.requestFullscreen();return;}catch{/* Embedded browsers can reject fullscreen; use the same panel as an overlay. */}}
    panel.classList.add('expanded');expanded=panel;
  }
  on('main-fullscreen','click',()=>expand('main-viewer-panel'));on('history-fullscreen','click',()=>expand('history-viewer-panel'));
  on('camera-fullscreen','click',()=>expand('camera-view'));
  const keyHandler=event=>{if(event.key==='Escape'&&expanded){expanded.classList.remove('expanded');expanded=null;}};doc.addEventListener('keydown',keyHandler);removers.push(()=>doc.removeEventListener('keydown',keyHandler));
  if(autoStart){connectSource(settings.get('source')==='live'?'live':'demo');const url=settings.get('camera');if(url){$('camera-url').value=url;if(!camera.connect(url))toast('Invalid camera stream URL');}}
  return Object.freeze({store,adapter,scenes,camera,destroy(){releasePress('dashboard destroyed');unsubscribe();adapter.dispose();camera.destroy();clearTimeout(toastTimer);for(const remove of removers)remove();for(const scene of Object.values(scenes))scene.destroy();expanded?.classList.remove('expanded');}});
}

if (typeof document !== 'undefined' && document.getElementById('main-canvas')) mountDashboard();
