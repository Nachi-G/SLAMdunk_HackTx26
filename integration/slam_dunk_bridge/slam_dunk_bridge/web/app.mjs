import { MISSIONS } from './planner.mjs';
import { MissionStore } from './model.mjs';
import { createAdapter } from './adapter.mjs';
import { createScene } from './scene.mjs';
import { createPov } from './pov.mjs';
import { createCameraFeed } from './camera.mjs';

/** Mount once; returning resources also supports independent UI integration tests. */
export function mountDashboard({ document: doc = globalThis.document, autoStart = true } = {}) {
  if (!doc || !doc.getElementById('main-canvas')) throw new TypeError('Dashboard markup is required');
  const $ = id => doc.getElementById(id);
  const store = new MissionStore(), adapter = createAdapter({ store });
  const removers = [], scenes = {};
  const pageEvents = doc.defaultView ?? globalThis;
  let topFollow = true, mainFollow = true, toastTimer = null, expanded = null;
  let activePress = null, renewalTimer = null, showPlan = true, simulationView = 'map', editorTool = 'goal', selectedObject = null;
  let environmentId='sandbox';
  function on(id,type,handler) { const el=typeof id==='string'?$(id):id;el.addEventListener(type,handler);removers.push(()=>el.removeEventListener(type,handler)); }
  function text(id,value) { $(id).textContent=String(value); }
  function pressed(id,value) { $(id).setAttribute('aria-pressed',String(value)); }
  function toast(message) { text('toast',message);$('toast').hidden=false;clearTimeout(toastTimer);toastTimer=setTimeout(()=>{$('toast').hidden=true;},3500); }
  function formatTime(seconds) { if(!Number.isFinite(seconds)||seconds<0)return '—';const value=Math.floor(seconds),m=Math.floor(value/60),s=value%60;return `${String(m).padStart(2,'0')}:${String(s).padStart(2,'0')}`; }
  function setTopFollow(value) { topFollow=value;scenes.top?.setFollow(value);pressed('top-follow',value);text('top-follow',`Follow ${value?'on':'off'} ◎`); }
  function setMainFollow(value) { mainFollow=value;scenes.main?.setFollow(value);pressed('main-follow',value);$('main-follow').classList.toggle('active',value); }
  scenes.top=createScene($('top-canvas'),{mode:'top',local:true,follow:true,onFollowChange:value=>setTopFollow(value),onGoal:point=>{if(editorTool==='goal')sendGoal(point);},onMapClick:(point,object)=>editArena(point,object)});
  scenes.main=createScene($('main-canvas'),{mode:'perspective',follow:true,onFollowChange:value=>setMainFollow(value),onMapClick:(point,object)=>editArena(point,object)});
  scenes.history=createScene($('history-canvas'),{mode:'top',follow:false});
  const pov=createPov($('pov-canvas'));
  const camera=createCameraFeed({image:$('camera-image'),empty:$('camera-empty'),status:$('camera-state'),note:$('camera-note')});

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
    text('source-badge',synthetic?'SYNTHETIC DEMO':state.controlAvailable?'LIVE BRIDGE · STEERING':'LIVE BRIDGE · READ ONLY');
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
    text('speed-source',synthetic?'SYNTHETIC TELEMETRY':'MEASURED TELEMETRY');
    text('battery',state.battery===null?'—':`${Math.round(state.battery)}%`);
    $('battery-fill').style.width=state.battery===null?'0%':`${state.battery}%`;
    text('flight-time',formatTime(state.flightSeconds));
    text('vehicle-status',synthetic?'DEMO':state.controlAvailable?'CONNECTED':'READ ONLY');
    text('vehicle-note',synthetic?'Synthetic telemetry · no vehicle connected':'Unavailable fields show — · no estimate inferred');
    $('margin').value=String(state.profile.value);
    text('margin-value',state.profile.value===0?'Default':state.profile.value===100?'More clearance':`${state.profile.value}% range`);
    text('static-margin',`${state.profile.inflation_hgp.toFixed(2)} m`);text('dynamic-margin',`${state.profile.dyn_base_inflation_m.toFixed(2)} m`);
    text('profile-status',demo?'Applied to browser planner · not ROS':'Configuration proposal · backend apply required');
    $('speed-slow').classList.toggle('selected',state.speedMode==='precision');$('speed-standard').classList.toggle('selected',state.speedMode==='standard');
    pressed('speed-slow',state.speedMode==='precision');pressed('speed-standard',state.speedMode==='standard');
    $('speed-slow').disabled=!demo;$('speed-standard').disabled=!demo;
    const canSteer=(demo||state.controlAvailable)&&!history&&!!state.pose&&!state.stale.pose&&!state.stale.map&&state.explorationMode!=='paused';
    for(const button of doc.querySelectorAll('[data-direction]'))button.disabled=!canSteer;
    $('stop').disabled=history||!(demo||state.controlConnection==='connected'&&state.controlCapabilities?.pause);
    const paused=state.explorationMode==='paused';
    text('pause-label',paused?'RESUME':'PAUSE');$('stop').setAttribute('aria-label',paused?'Resume exploration':'Pause exploration');
    text('exploration-mode',({autonomous:'Autonomous exploration',steering:'Operator steering',paused:'Exploration paused',waiting:'Waiting for exploration',fault:'Control fault'})[state.explorationMode]??'Waiting for exploration');
    text('control-help',demo||state.controlAvailable?'Hold a direction to steer. Release to explore.':'Read-only telemetry · connect the command supervisor');
    text('command-status',state.command?.message??(demo?'Autonomous exploration active':'Command supervisor not connected'));
    $('stop').title=!demo&&!state.controlCapabilities?.pause?'Planner braking/hold service is not available':'';
    if(activePress&&!canSteer)releasePress('steering unavailable');
    text('data-title',demo?'Make your own path.':'Your backend. Your live view.');
    text('data-note',demo?'Choose a target and add moving objects in Build arena. Follow the planned route in 3D or Drone POV.':state.statusMessage);
    text('footer-status',demo?'Browser demo · hold to steer':state.controlAvailable?'SANDO supervisor connected · leased steering':'Read-only telemetry · no command transport');
    const target=state.command?.type==='move'&&['accepted','moving'].includes(state.command.phase)?state.command.target:null;
    const simulation=demo?adapter.getSimulation():null;
    const hidden=simulation?.hidden===true;
    $('planner-panel').hidden=!demo||hidden;
    $('environment-panel').hidden=!demo;
    $('environment').value=environmentId;
    $('unknown-controls').hidden=!hidden;
    text('environment-challenge',hidden?simulation.challenge:'Build and edit a visible obstacle course.');
    if(hidden){
      text('survey-status',simulation.status);text('survey-area',`${simulation.observedArea.toFixed(1)} m²`);
      text('survey-coverage',`${(simulation.coverage*100).toFixed(1)}%`);text('survey-frontiers',simulation.frontiers);text('survey-scans',simulation.scanSequence);
      $('survey-progress').style.width=`${simulation.coverage*100}%`;
      $('begin-exploration').disabled=simulation.started;text('begin-exploration',simulation.started?'Exploration started':'Begin exploration ▶');
      text('data-title',`${simulation.name} · discovering the environment`);text('data-note','The map is built from occluded range scans. Dark space has not been observed.');
      text('exploration-mode',state.explorationMode==='paused'?'Exploration paused':simulation.started?'Frontier exploration':'Ready to explore');
      text('command-status',simulation.status);text('control-help','Hold to guide through observed free space. Release to explore.');
      text('scene-state',`MAPPING UNKNOWN ${simulation.environment}`);
      $('empty-state').hidden=simulation.observedCells>0;
      for(const button of doc.querySelectorAll('[data-direction]'))button.disabled=button.disabled||!simulation.started;
    }
    const exploration=!demo?state.exploration:null;
    $('exploration-panel').hidden=!exploration;
    if(exploration){
      text('exploration-state',exploration.state.toUpperCase());text('exploration-reason',exploration.reason);
      text('exploration-frontiers',exploration.frontier_cells);text('exploration-reachable',exploration.reachable_cells);
      text('exploration-unknown',exploration.unknown_voxels);text('exploration-scope',exploration.completion_scope);
    }
    $('simulation-views').hidden=!demo;
    const isPov=demo&&!hidden&&simulationView==='pov';
    $('view-pov').hidden=hidden;
    text('view-arena',hidden?'Survey 2D':'Build arena');
    $('main-canvas-wrap').classList.toggle('pov-mode',isPov);
    text('viewer-kind',hidden?'OBSERVED FREE SPACE / OCCUPIED POINTS':isPov?'SIMULATED CAMERA':demo&&simulationView==='arena'?'CLICK TO EDIT':'OCCUPIED GEOMETRY');
    $('main-canvas').setAttribute('aria-label',demo&&simulationView==='arena'?'Arena editor. Click to use the selected goal or obstacle tool. Drag to pan; scroll to zoom.':'Interactive three dimensional map. Drag to orbit; scroll to zoom.');
    $('pov-canvas').hidden=!isPov;$('pov-label').hidden=!isPov;$('main-canvas').hidden=isPov;
    $('main-follow').disabled=isPov;$('main-fit').disabled=isPov;
    text('viewer-title',hidden?`${simulation.name} · observed map`:isPov?'Drone POV':demo&&simulationView==='arena'?'Build your obstacle course':'Live 3D environment');
    text('viewer-hint',hidden?'SENSOR OBSERVATIONS ONLY · DARK SPACE IS UNKNOWN':isPov?'SIMULATED POV · SPACE TO PAUSE':demo&&simulationView==='arena'?'CLICK TO EDIT · DRAG TO PAN · SCROLL TO ZOOM':'DRAG TO ORBIT · SCROLL TO ZOOM');
    for(const [id,view]of [['view-pov','pov'],['view-arena','arena'],['view-map','map']]){pressed(id,simulationView===view);$(id).classList.toggle('selected',simulationView===view);}
    const selectedActor=selectedObject?.kind==='moving'?simulation?.movers.find(o=>o.id===selectedObject.id):null;
    const selectedBox=selectedObject?.kind==='static'?simulation?.placed.find(r=>r.every((v,i)=>v===selectedObject.box[i])):null;
    if(!selectedActor&&!selectedBox)selectedObject=null;
    $('selected-object').hidden=!selectedObject;
    text('selected-object-label',selectedActor?`Selected ${selectedActor.id}`:'Selected static object');
    for(const axis of ['x','y'])pressed('selected-motion-'+axis,selectedActor?.axis===axis);
    if(simulation){text('obstacle-count',`${simulation.objectCount} / 24`);if(isPov)pov.setData(simulation);}

    if(simulation&&!hidden){text('planner-status',simulation.status);text('planner-clearance',`${simulation.clearance.toFixed(2)} m`);text('planner-count',`${simulation.plans} / ${simulation.completed}`);}
    const overlay=simulation?{planned:showPlan?simulation.path:[],movers:simulation.movers,goal:simulation.goal,predictions:simulation.predictions,heat:showPlan,selectedId:selectedActor?.id,selectedBox,freeCells:simulation.freeCells??[],cellSize:simulation.cellSize,margin:state.profile.inflation_hgp+state.profile.dyn_base_inflation_m+simulation.bodyRadius}:{goal:exploration?.goal??null};
    scenes.top.setData({points:state.mapPoints,pose:state.pose,trail:state.trail,target,placed:simulation?.placed??[],staticMargin:state.profile.inflation_hgp+.24,...overlay});
    scenes.main.setData({points:state.mapPoints,pose:state.pose,trail:state.trail,target,placed:simulation?.placed??[],staticMargin:state.profile.inflation_hgp+.24,...overlay});
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
  if(['A','B','C'].includes(settings.get('environment')))environmentId=settings.get('environment');
  function connectSource(source){
    releasePress('source changed');
    selectedObject=null;
    if(source==='demo')adapter.startDemo({environment:environmentId==='sandbox'?null:environmentId});
    else{adapter.connectLive();const control=settings.get('control');if(control&&!adapter.connectControl(control))toast('Invalid command supervisor URL');}
    setTopFollow(true);setMainFollow(true);
    if(source==='live'){scenes.main.setMode('perspective');}else applySimulationView(simulationView);
  }
  function applySimulationView(view){
    if(view==='pov'&&adapter.getSimulation()?.hidden)return;
    simulationView=view;
    if(view==='arena'){setMainFollow(false);scenes.main.setMode('top');scenes.main.fit();}
    else if(view==='map'){scenes.main.setMode('perspective');setMainFollow(true);}
    // A model notification updates visibility immediately, also while paused.
    store.tick(Date.now());
  }
  function editArena(point,objectPoint){
    if(store.getState().source!=='demo')return;
    if(adapter.getSimulation()?.hidden){sendGoal(point);return;}
    if(editorTool==='goal'&&objectPoint){
      const sim=adapter.getSimulation(),actor=sim?.movers.find(o=>o.axis&&Math.hypot(o.position[0]-objectPoint[0],o.position[1]-objectPoint[1])<.01);
      const box=sim?.placed.find(r=>objectPoint[0]>=r[0]&&objectPoint[0]<=r[2]&&objectPoint[1]>=r[1]&&objectPoint[1]<=r[3]);
      if(actor||box){releasePress('Object selected');selectedObject=actor?{kind:'moving',id:actor.id}:{kind:'static',box:[...box]};store.tick(Date.now());return;}
    }
    selectedObject=null;
    if(editorTool==='goal'){sendGoal(point);return;}
    releasePress('scene edited');
    const result=adapter.editSimulation(editorTool==='add'?'add':'remove',editorTool==='remove'&&objectPoint?objectPoint:point,$('obstacle-kind').value||'moving',{axis:$('movement-axis').value||'x'});
    toast(result.ok?(editorTool==='add'?'Obstacle added · route invalidated':'Obstacle removed · replanning'):result.reason);
    store.tick(Date.now());
  }
  for(const [id,tool]of [['tool-goal','goal'],['tool-add','add'],['tool-remove','remove']])on(id,'click',()=>{
    editorTool=tool;for(const [button,value]of [['tool-goal','goal'],['tool-add','add'],['tool-remove','remove']]){pressed(button,value===tool);$(button).classList.toggle('selected',value===tool);}
    text('editor-help',tool==='add'?'Click open space to add an obstacle. The drone replans immediately.':tool==='remove'?'Click a judge-added obstacle to remove it.':'Click open space to choose the drone’s destination.');
    applySimulationView('arena');
  });
  on('deselect-object','click',()=>{selectedObject=null;store.tick(Date.now());});
  for(const axis of ['x','y'])on('selected-motion-'+axis,'click',()=>{
    releasePress('Object motion changed');const result=adapter.editSimulation('motion',selectedObject,axis);
    if(result.ok){selectedObject={kind:'moving',id:result.id};toast(`Object moving along ${axis.toUpperCase()}`);}else toast(result.reason);
    store.tick(Date.now());
  });
  on('view-pov','click',()=>applySimulationView('pov'));on('view-arena','click',()=>applySimulationView('arena'));on('view-map','click',()=>applySimulationView('map'));
  on('clear-obstacles','click',()=>{const result=adapter.editSimulation('clear');toast(result.ok?'Added obstacles cleared':result.reason);store.tick(Date.now());});
  function sendGoal(point){
    if(store.getState().source!=='demo'){toast('Target selection is available in simulation mode');return;}
    releasePress('goal selected');const result=adapter.setSimulationGoal(point);
    if(result?.ok&&store.getState().explorationMode==='paused')store.resumeExploration();
    toast(result?.ok?'Destination set · planning around obstacles':result?.reason??'Simulation unavailable');
  }
  on('send-goal','click',()=>sendGoal(MISSIONS[Number($('mission-goal').value)]??MISSIONS[1]));
  on('show-plan','change',()=>{showPlan=$('show-plan').checked;store.tick(Date.now());});
  on('reset-lab','click',()=>{releasePress('simulation reset');adapter.startDemo({environment:environmentId==='sandbox'?null:environmentId});setTopFollow(true);applySimulationView(simulationView);toast('Simulation restarted');});
  on('environment','change',()=>{
    const value=$('environment').value;if(!['sandbox','A','B','C'].includes(value)||store.getState().source!=='demo')return;
    environmentId=value;selectedObject=null;editorTool='goal';simulationView='map';connectSource('demo');toast(value==='sandbox'?'Open sandbox ready':`Unknown ${value} ready · begin exploration to scan`);
  });
  on('begin-exploration','click',()=>{releasePress('Exploration started');const result=adapter.startExploration();if(!result.ok)toast(result.reason);store.tick(Date.now());});
  on('reset-survey','click',()=>{releasePress('Survey reset');adapter.startDemo({environment:environmentId==='sandbox'?null:environmentId});setTopFollow(true);applySimulationView('map');toast('Survey reset · map cleared');});
  for(const id of ['top-canvas','main-canvas','pov-canvas'])on(id,'keydown',event=>{if(event.key===' '){event.preventDefault();$('stop').click();}});
  on('live-tab','click',()=>changeView('live'));on('history-tab','click',()=>changeView('history'));
  on('open-history','click',()=>{changeView('history');$('history-tab').focus();});on('return-live','click',()=>{changeView('live');$('live-tab').focus();});
  on('source','change',()=>connectSource($('source').value));
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
  return Object.freeze({store,adapter,scenes,camera,destroy(){releasePress('dashboard destroyed');unsubscribe();adapter.dispose();camera.destroy();pov.destroy();clearTimeout(toastTimer);for(const remove of removers)remove();for(const scene of Object.values(scenes))scene.destroy();expanded?.classList.remove('expanded');}});
}

if (typeof document !== 'undefined' && document.getElementById('main-canvas')) mountDashboard();
