import {createMissionLog} from './mission-log.mjs';
import {MissionStore} from './model.mjs';
import {createAdapter} from './adapter.mjs';
import {createScene} from './scene.mjs';
import {mountMissionMap} from './mission-map.mjs';

export function mountRoom({document:doc=globalThis.document,autoStart=true,recordingStorage}={}){
  const $=id=>doc.getElementById(id),store=new MissionStore(),adapter=createAdapter({store,room:true});
  const recorder=createMissionLog({storage:recordingStorage});
  const removers=[],win=doc.defaultView??globalThis;
  let reviewing=false;
  let tool='goal',heat=true,tracking=true,active=null,renewal=null,toastTimer=null,previous=null,traveled=0,selectedObject=null;
  function on(el,type,fn){if(typeof el==='string')el=$(el);el.addEventListener(type,fn);removers.push(()=>el.removeEventListener(type,fn));}
  function text(id,value){$(id).textContent=String(value);}
  function toast(message){text('room-toast',message);$('room-toast').hidden=false;clearTimeout(toastTimer);toastTimer=setTimeout(()=>{$('room-toast').hidden=true;},3000);}
  const scene=createScene($('room-canvas'),{mode:'perspective',follow:false,onMapClick:(point,objectPoint)=>clickRoom(point,objectPoint)});
  const review=mountMissionMap({document:doc,dataSource:()=>recorder.getSnapshot(),onReturn:()=>setWorkspaceView(false)});
  function setWorkspaceView(showReview,writeHistory=true){
    release('Workspace view changed');recorder.flush();reviewing=showReview;
    $('flight-room-screen').hidden=showReview;$('mission-review-screen').hidden=!showReview;
    $('flight-room-view').classList.toggle('active',!showReview);$('flight-room-view').setAttribute('aria-pressed',String(!showReview));
    $('mission-map-link').classList.toggle('active',showReview);$('mission-map-link').setAttribute('aria-current',showReview?'page':'false');
    if(showReview){review.refresh();review.scene.fit();}else scene.fit();
    if(writeHistory)win.history?.pushState(null,'',showReview?'#mission-map':'#flight-room');
  }
  function release(reason='Push released'){
    if(active){active.button.classList.remove('held');active.button.setAttribute('aria-pressed','false');}
    active=null;clearInterval(renewal);renewal=null;adapter.releaseSteering(reason);
  }
  function begin(direction,button,kind,id){
    if(reviewing||active||doc.hidden)return;
    if(!adapter.getSimulation()?.started){toast('Click Start before pushing the drone');return;}
    if(store.getState().explorationMode==='paused')adapter.resumeExploration();
    const result=adapter.startSteering(direction);if(!result.ok){toast(result.reason);return;}
    active={direction,button,kind,id};button.classList.add('held');button.setAttribute('aria-pressed','true');
    renewal=setInterval(()=>{if(doc.hidden||!adapter.renewSteering().ok)release('Push ended');},100);
  }
  function clickRoom(point,objectPoint){
    if(tool!=='remove'&&objectPoint){
      release('Object selected');const sim=adapter.getSimulation();
      const actor=sim?.movers.find(o=>Math.hypot(o.position[0]-objectPoint[0],o.position[1]-objectPoint[1])<.01);
      const box=sim?.placed.find(r=>objectPoint[0]>=r[0]&&objectPoint[0]<=r[2]&&objectPoint[1]>=r[1]&&objectPoint[1]<=r[3]);
      if(actor||box){selectedObject=actor?{kind:'moving',id:actor.id}:{kind:'static',box:[...box]};store.tick(Date.now());return;}
    }
    selectedObject=null;place(tool==='remove'&&objectPoint?objectPoint:point);
  }
  function place(point){
    release('Room edited');
    let result;
    if(tool==='goal'){
      const goal=[point[0],point[1],Number($('goal-height').value)||1.8];
      result=adapter.setSimulationGoal(goal);
      if(result?.ok){if(adapter.getSimulation()?.started&&store.getState().explorationMode==='paused')store.resumeExploration();toast(adapter.getSimulation()?.started?'Destination set · planning flight':'Destination ready · click Start');}
    }else{
      result=adapter.editSimulation(tool==='remove'?'remove':'add',point,tool==='moving'?'moving':'crate',{axis:$('movement-axis').value||'x'});
      if(result.ok)toast(tool==='remove'?'Object removed · replanning':tool==='moving'?'Moving object added · predicting its path':'Static object added · replanning');
    }
    if(!result?.ok)toast(result?.reason??'Start the room first');
    store.tick(Date.now());
  }
  const unsubscribe=store.subscribe(state=>{
    const sim=adapter.getSimulation(),p=state.pose?.position;
    if(!p){previous=null;traveled=0;}else{if(previous)traveled+=Math.hypot(...p.map((v,i)=>v-previous[i]));previous=[...p];}
    const selectedActor=selectedObject?.kind==='moving'?sim?.movers.find(o=>o.id===selectedObject.id):null;
    const selectedBox=selectedObject?.kind==='static'?sim?.placed.find(r=>r.every((v,i)=>v===selectedObject.box[i])):null;
    if(!selectedActor&&!selectedBox)selectedObject=null;
    $('selected-object').hidden=!selectedObject;
    text('selected-object-label',selectedActor?`Selected ${selectedActor.id}`:'Selected static object');
    for(const axis of ['x','y']){const button=$('selected-motion-'+axis);button.classList.toggle('selected',selectedActor?.axis===axis);button.setAttribute('aria-pressed',String(selectedActor?.axis===axis));}
    scene.setData({room:true,bounds:sim?.bounds,fixtures:sim?.fixtures??[],bodyRadius:sim?.bodyRadius??.62,selectedId:selectedActor?.id,selectedBox,points:state.mapPoints,pose:state.pose,trail:tracking?state.trail:[],planned:tracking?sim?.path??[]:[],goal:sim?.goal??null,placed:sim?.placed??[],movers:sim?.movers??[],predictions:sim?.predictions??[],obstacles:sim?.obstacles??[],heat,staticMargin:state.profile.inflation_hgp+(sim?.bodyRadius??.62),margin:state.profile.inflation_hgp+state.profile.dyn_base_inflation_m+(sim?.bodyRadius??.62)});
    text('room-status',sim?.status??'Preparing room');
    text('room-position',p?`X ${p[0].toFixed(1)} · Y ${p[1].toFixed(1)} · Z ${p[2].toFixed(1)} m`:'X — · Y — · Z —');
    text('room-speed',`${(state.speed??0).toFixed(2)} m/s`);
    text('room-clearance',sim?`${sim.clearance.toFixed(2)} m`:'—');
    text('room-distance',sim?.goal&&p?`${Math.hypot(...sim.goal.map((v,i)=>v-p[i])).toFixed(1)} m`:'Choose a goal');
    text('room-traveled',`${traveled.toFixed(1)} m`);text('room-plans',sim?`${sim.plans} / ${sim.completed}`:'0 / 0');
    recorder.capture(sim,state,traveled);
    text('recording-state',recorder.isPersistent()?'Recording locally':'History storage unavailable');
    text('room-object-count',`${sim?.objectCount??0} / 24`);
    const started=sim?.started===true;$('start-room').hidden=started;$('pause-room').disabled=!started;
    const paused=state.explorationMode==='paused';text('pause-room',paused?'Resume':'Pause');$('pause-room').setAttribute('aria-pressed',String(paused));
    for(const direction of ['forward','backward','left','right','up','down'])$('push-'+direction).disabled=!started||!p||state.stale.pose||state.stale.map;
    if(active&&(state.stale.pose||state.stale.map||!state.steering))release('Push unavailable');
  });
  const hints={goal:'Click the room floor to choose a destination',static:'Click the floor to add a static object',moving:'Click the floor to add a moving object',remove:'Click an added object’s footprint to remove it'};
  function updateHint(){text('tool-hint',tool==='moving'?`Click the floor to add an object moving ${$('movement-axis').value==='y'?'up–down (room Y)':'left–right (room X)'}`:hints[tool]);}
  on('movement-axis','change',updateHint);
  on('deselect-object','click',()=>{selectedObject=null;store.tick(Date.now());});
  for(const axis of ['x','y'])on('selected-motion-'+axis,'click',()=>{
    release('Object direction changed');const result=adapter.editSimulation('motion',selectedObject,axis);
    if(result.ok){selectedObject={kind:'moving',id:result.id};toast(`Object movement set to room ${axis.toUpperCase()}`);}else toast(result.reason);
    store.tick(Date.now());
  });
  for(const value of ['goal','static','moving','remove'])on('room-'+value,'click',()=>{
    release('Tool changed');tool=value;for(const t of ['goal','static','moving','remove']){$('room-'+t).classList.toggle('selected',t===tool);$('room-'+t).setAttribute('aria-pressed',String(t===tool));}$('movement-settings').hidden=tool!=='moving';updateHint();
  });
  on('goal-height','input',()=>text('goal-height-value',`${Number($('goal-height').value).toFixed(1)} m`));
  on('room-heat','change',()=>{heat=$('room-heat').checked;store.tick(Date.now());});
  on('room-trail','change',()=>{tracking=$('room-trail').checked;store.tick(Date.now());});
  on('clear-room','click',()=>{release();adapter.editSimulation('clear');store.tick(Date.now());toast('Room cleared');});
  on('reset-room','click',()=>{release();previous=null;traveled=0;recorder.reset();adapter.startDemo();scene.fit();toast('Furnished room restarted');});
  on('fit-room','click',()=>scene.fit());
  on('start-room','click',()=>{release();const result=adapter.startSimulation();if(!result.ok)toast(result.reason);});
  function pause(){if(!adapter.getSimulation()?.started)return;release();if(store.getState().explorationMode==='paused')adapter.resumeExploration();else adapter.pauseExploration();}
  on('pause-room','click',pause);
  for(const direction of ['forward','backward','left','right','up','down']){
    const button=$('push-'+direction);button.setAttribute('aria-pressed','false');
    on(button,'pointerdown',event=>{if(event.isPrimary===false||event.button!==undefined&&event.button!==0)return;event.preventDefault();begin(direction,button,'pointer',event.pointerId);if(active)button.setPointerCapture?.(event.pointerId);});
    for(const type of ['pointerup','pointercancel','lostpointercapture'])on(button,type,event=>{if(active?.kind==='pointer'&&active.id===event.pointerId)release();});
    on(button,'keydown',event=>{if([' ','Enter'].includes(event.key)){event.preventDefault();if(!event.repeat)begin(direction,button,'button',event.key);}});
    on(button,'keyup',event=>{if(active?.kind==='button'&&active.id===event.key){event.preventDefault();release();}});
    on(button,'blur',()=>{if(active?.button===button)release('Focus lost');});
  }
  const keys={w:'forward',s:'backward',a:'left',d:'right',q:'down',e:'up'};
  on(doc,'keydown',event=>{
    if(reviewing)return;
    if(['INPUT','SELECT','TEXTAREA','BUTTON','SUMMARY'].includes(event.target?.tagName))return;
    if(event.key===' '){event.preventDefault();if(!event.repeat)pause();return;}
    const direction=keys[event.key?.toLowerCase()];if(direction){event.preventDefault();if(!event.repeat)begin(direction,$('push-'+direction),'key',event.key.toLowerCase());}
  });
  on(doc,'keyup',event=>{if(active?.kind==='key'&&active.id===event.key?.toLowerCase()){event.preventDefault();release();}});
  on(doc,'visibilitychange',()=>{if(doc.hidden)release('Page hidden');});
  on('mission-map-link','click',event=>{event.preventDefault();setWorkspaceView(true);});
  on('flight-room-view','click',()=>setWorkspaceView(false));
  on(win,'popstate',()=>setWorkspaceView(win.location?.hash==='#mission-map',false));
  on(win,'pagehide',()=>recorder.flush());
  for(const type of ['blur','pagehide'])on(win,type,()=>release('Window inactive'));
  on(win,'pointerup',event=>{if(active?.kind==='pointer'&&active.id===event.pointerId)release();});
  if(autoStart)adapter.startDemo();
  setWorkspaceView(win.location?.hash==='#mission-map',false);
  return Object.freeze({store,adapter,scene,recorder,review,destroy(){release();unsubscribe();recorder.flush();adapter.dispose();scene.destroy();review.destroy();clearTimeout(toastTimer);for(const remove of removers)remove();}});
}
if(typeof document!=='undefined'&&document.getElementById('room-canvas')){
  if(new URLSearchParams(globalThis.location?.search??'').get('source')==='live')globalThis.location.replace('/dashboard.html'+globalThis.location.search);
  else mountRoom();
}
