import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import {mountDashboard} from '../slam_dunk_bridge/web/app.mjs';
import {createScene} from '../slam_dunk_bridge/web/scene.mjs';
import {createCameraFeed} from '../slam_dunk_bridge/web/camera.mjs';

// A deliberately small DOM/canvas test double, not a browser-layout assertion.
// These scenarios execute the real app event handlers, model, demo and adapter.
class Element {
  constructor(id='') {
    this.id=id; this.handlers=new Map(); this.attributes=new Map(); this.dataset={};
    this.value=''; this.hidden=false; this.disabled=false; this.style={}; this.children=[];
    this.classes=new Set(); this.text=''; this.parentElement=null;
    this.classList={toggle:(name,on)=>{const yes=on??!this.classes.has(name);yes?this.classes.add(name):this.classes.delete(name);return yes;},add:name=>this.classes.add(name),remove:name=>this.classes.delete(name),contains:name=>this.classes.has(name)};
  }
  get textContent(){return this.text+this.children.map(e=>e.textContent).join('');}
  set textContent(value){this.text=String(value);this.children=[];}
  addEventListener(type,fn){if(!this.handlers.has(type))this.handlers.set(type,new Set());this.handlers.get(type).add(fn);}
  removeEventListener(type,fn){this.handlers.get(type)?.delete(fn);}
  fire(type,props={}){if(type==='click'&&this.disabled)return;const event={type,target:this,key:'',preventDefault(){this.prevented=true;},...props};for(const fn of this.handlers.get(type)??[])fn(event);return event;}
  click(){this.fire('click');}
  setAttribute(name,value){this.attributes.set(name,String(value));}
  getAttribute(name){return this.attributes.get(name)??null;}
  removeAttribute(name){this.attributes.delete(name);if(name==='src')this.src='';}
  appendChild(child){this.children.push(child);child.parentElement=this;return child;}
  replaceChildren(...children){this.text='';this.children=[];children.forEach(c=>this.appendChild(c));}
  focus(){this.focused=true;}
  getBoundingClientRect(){return {width:this.boxWidth??640,height:this.boxHeight??420};}
  setPointerCapture(){}
  getContext(){return this.context;}
}

function canvasContext() {
  const metrics={calls:0,points:0};
  const context=new Proxy({}, {get(target,key){
    if(key==='metrics')return metrics;
    if(!(key in target))target[key]=(...args)=>{metrics.calls++;if(key==='fillRect')metrics.points++;for(const a of args)if(typeof a==='number')assert.ok(Number.isFinite(a),`Nonfinite canvas ${String(key)}`);};
    return target[key];
  },set(target,key,value){target[key]=value;return true;}});
  return context;
}

function markup() {
  const html=readFileSync(new URL('../slam_dunk_bridge/web/index.html',import.meta.url),'utf8');
  const elements=new Map();
  for(const m of html.matchAll(/<[^>]+\bid="([^"]+)"[^>]*>/g)){
    const e=new Element(m[1]);e.parentElement=new Element();e.hidden=/\bhidden\b/.test(m[0]);
    const value=m[0].match(/\bvalue="([^"]*)"/);if(value)e.value=value[1];
    if(m[0].startsWith('<canvas'))e.context=canvasContext();elements.set(e.id,e);
  }
  elements.get('source').value='demo';
  const directions=[...html.matchAll(/data-direction="([^"]+)"/g)].map(m=>{const e=new Element();e.dataset.direction=m[1];return e;});
  const doc=new Element('document');doc.elements=elements;doc.directions=directions;
  doc.defaultView=new Element('window');
  doc.getElementById=id=>elements.get(id)??null;
  doc.querySelectorAll=selector=>selector==='[data-direction]'?directions:[];
  doc.createElement=tag=>new Element(tag);doc.createTextNode=text=>{const e=new Element();e.textContent=text;return e;};
  return doc;
}

function environment(seed) {
  const previous=new Map(),raf=new Map(),intervals=new Map(),timeouts=new Map();let id=0,time=1700000000000+seed*10000;
  const set=(key,value)=>{previous.set(key,{own:Object.hasOwn(globalThis,key),value:globalThis[key]});globalThis[key]=value;};
  class Socket {
    static instances=[];
    constructor(url){this.url=url;this.sent=[];this.readyState=0;Socket.instances.push(this);}
    close(){this.readyState=3;this.onclose?.();}
    send(value){if(!this.url.endsWith('/control'))throw new Error('Read-only stream attempted outbound control');this.sent.push(JSON.parse(value));}
    open(){this.readyState=1;this.onopen?.();}
    message(data){this.onmessage?.({data:typeof data==='string'?data:JSON.stringify(data)});}
  }
  set('requestAnimationFrame',fn=>{raf.set(++id,fn);return id;});set('cancelAnimationFrame',n=>raf.delete(n));
  set('setInterval',fn=>{intervals.set(++id,fn);return id;});set('clearInterval',n=>intervals.delete(n));
  set('setTimeout',fn=>{timeouts.set(++id,fn);return id;});set('clearTimeout',n=>timeouts.delete(n));
  set('ResizeObserver',class {observe(){}disconnect(){}});set('WebSocket',Socket);
  set('location',new URL('http://127.0.0.1:8080/'));set('devicePixelRatio',1+(seed%3)/2);
  const realNow=Date.now;Date.now=()=>time;
  const flush=()=>{const next=[...raf.values()];raf.clear();next.forEach(fn=>fn(time));};
  return {Socket,raf,intervals,flush,advance(ms){time+=ms;[...intervals.values()].forEach(fn=>fn());flush();},restore(){Date.now=realNow;for(const [key,p]of previous)p.own?globalThis[key]=p.value:delete globalThis[key];}};
}

export function runIntegrationTests() {
  const counts={fullDashboardScenarios:0,holdReleaseScenarios:0,cameraLifecycleScenarios:0,sceneLifecycleScenarios:0,sceneInvalidConstructionCases:0};
  for(let seed=0;seed<300;seed++) {
    const env=environment(seed),doc=markup(),get=id=>doc.getElementById(id);
    let app;
    try {
      app=mountDashboard({document:doc,autoStart:true});env.flush();
      assert.equal(app.store.getState().source,'demo');assert.ok(app.store.getState().mapPoints.length>0);
      assert.ok(get('detail-source').textContent.includes('Synthetic'));assert.equal(get('history-screen').hidden,true);
      for(let step=0;step<12;step++)env.advance(100+seed%17);
      assert.ok(get('main-canvas').context.metrics.points>0);assert.equal(get('site-canvas'),null);
      assert.equal(get('camera-image').hidden,true);assert.ok(get('camera-note').textContent.includes('No RGB'));
      assert.ok(get('battery').textContent.endsWith('%'));assert.notEqual(get('flight-time').textContent,'—');
      const margin=seed%101;get('margin').value=String(margin);get('margin').fire('input');
      assert.equal(app.store.getState().profile.value,margin);
      assert.equal(get('static-margin').textContent,`${(.45+.45*margin/100).toFixed(2)} m`);
      assert.equal(get('dynamic-margin').textContent,`${(.1+.2*margin/100).toFixed(2)} m`);
      get(seed%2?'speed-standard':'speed-slow').click();assert.equal(app.store.getState().speedMode,seed%2?'standard':'precision');
      const direction=doc.directions[seed%doc.directions.length];direction.fire('pointerdown',{pointerId:seed+1,button:0});
      assert.equal(app.store.getState().explorationMode,'steering');assert.equal(direction.getAttribute('aria-pressed'),'true');
      env.advance(200);direction.fire('pointerup',{pointerId:seed+1});assert.equal(app.store.getState().explorationMode,'autonomous');
      get('stop').click();assert.equal(app.store.getState().explorationMode,'paused');const held=[...app.store.getState().pose.position];env.advance(200);assert.deepEqual(app.store.getState().pose.position,held);
      get('stop').click();assert.equal(app.store.getState().explorationMode,'autonomous');
      get('top-follow').click();assert.equal(get('top-follow').getAttribute('aria-pressed'),'false');get('top-follow').click();
      get('main-fit').click();assert.equal(get('main-follow').getAttribute('aria-pressed'),'false');get('main-follow').click();
      get('main-canvas').fire('keydown',{key:'0'});assert.equal(get('main-follow').getAttribute('aria-pressed'),'false');
      env.advance(100);assert.equal(get('main-follow').getAttribute('aria-pressed'),'false');get('main-follow').click();
      get('open-history').click();assert.equal(app.store.getState().view,'history');assert.equal(get('live-screen').hidden,true);
      assert.ok(doc.directions.every(b=>b.disabled));
      const frames=app.store.getState().history;get('timeline').value=String(frames[0].timestamp);get('timeline').fire('input');
      assert.equal(app.store.getDisplay().timestamp,frames[0].timestamp);assert.ok(get('history-chip').textContent.includes('HISTORICAL'));
      get('history-3d').click();assert.equal(get('history-3d').getAttribute('aria-pressed'),'true');get('history-2d').click();get('history-fit').click();
      get('playback').click();assert.equal(app.store.getState().playing,true);env.advance(100);get('playback').click();assert.equal(app.store.getState().playing,false);
      get('history-fullscreen').click();assert.ok(get('history-viewer-panel').classList.contains('expanded'));doc.fire('keydown',{key:'Escape'});assert.ok(!get('history-viewer-panel').classList.contains('expanded'));
      get('return-live').click();assert.equal(app.store.getState().view,'live');
      get('main-fullscreen').click();assert.ok(get('main-viewer-panel').classList.contains('expanded'));get('main-fullscreen').click();assert.ok(!get('main-viewer-panel').classList.contains('expanded'));
      get('source').value='live';get('source').fire('change');
      assert.equal(app.store.getState().source,'live');assert.equal(app.store.getState().mapPoints.length,0);assert.ok(doc.directions.every(b=>b.disabled));assert.equal(get('stop').disabled,true);
      const socket=env.Socket.instances.at(-1);socket.open();
      const stamp={sec:1000+seed,nanosec:0};
      socket.message({protocol_version:1,type:'pose',frame_id:'map',stamp,position:[seed/10,-2,1],orientation:[0,0,0,1]});
      socket.message({protocol_version:1,type:'map',frame_id:'map',stamp,operation:'replace',revision:1,points:[[0,0,0],[1,1,2]]});
      assert.equal(get('battery').textContent,'97%');assert.equal(get('speed').textContent,'—');assert.equal(get('flight-time').textContent,'—');
      assert.ok(get('profile-status').textContent.includes('proposal'));assert.equal(app.store.getState().connection,'connected');
      socket.message('{invalid');assert.ok(app.store.getState().rejected>0);
      socket.message({protocol_version:1,type:'map',frame_id:'map',stamp:{sec:stamp.sec+1,nanosec:0},operation:'replace',revision:2,points:[]});
      assert.equal(app.store.getState().mapPoints.length,0);assert.equal(get('empty-state').hidden,false);assert.ok(app.store.getState().combinedPoints.length>0);
      env.advance(3100);assert.equal(app.store.getState().stale.pose,true);socket.close();assert.equal(app.store.getState().connection,'disconnected');
      get('source').value='demo';get('source').fire('change');env.flush();assert.equal(app.store.getState().source,'demo');assert.ok(app.store.getState().mapPoints.length>0);
      // Detached callbacks from the old source must not overwrite the new demo.
      socket.message({protocol_version:1,type:'pose',frame_id:'evil',stamp:{sec:999999,nanosec:0},position:[0,0,0],orientation:[0,0,0,1]});
      assert.equal(app.store.getState().frameId,'map');
      app.destroy();app=null;assert.equal(env.raf.size,0);assert.equal(env.intervals.size,0);
      assert.ok([...doc.elements.values(),...doc.directions,doc,doc.defaultView].every(e=>[...e.handlers.values()].every(set=>set.size===0)));
      counts.fullDashboardScenarios++;
    } finally {app?.destroy();env.restore();}
  }
  for(let seed=0;seed<300;seed++) {
    const env=environment(seed),doc=markup(),button=doc.directions[seed%4];let app;
    try{
      app=mountDashboard({document:doc});env.advance(200);
      const keyboard=seed%8===7;
      if(keyboard)button.fire('keydown',{key:seed%2?' ':'Enter',repeat:false});
      else button.fire('pointerdown',{pointerId:seed+1,button:0});
      assert.equal(app.store.getState().explorationMode,'steering');
      env.advance(150);
      switch(seed%8){
        case 0:button.fire('pointerup',{pointerId:seed+1});break;
        case 1:button.fire('pointercancel',{pointerId:seed+1});break;
        case 2:button.fire('lostpointercapture',{pointerId:seed+1});break;
        case 3:doc.hidden=true;doc.fire('visibilitychange');break;
        case 4:doc.defaultView.fire('blur');break;
        case 5:doc.defaultView.fire('pagehide');break;
        case 6:doc.getElementById('history-tab').click();break;
        case 7:button.fire('keyup',{key:seed%2?' ':'Enter'});break;
      }
      assert.equal(app.store.getState().explorationMode,'autonomous');
      assert.equal(button.getAttribute('aria-pressed'),'false');
      // Releasing twice cannot reacquire or leave a renewal interval running.
      button.fire('pointerup',{pointerId:seed+1});env.advance(200);
      assert.equal(app.store.getState().explorationMode,'autonomous');
      counts.holdReleaseScenarios++;
    }finally{app?.destroy();assert.equal(env.intervals.size,0);env.restore();}
  }
  for(let seed=0;seed<300;seed++) {
    const image=new Element(),empty=new Element(),status=new Element(),note=new Element();
    const camera=createCameraFeed({image,empty,status,note,baseUrl:'http://127.0.0.1:8080/'});
    assert.equal(camera.connect(`http://127.0.0.1:${8081+seed}/stream?topic=%2FNX01%2Fd435%2Fcolor%2Fimage_raw`),true);
    assert.equal(image.hidden,false);assert.equal(empty.hidden,true);assert.ok(note.textContent.includes('freshness unavailable'));
    const obsoleteLoad=image.onload,obsoleteError=image.onerror;
    image.onload();assert.equal(status.textContent,'STREAM RECEIVED');
    assert.equal(camera.setVisible(false),true);assert.equal(image.src,'');assert.equal(image.hidden,true);
    obsoleteError();assert.equal(status.textContent,'FEED SUSPENDED');
    camera.setVisible(true);image.onerror();assert.equal(status.textContent,'CAMERA UNAVAILABLE');assert.equal(empty.hidden,false);
    camera.connect('/stream?topic=camera');obsoleteLoad();assert.equal(status.textContent,'STREAM CONFIGURED');
    for(const invalid of ['javascript:alert(1)','file:///tmp/a','http://user:password@robot/feed','http://robot/feed#fragment','',null,'https://['])assert.equal(camera.connect(invalid),false);
    assert.equal(camera.disconnect(),true);assert.equal(status.textContent,'NOT CONNECTED');
    camera.destroy();assert.equal(camera.connect('http://robot/feed'),false);assert.equal(camera.setVisible(true),false);
    counts.cameraLifecycleScenarios++;
  }
  for(let seed=0;seed<300;seed++) {
    const env=environment(seed),canvas=new Element();canvas.context=canvasContext();canvas.boxWidth=200+seed;canvas.boxHeight=150+seed%100;
    let scene;
    try {
      scene=createScene(canvas,{mode:seed%2?'top':'perspective',local:true,follow:true});
      const points=Array.from({length:30},(_,i)=>[Math.sin(i+seed)*8,Math.cos(i+seed)*7,(i%10)*.3]);
      const pose={position:[seed%5,seed%7,1.8],orientation:[0,0,Math.sin(seed),Math.cos(seed)]};
      assert.equal(scene.setData({points,pose,trail:points.slice(0,8),target:[0,0,2]}),true);env.flush();
      assert.ok(canvas.context.metrics.calls>0);scene.fit();assert.equal(scene.setMode(seed%2?'perspective':'top'),true);
      assert.equal(scene.setFollow(false),true);assert.equal(scene.setFollow(true),true);assert.equal(scene.setFollow('true'),false);assert.equal(scene.setMode('other'),false);assert.equal(scene.setData(null),false);
      for(const key of ['ArrowLeft','ArrowRight','ArrowUp','ArrowDown','+','-','=','0'])assert.equal(canvas.fire('keydown',{key}).prevented,true);
      for(const deltaY of [-1e6,-2,0,2,1e6])canvas.fire('wheel',{deltaY});
      canvas.fire('pointerdown',{clientX:1,clientY:2,pointerId:1,shiftKey:!!(seed%2)});canvas.fire('pointermove',{clientX:10,clientY:20});canvas.fire('pointerup');canvas.fire('pointercancel');canvas.fire('lostpointercapture');
      env.flush();assert.equal(scene.setData({points:[],pose:null,trail:[],target:null}),true);scene.fit();env.flush();
      scene.destroy();assert.equal(scene.setData({points:[],trail:[]}),false);scene.destroy();scene=null;
      assert.equal(env.raf.size,0);assert.ok([...canvas.handlers.values()].every(s=>s.size===0));counts.sceneLifecycleScenarios++;
    } finally {scene?.destroy();env.restore();}
  }
  for(let i=0;i<300;i++){assert.throws(()=>createScene(i%2?{}:null),TypeError);counts.sceneInvalidConstructionCases++;}
  return counts;
}

if(import.meta.url===new URL(process.argv[1],'file:///').href)console.log(JSON.stringify(runIntegrationTests(),null,2));

export function runPinInteractionTests(){
  const results=[];
  for(const canvasId of ['top-canvas','main-canvas']){
    const env=environment(0),doc=markup(),get=id=>doc.getElementById(id);let app;
    try{
      app=mountDashboard({document:doc,autoStart:false});
      app.adapter.connectLive('ws://localhost:8765/stream');const telemetry=env.Socket.instances.at(-1);telemetry.open();
      const stamp={sec:1700000000,nanosec:0};
      telemetry.message({type:'pose',protocol_version:1,frame_id:'gazebo_world',stamp,position:[0,0,2],orientation:[0,0,0,1]});
      telemetry.message({type:'map',protocol_version:1,frame_id:'gazebo_world',stamp,operation:'replace',revision:1,points:[[3,3,2]]});
      app.adapter.connectControl('ws://localhost:8766/control');const control=env.Socket.instances.at(-1);control.open();
      control.message({type:'control_status',protocol_version:1,epoch:0,mode:'waiting',ready:false,navigation_ready:true,lease_id:null,reason:'Preparing flight',ttl_ms:500,capabilities:{steering:true,pause:false}});
      env.flush();assert.equal(get('pin-mode').disabled,false);
      get('pin-mode').click();const canvas=get(canvasId);
      canvas.fire('pointerdown',{clientX:320,clientY:210,pointerId:1});
      canvas.fire('pointermove',{clientX:322,clientY:211,pointerId:1});
      canvas.fire('pointerup',{clientX:320,clientY:210,pointerId:1});
      assert.ok(get('pin-status').textContent.includes('waiting for PX4 controller acceptance'));
      // Placement sends immediately; no second button click is required.
      assert.equal(control.sent.filter(m=>m.type==='navigate').length,1);
      const command=control.sent.at(-1);assert.equal(command.type,'navigate');
      assert.ok(command.position.every(v=>Math.abs(v)<1e-8));
      control.message({type:'control_status',protocol_version:1,epoch:1,mode:'waiting',ready:false,navigation_ready:true,lease_id:null,reason:'Pinned destination accepted',last_request_id:command.request_id,accepted:true,ttl_ms:500,capabilities:{steering:true,pause:false}});
      assert.ok(get('pin-status').textContent.includes('Accepted:'));
      assert.equal(get('battery').textContent,'97%');assert.equal(get('battery-fill').style.width,'97%');
      results.push(canvasId);
    }finally{app?.destroy();env.restore();}
  }
  return results;
}

export async function runQueuedPinTest(){
  const env=environment(0),doc=markup(),get=id=>doc.getElementById(id);let app;
  try{
    app=mountDashboard({document:doc,autoStart:false});app.adapter.connectLive('ws://localhost:8765/stream');
    const telemetry=env.Socket.instances.at(-1);telemetry.open();const stamp={sec:1700000000,nanosec:0};
    telemetry.message({type:'pose',protocol_version:1,frame_id:'gazebo_world',stamp,position:[0,0,2],orientation:[0,0,0,1]});
    telemetry.message({type:'map',protocol_version:1,frame_id:'gazebo_world',stamp,operation:'replace',revision:1,points:[[3,3,2]]});
    app.adapter.connectControl('ws://localhost:8765/control');const control=env.Socket.instances.at(-1);control.open();
    const status={type:'control_status',protocol_version:1,epoch:0,mode:'waiting',ready:false,navigation_ready:false,lease_id:null,reason:'Preparing',ttl_ms:500,capabilities:{steering:true,pause:false}};
    control.message(status);env.flush();get('pin-mode').click();
    get('top-canvas').fire('pointerdown',{clientX:320,clientY:210,pointerId:1});
    get('top-canvas').fire('pointerup',{clientX:320,clientY:210,pointerId:1});
    assert.equal(control.sent.length,0);assert.ok(get('pin-status').textContent.includes('queued'));
    control.message({...status,navigation_ready:true});await Promise.resolve();
    assert.equal(control.sent.length,1);assert.equal(control.sent[0].type,'navigate');
    control.message({...status,navigation_ready:true});await Promise.resolve();assert.equal(control.sent.length,1);
    const first=control.sent[0];
    get('pin-mode').click();get('top-canvas').fire('pointerdown',{clientX:340,clientY:210,pointerId:2});
    get('top-canvas').fire('pointerup',{clientX:340,clientY:210,pointerId:2});
    assert.equal(control.sent.length,1,'second pin waits for first acknowledgement');
    control.message({...status,epoch:1,navigation_ready:true,last_request_id:first.request_id,accepted:true,destination:[0,0],reason:'Pinned destination accepted'});
    env.advance(1100);await Promise.resolve();
    assert.equal(control.sent.length,2);assert.equal(control.sent[1].epoch,1,'latest pin uses acknowledged epoch');
    assert.ok(control.sent[1].position[0]>.5,'latest selected point wins');
    const second=control.sent[1];
    control.message({...status,epoch:2,navigation_ready:true,last_request_id:second.request_id,accepted:true,destination:[1.2,0],reason:'Pin adjusted to nearby clear destination'});
    assert.ok(get('pin-status').textContent.includes('adjusted'));
    assert.deepEqual(app.store.getState().navigationAck.destination,[1.2,0]);
    get('pin-mode').click();get('top-canvas').fire('pointerdown',{clientX:350,clientY:210,pointerId:3});get('top-canvas').fire('pointerup',{clientX:350,clientY:210,pointerId:3});
    const third=control.sent.at(-1);assert.equal(control.sent.length,3);
    control.message({...status,epoch:3,navigation_ready:true,last_request_id:third.request_id,accepted:false,retryable:true,reason:'Destination requires fresh navigation'});
    env.advance(1100);await Promise.resolve();assert.equal(control.sent.length,4);assert.equal(control.sent.at(-1).epoch,3);
    return 'Queued and rapid pins, adjusted destinations and transient rejection retry';
  }finally{app?.destroy();env.restore();}
}

export function runMissionButtonTests(){
  const env=environment(0),doc=markup(),get=id=>doc.getElementById(id);let app;
  try{
    app=mountDashboard({document:doc,autoStart:false});app.adapter.connectLive('ws://localhost:8765/stream');
    const telemetry=env.Socket.instances.at(-1);telemetry.open();const stamp={sec:1700000000,nanosec:0};
    telemetry.message({type:'pose',protocol_version:1,frame_id:'gazebo_world',stamp,position:[0,0,2],orientation:[0,0,0,1],velocity:[3,4,0]});
    telemetry.message({type:'map',protocol_version:1,frame_id:'gazebo_world',stamp,operation:'replace',revision:1,points:[[3,3,2]]});
    app.adapter.connectControl('ws://localhost:8765/control');const control=env.Socket.instances.at(-1);control.open();
    const status={type:'control_status',protocol_version:1,epoch:0,mode:'waiting',ready:false,navigation_ready:true,start_ready:true,return_ready:false,mission_phase:'PREFLIGHT',lease_id:null,reason:'Standby',ttl_ms:500,capabilities:{steering:true,pause:false}};
    control.message(status);env.flush();assert.equal(control.sent.length,0);assert.equal(get('mission-start').disabled,false);assert.equal(get('mission-home').disabled,true);
    assert.equal(get('speed').textContent,'5.00');
    get('mission-start').click();const start=control.sent.at(-1);assert.equal(start.type,'start');assert.equal(get('mission-start').disabled,true);
    control.message({...status,epoch:1,mode:'autonomous',ready:true,start_ready:false,return_ready:true,mission_phase:'NAVIGATE',last_request_id:start.request_id,accepted:true,reason:'Start accepted'});
    assert.equal(get('mission-home').disabled,false);get('mission-home').click();const home=control.sent.at(-1);assert.equal(home.type,'return_home');assert.equal(home.epoch,1);
    control.message({...status,epoch:2,mode:'autonomous',ready:true,start_ready:false,return_ready:true,last_request_id:home.request_id,accepted:true,reason:'Returning home and landing'});
    assert.ok(get('mission-status').textContent.includes('Returning home'));
    return 'Standby Start, Return Home, acknowledgements and measured 5.00 m/s';
  }finally{app?.destroy();env.restore();}
}
