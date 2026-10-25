import {readMissions} from './mission-log.mjs';
import {roomBounds} from './planner.mjs';
import {createScene} from './scene.mjs';
export function frameAt(run,time=null){
  if(!run?.frames.length)return -1;if(time===null)return run.frames.length-1;
  let lo=0,hi=run.frames.length-1;while(lo<hi){const mid=Math.ceil((lo+hi)/2);if(run.frames[mid].t<=time)lo=mid;else hi=mid-1;}return lo;
}
export function reviewData(run,index,context=false){
  if(!run||index<0||index>=run.frames.length)return null;
  const frame=run.frames[index];return {frame,observations:run.observations.filter(p=>p[3]<=frame.t).map(p=>p.slice(0,3)),trail:run.frames.slice(0,index+1).map(f=>f.position),events:run.events.filter(e=>e.t<=frame.t),placed:context?frame.placed:[],movers:context?frame.movers:[]};
}
export function mountMissionMap({document:doc=globalThis.document,storage,autoPoll=true,dataSource,onReturn}={}){
  const $=id=>doc.getElementById(id),removers=[];
  if(storage===undefined)try{storage=globalThis.localStorage;}catch{storage=null;}
  let missions=null,selectedId='current',cursor=null,playing=false,context=false,lastPlay=Date.now(),pollTimer=null,playTimer=null,missionSignature='';
  const scene=createScene($('review-canvas'),{mode:'top',follow:false});
  function on(el,type,fn){if(typeof el==='string')el=$(el);el.addEventListener(type,fn);removers.push(()=>el.removeEventListener(type,fn));}
  const text=(id,value)=>{$(id).textContent=String(value);};
  const time=ms=>{const seconds=Math.max(0,Math.floor(ms/1000));return `${String(Math.floor(seconds/60)).padStart(2,'0')}:${String(seconds%60).padStart(2,'0')}`;};
  const run=()=>selectedId==='current'?missions?.current:missions?.archives.find(r=>r.id===selectedId);
  for(const id of ['flight-room-link','return-room-link'])if($(id)&&onReturn)on(id,'click',event=>{event.preventDefault();onReturn();});
  function render(){
    const mission=run(),index=frameAt(mission,cursor),view=reviewData(mission,index,context),frame=view?.frame;
    $('review-empty').hidden=!!frame;
    for(const id of ['review-timeline','review-play','review-live','review-download'])$(id).disabled=!frame;
    if(!frame){text('review-storage',storage?'No saved recording yet.':'Local history storage is unavailable.');return;}
    const [x0,y0,x1,y1]=frame.bounds??roomBounds(frame.obstacles);
    scene.setData({room:true,bounds:[x0,y0,x1,y1],fixtures:context?frame.fixtures??[]:[],points:[[x0,y0,0],[x1,y0,0],[x1,y1,3.3],[x0,y1,3.3]],pose:{position:frame.position,yaw:frame.yaw},trail:view.trail,planned:frame.planned,goal:frame.goal,observations:view.observations,placed:view.placed,movers:view.movers,heat:false,bodyRadius:frame.bodyRadius,staticMargin:frame.bodyRadius+.45});
    text('review-state',selectedId!=='current'?'ARCHIVED MISSION':cursor===null?'FOLLOWING LATEST RECORDING':'HISTORICAL FRAME');
    text('review-position',`X ${frame.position[0].toFixed(1)} · Y ${frame.position[1].toFixed(1)} · Z ${frame.position[2].toFixed(1)} m`);
    const first=mission.frames[0].t,last=mission.frames.at(-1).t;
    text('review-time',time(frame.t-mission.created));text('review-duration',`${time(last-first)} retained · ${mission.frames.length} frames`);
    text('review-start',mission.truncated?'Oldest retained frame':'Recording start');
    $('review-timeline').max=String(Math.max(1,mission.frames.length-1));$('review-timeline').value=String(index);
    text('review-play',playing?'Ⅱ Pause replay':'▶ Play');text('review-speed',`${frame.speed.toFixed(2)} m/s`);text('review-clearance',`${frame.clearance.toFixed(2)} m`);text('review-distance',`${frame.distance.toFixed(1)} m`);text('review-points',view.observations.length);text('review-plans',`${frame.plans} / ${frame.completed}`);
    text('review-storage',selectedId==='current'?(dataSource?'Live recording from this flight room.':'Updates from the flight room approximately every 1.5 seconds.'):'Preserved when the room was reset or reloaded.');
    text('review-limits',`${mission.truncated?'Older trajectory frames were removed to fit storage. ':''}Up to 30 minutes of frames, 600 recent events, and three prior missions; storage limits can shorten retention. Saved in this browser and port.`);
    text('review-status',frame.status);text('review-event-count',`${view.events.length} EVENTS`);
    $('review-events').replaceChildren();
    for(const event of view.events.slice(-80).reverse()){
      const button=doc.createElement('button');button.textContent=`${time(event.t-mission.created)}  ${event.label}`;
      button.addEventListener('click',()=>{cursor=event.t;playing=false;render();});$('review-events').appendChild(button);
    }
  }
  function refresh(){
    missions=dataSource?dataSource():readMissions(storage);
    const signature=missions?JSON.stringify([missions.current.id,...missions.archives.map(r=>r.id)]):'';
    if(signature!==missionSignature){
      missionSignature=signature;$('review-mission').replaceChildren();
      if(missions)for(const [value,label]of [['current',`${missions.current.name} · Latest`],...missions.archives.map(r=>[r.id,r.name])]){const option=doc.createElement('option');option.value=value;option.textContent=label;$('review-mission').appendChild(option);}
      if(selectedId!=='current'&&!missions?.archives.some(r=>r.id===selectedId)){selectedId='current';cursor=null;}
      $('review-mission').value=selectedId;
    }
    render();
  }
  on('review-mission','change',()=>{selectedId=$('review-mission').value;cursor=null;playing=false;render();scene.fit();});
  on('review-timeline','input',()=>{const mission=run(),frame=mission?.frames[Number($('review-timeline').value)];if(frame){cursor=frame.t;playing=false;render();}});
  on('review-live','click',()=>{cursor=null;playing=false;render();});
  on('review-play','click',()=>{const mission=run();if(!mission?.frames.length)return;if(!playing&&(cursor===null||cursor>=mission.frames.at(-1).t))cursor=mission.frames[0].t;playing=!playing;lastPlay=Date.now();render();});
  on('review-context','change',()=>{context=$('review-context').checked;render();});
  function mode(value){scene.setMode(value);for(const [id,type]of [['map-top','top'],['map-3d','perspective']]){$(id).classList.toggle('selected',type===value);$(id).setAttribute('aria-pressed',String(type===value));}}
  on('map-top','click',()=>mode('top'));on('map-3d','click',()=>mode('perspective'));on('map-fit','click',()=>scene.fit());
  on('review-download','click',()=>{const mission=run();if(!mission)return;const url=URL.createObjectURL(new Blob([JSON.stringify(mission,null,2)],{type:'application/json'})),a=doc.createElement('a');a.href=url;a.download=`slamdunk-${mission.id}.json`;a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);});
  if(autoPoll){pollTimer=setInterval(refresh,1500);playTimer=setInterval(()=>{if(!playing)return;const mission=run();if(!mission?.frames.length)return;const now=Date.now();cursor=Math.min(mission.frames.at(-1).t,(cursor??mission.frames[0].t)+Math.max(0,Math.min(1000,now-lastPlay)));lastPlay=now;if(cursor>=mission.frames.at(-1).t)playing=false;render();},100);}
  refresh();
  return Object.freeze({scene,refresh,getView:()=>reviewData(run(),frameAt(run(),cursor),context),destroy(){clearInterval(pollTimer);clearInterval(playTimer);for(const remove of removers)remove();scene.destroy();}});
}
if(typeof document!=='undefined'&&document.getElementById('review-canvas')&&!document.getElementById('room-canvas'))globalThis.location.replace('/flight-room.html#mission-map');
