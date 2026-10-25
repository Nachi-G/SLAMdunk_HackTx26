import assert from 'node:assert/strict';
import {createMissionLog,readMissions,scanRoom,MISSION_KEY} from '../slam_dunk_bridge/web/mission-log.mjs';
import {mountMissionMap,reviewData} from '../slam_dunk_bridge/web/mission-map.mjs';
import {mountRoom} from '../slam_dunk_bridge/web/room.mjs';
import {createTestMarkup,createTestEnvironment} from './ui_integration.test.mjs';

export function runMissionMapTests(){
  const hits=scanRoom({position:[0,0,1.8],obstacles:[[2,-1,3,1],[10,-10,10.2,10]],movers:[]});
  assert.ok(hits.some(p=>p[0]===2&&p[1]===0),'scan hits nearest obstacle');assert.ok(hits.every(p=>Math.hypot(p[0],p[1])<=8.01),'range limited');assert.ok(!hits.some(p=>p[0]>3),'occluded and out-of-range wall is not observed');
  const values=new Map(),storage={getItem:key=>values.get(key)??null,setItem:(key,value)=>values.set(key,value)};
  const env=createTestEnvironment(1100),doc=createTestMarkup('index.html'),get=id=>doc.getElementById(id);let room,map;
  try{
    room=mountRoom({document:doc,recordingStorage:storage});env.advance(100);
    const clickWorld=p=>{const q=room.scene.project(p),event={clientX:q[0],clientY:q[1],pointerId:1,button:0};get('room-canvas').fire('pointerdown',event);get('room-canvas').fire('pointerup',event);};
    get('room-static').click();clickWorld([0,0,0]);get('room-moving').click();clickWorld([2,-3,0]);get('room-goal').click();clickWorld([7,4,0]);
    env.advance(600);room.recorder.flush();const pending=readMissions(storage);assert.ok(pending);assert.equal(pending.current.frames.at(-1).started,false);
    env.advance(1000);room.recorder.flush();assert.equal(readMissions(storage).current.frames.length,pending.current.frames.length,'idle time does not evict useful history with duplicate frames');
    const mapDoc=createTestMarkup('mission-map.html'),m=id=>mapDoc.getElementById(id);
    map=mountMissionMap({document:mapDoc,storage,autoPoll:false});assert.equal(m('review-empty').hidden,true);
    get('start-room').click();for(let i=0;i<20;i++)env.advance(500);
    get('push-left').fire('pointerdown',{pointerId:5,button:0});env.advance(100);get('push-left').fire('pointerup',{pointerId:5});
    env.advance(600);room.recorder.flush();const recorded=readMissions(storage).current;
    assert.deepEqual(recorded.frames.at(-1).bounds,[-16,-12,16,12]);assert.equal(recorded.frames.at(-1).fixtures.length,10,'recorded furniture is available during replay');
    const liveScene=room.adapter.getSimulation(),missionId=room.recorder.getSnapshot().current.id;
    for(let i=0;i<5;i++){
      const event=get('mission-map-link').fire('click');assert.equal(event.prevented,true);
      assert.equal(get('flight-room-screen').hidden,true);assert.equal(get('mission-review-screen').hidden,false);
      assert.deepEqual(room.review.getView().frame.position,recorded.frames.at(-1).position);
      const mode=room.store.getState().explorationMode;doc.fire('keydown',{key:'w'});assert.equal(room.store.getState().explorationMode,mode,'replay cannot start a push');
      get(i%2?'flight-room-view':'return-room-link').click();
      assert.equal(get('flight-room-screen').hidden,false);assert.equal(get('mission-review-screen').hidden,true);
      assert.deepEqual(room.adapter.getSimulation(),liveScene,'switching never resets the active simulation');
      assert.equal(room.recorder.getSnapshot().current.id,missionId,'switching preserves mission identity');
    }
    assert.ok(recorded.frames.length>10);assert.ok(recorded.events.some(e=>e.kind==='start'));assert.ok(recorded.events.some(e=>e.label.includes('push')));assert.ok(recorded.observations.length>0);
    map.refresh();const latest=map.getView();assert.deepEqual(latest.frame.position,recorded.frames.at(-1).position);assert.ok(latest.trail.length>1);
    m('review-timeline').value='0';m('review-timeline').fire('input');const first=map.getView();assert.equal(first.trail.length,1);assert.ok(first.observations.length<=latest.observations.length);assert.equal(first.frame.goal,null,'future destination not leaked into replay');
    m('review-context').checked=true;m('review-context').fire('change');assert.equal(map.getView().placed.length,0,'initial frame has no future obstacles');
    m('review-live').click();assert.equal(map.getView().placed.length,1);assert.equal(map.getView().movers.length,1);
    const drone=room.adapter.getSimulation().position;m('map-3d').click();m('review-play').click();assert.deepEqual(room.adapter.getSimulation().position,drone,'review controls never control drone');
    get('reset-room').click();room.recorder.flush();const reset=readMissions(storage);assert.equal(reset.archives[0].id,recorded.id);assert.notEqual(reset.current.id,recorded.id);
    map.refresh();m('review-mission').value=recorded.id;m('review-mission').fire('change');assert.deepEqual(map.getView().frame.position,recorded.frames.at(-1).position,'reset preserves prior flight');
    assert.equal(reviewData(recorded,-1),null);
    storage.setItem(MISSION_KEY,'not json');assert.equal(readMissions(storage),null);
    storage.setItem(MISSION_KEY,JSON.stringify({...reset,current:{...reset.current,frames:[{t:0,position:['bad',0,0]}]}}));assert.equal(readMissions(storage),null,'malformed stored coordinates rejected');
  }finally{map?.destroy();room?.destroy();env.restore();}
  const unavailable=createMissionLog({storage:{getItem(){throw Error('Blocked');},setItem(){throw Error('Blocked');}}});assert.equal(unavailable.isPersistent(),false);
  return {sensorOcclusion:true,persistentReplay:true,archivedReset:true,source:'synthetic_room'};
}
