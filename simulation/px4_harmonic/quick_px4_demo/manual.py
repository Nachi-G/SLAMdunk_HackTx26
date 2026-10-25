"""Browser lease adapter. Velocity override is simulation-only, with no avoidance."""
import math
import sys
import time
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[3]/'integration/slam_dunk_bridge'))
from slam_dunk_bridge.control import SteeringSupervisor

class ManualSteering(SteeringSupervisor):
    def __init__(self,clock=time.monotonic,navigate=None,navigation_ready=None,mission_command=None,mission_status=None):
        super().__init__(lambda *_:None,clock=clock,enabled=True,waypoints=((0,0,2),))
        self.mission_command=mission_command;self.mission_status=mission_status
        self.navigate=navigate;self.navigation_ready=navigation_ready
        self.available=False;self.observed=-math.inf;self.position=(0,0,2)
    def observe(self,position,yaw,available):
        with self.lock:
            self.position=tuple(position);self.yaw=float(yaw)
            self.available=bool(available);self.observed=self.clock();self.tick()
    def _ready(self,now):
        return self.available and not self.fault and 0<=now-self.observed<.3
    def _publish(self,target,**kwargs):return True
    def _target(self,direction):
        return (self.position[0]+direction[0],self.position[1]+direction[1],self.position[2])
    def _autonomous(self,**kwargs):
        self.mode='fault' if self.fault else ('autonomous' if self._ready(self.clock()) else 'waiting')
        self.reason=self.fault or ('Following course; hold to steer without obstacle avoidance' if self._ready(self.clock()) else 'Steering available during PX4 navigation only')
    def tick(self):
        with self.lock:
            if self.lease and (self.clock()>=self.lease['expires'] or not self._ready(self.clock())):
                self._end_lease('Released or expired; rejoining course')
            if not self.lease:self._autonomous()
    def velocity(self):
        with self.lock:
            self.tick()
            return tuple(.8*v for v in self.lease['direction']) if self.lease else None
    def stop(self,reason):
        with self.lock:
            self.fault=reason;self.available=False
            if self.lease:self._end_lease(reason)
            self._autonomous()

    def destination_ready(self):
        return bool(not self.fault and (self.navigation_ready() if callable(self.navigation_ready) else self._ready(self.clock())))
    def status(self,**kwargs):
        result=super().status(**kwargs)
        result['navigation_ready']=self.destination_ready()
        if callable(self.mission_status):result.update(self.mission_status())
        return result

    def handle(self,owner_id,message):
        if isinstance(message,dict) and message.get('type') in ('start','return_home'):
            with self.lock:
                action=message['type']
                try:
                    if message.get('protocol_version')!=1 or type(message.get('epoch')) is not int or message['epoch']!=self.epoch or not callable(self.mission_command) or not isinstance(message.get('request_id'),str) or not 0<len(message['request_id'])<=96:raise ValueError('Invalid or stale mission request')
                    flags=self.mission_status() if callable(self.mission_status) else {}
                    if not flags.get('start_ready' if action=='start' else 'return_ready'):raise ValueError('Mission action unavailable in current flight state')
                    if self.lease:self._end_lease('Return Home supersedes manual steering')
                    details=self.mission_command(action);self.epoch+=1
                    result=self.status(accepted=True,request_id=message['request_id'],reason=details['reason'])
                    if 'position' in details:result['destination']=details['position']
                except ValueError as error:result=self.status(accepted=False,request_id=message.get('request_id'),reason=str(error))
        elif isinstance(message,dict) and message.get('type')=='navigate':
            with self.lock:
                self.tick();target=message.get('position')
                valid=(message.get('protocol_version')==1 and not isinstance(message.get('protocol_version'),bool)
                       and isinstance(owner_id,str) and 0<len(owner_id)<=128
                       and isinstance(message.get('request_id'),str) and 0<len(message['request_id'])<=96
                       and type(message.get('epoch')) is int and message['epoch']==self.epoch
                       and self.destination_ready() and self.lease is None and callable(self.navigate)
                       and isinstance(target,list) and len(target)==2
                       and all(type(v) in (int,float) and math.isfinite(v) for v in target))
                try:
                    if not valid:raise ValueError('Destination requires fresh navigation, no held steering, and valid XY')
                    details=self.navigate(target);self.epoch+=1
                    details=details if isinstance(details,dict) else {'position':target,'reason':'Pinned destination accepted'}
                    result=self.status(accepted=True,request_id=message.get('request_id'),reason=details['reason'])
                    result['destination']=details['position'];result['requested_position']=target
                except ValueError as error:
                    result=self.status(accepted=False,request_id=message.get('request_id'),reason=str(error))
                    result['retryable']=bool(str(error).startswith('Destination requires') and not self.fault and callable(self.navigate) and isinstance(target,list) and len(target)==2 and all(type(v) in (int,float) and math.isfinite(v) for v in target) and type(message.get('epoch')) is int and (message['epoch']!=self.epoch or not self.destination_ready() or self.lease is not None))
        else:result=super().handle(owner_id,message)
        if isinstance(message,dict) and (message.get('type')!='steer_renew' or result.get('accepted') is False):
            print('BROWSER CONTROL: '+str(message.get('type'))+' '+str(message.get('position',message.get('direction','')))+
                  ' accepted='+str(result.get('accepted'))+' reason='+str(result.get('reason')),flush=True)
        return result
