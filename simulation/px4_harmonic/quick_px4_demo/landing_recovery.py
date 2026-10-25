"""Conservative swept-geometry landing and low-speed buffer recovery; no flight I/O."""
import numpy as np
from .planner import DRONE_RADIUS,movers,route

BODY_FLOOR=.30  # Keeps additional separation above the controller's 0.25 m guard.
RECOVERY_SPEED=.15
RECOVERY_ACCEL=.20
LANDING_CLEARANCE=.85


def swept_distances(points,static,profile='baseline'):
    # Match the controller's extra 0.15 m timing allowance on moving objects.
    points=np.asarray(points,dtype=float)
    fixed=np.linalg.norm(points[...,None,:]-static[:,:2],axis=-1)-static[:,2]-DRONE_RADIUS
    moving=[]
    for x,y,amplitude,omega,phase,radius in movers(profile):
        moving.append(np.hypot(points[...,0]-x,np.maximum(np.abs(points[...,1]-y)-amplitude,0))-radius-DRONE_RADIUS-.15)
    return np.concatenate((fixed,np.stack(moving,axis=-1)),axis=-1)


def nearby_grid(point,radius=2.5):
    point=np.asarray(point,dtype=float)
    offsets=np.stack(np.meshgrid(np.arange(-radius,radius+.001,.1),np.arange(-radius,radius+.001,.1)),axis=-1).reshape(-1,2)
    distance=np.linalg.norm(offsets,axis=1);points=point+offsets
    valid=(distance<=radius)&(points[:,0]>=-1.8)&(points[:,0]<=15.8)&(points[:,1]>=-9.8)&(points[:,1]<=9.8)
    order=np.argsort(distance[valid]);return points[valid][order]


def recovery_target(position,goal,t,static,profile='baseline'):
    initial=swept_distances(position,static,profile)
    if initial.min()<BODY_FLOOR:raise ValueError('Insufficient body clearance for automatic recovery')
    candidates=nearby_grid(position)
    candidates=candidates[swept_distances(candidates,static,profile).min(axis=-1)>=.78]
    constrained=initial<.75
    for target in candidates[:80]:
        segment=np.asarray(position)+(target-position)*np.linspace(0,1,41)[:,None]
        distance=swept_distances(segment,static,profile)
        if distance.min()<BODY_FLOOR:continue
        if np.any(distance[:,constrained]<initial[constrained]-.001):continue
        if np.any(np.diff(distance[:,constrained],axis=0)<-.002):continue
        try:tail=route(target,goal,t,static,profile)
        except ValueError:continue
        return target,np.vstack((position,tail))
    raise ValueError('No checked escape from the current obstacle buffer')


def recovery_velocity(position,velocity,target,previous,dt,static,profile='baseline'):
    position=np.asarray(position);velocity=np.asarray(velocity);previous=np.asarray(previous)
    initial=swept_distances(position,static,profile)
    if initial.min()<BODY_FLOOR or np.linalg.norm(velocity)>.30:return np.zeros(2),False,np.empty((0,2))
    difference=np.asarray(target)-position
    desired=difference*min(.7,RECOVERY_SPEED/max(np.linalg.norm(difference),1e-9))
    delta=desired-previous
    command=previous+delta*min(1.,RECOVERY_ACCEL*max(.001,min(dt,.15))/max(np.linalg.norm(delta),1e-9))
    if np.linalg.norm(command)>RECOVERY_SPEED+1e-6:return np.zeros(2),False,np.empty((0,2))
    # Verify the actual issued command, followed by braking, against ALL swept obstacles.
    p=position.copy();v=velocity.copy();prediction=[]
    for i in range(40):
        wanted=command if i<20 else np.zeros(2)
        change=wanted-v;v+=change*min(1.,RECOVERY_ACCEL*.1/max(np.linalg.norm(change),1e-9));p+=v*.1;prediction.append(p.copy())
    prediction=np.asarray(prediction);distance=swept_distances(prediction,static,profile)
    constrained=initial<.75
    valid=(distance.min()>=BODY_FLOOR and np.all(distance[:,constrained]>=initial[constrained]-.002)
           and distance[-1].min()>=initial.min()+.002)
    return command,bool(valid),prediction if valid else np.empty((0,2))


def landing_site(position,requested,t,static,profile='baseline'):
    requested=np.asarray(requested,dtype=float)
    candidates=np.vstack((requested,nearby_grid(requested)))
    candidates=candidates[swept_distances(candidates,static,profile).min(axis=-1)>=LANDING_CLEARANCE]
    for target in candidates[:40]:
        try:return target,route(position,target,t,static,profile)
        except ValueError:continue
    raise ValueError('No reachable landing site with clearance for descent and next takeoff')


def landing_segment_clear(position,target,static,profile='baseline'):
    segment=np.asarray(position)+(np.asarray(target)-position)*np.linspace(0,1,41)[:,None]
    return bool(swept_distances(segment,static,profile).min()>=BODY_FLOOR)
