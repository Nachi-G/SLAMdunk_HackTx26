"""Ground-truth forest planning for the separate PX4 demo; no flight I/O."""
import heapq
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
DRONE_RADIUS=.65
MARGIN=.35
MAX_SPEED=.65
MAX_ACCEL=.45
# x, center_y, lateral amplitude, frequency, phase, radius
MOVERS=((4.,0.,2.5,.28,0.,.65),(8.,0.,2.8,.24,1.7,.65))
SLALOM_MOVERS=((2.8,0.,.35,.45,0.,.65),(6.5,2.7,.35,.4,1.7,.65),
               (10.5,-2.2,.35,.42,.8,.65))
SLALOM_WAYPOINTS=((0.,0.),(2.8,2.8),(6.4,-.4),(11.2,1.2),(13.,0.))

CHALLENGE_MOVERS=SLALOM_MOVERS+tuple(
    (x,y,.55,.30+.025*i,.7*i,.65)
    for i,(x,y) in enumerate(((0.,6.),(4.,6.),(8.,6.),(12.,6.),
                             (0.,-6.),(4.,-6.),(8.,-6.),(12.,-6.),(16.,0.))))
CHALLENGE_WAYPOINTS=SLALOM_WAYPOINTS+((11.2,1.2),(6.4,-.4),(2.8,2.8),(0.,0.))


def movers(profile='baseline'):
    if profile not in ('baseline','slalom','challenge'):raise ValueError('Unknown demo profile')
    return {'baseline':MOVERS,'slalom':SLALOM_MOVERS,'challenge':CHALLENGE_MOVERS}[profile]


def trees():
    result=[]
    for m in ET.parse(ROOT/'gazebo_worlds/hacktx_forest.sdf').findall('world/model'):
        c=m.find('link/collision/geometry/cylinder')
        if c is not None:
            p=list(map(float,m.findtext('pose').split()))
            result.append((p[0],p[1],float(c.findtext('radius'))))
    return np.asarray(result)


def moving_positions(t,profile='baseline'):
    t=np.asarray(t,dtype=float)
    return np.stack([np.stack((np.full_like(t,x),center_y+amplitude*np.sin(omega*t+phase)),axis=-1)
                     for x,center_y,amplitude,omega,phase,radius in movers(profile)],axis=-2)


def clearance(points,t,static,margin=MARGIN,profile='baseline'):
    points=np.asarray(points,dtype=float)
    static_distance=np.linalg.norm(points[...,None,:]-static[:,:2],axis=-1)-static[:,2]-DRONE_RADIUS-margin
    dynamic=moving_positions(t,profile)
    dynamic_distance=np.linalg.norm(points[...,None,:]-dynamic,axis=-1)-np.array([m[5] for m in movers(profile)])-DRONE_RADIUS-margin-.15
    return np.minimum(static_distance.min(axis=-1),dynamic_distance.min(axis=-1))


def route(start,goal,t,static,profile='baseline'):
    resolution=.2 if profile!='baseline' else .4
    origin=np.array([-2.,-10.])
    shape=(91,101) if profile!='baseline' else (46,51)
    def cell(p):return tuple(np.rint((np.asarray(p)-origin)/resolution).astype(int))
    def position(c):return origin+np.asarray(c)*resolution
    a,b=cell(start),cell(goal)
    coordinates=np.stack(np.meshgrid(np.arange(shape[0]),np.arange(shape[1]),indexing='ij'),axis=-1)
    world=origin+coordinates*resolution
    # Global route avoids each mover's full known swept capsule. This prevents
    # repeated side switching/local traps as objects cross the corridor.
    static_distance=np.linalg.norm(world[...,None,:]-static[:,:2],axis=-1)-static[:,2]-DRONE_RADIUS-MARGIN
    blocked=static_distance.min(axis=-1)<.15
    for x,center_y,amplitude,omega,phase,radius in movers(profile):
        capsule_distance=np.hypot(world[...,0]-x,np.maximum(np.abs(world[...,1]-center_y)-amplitude,0))
        blocked |= capsule_distance < radius+DRONE_RADIUS+MARGIN+.3
    if not all(0<=a[i]<shape[i] and 0<=b[i]<shape[i] for i in range(2)):raise ValueError('Goal/start outside demo bounds')
    blocked[a]=False
    if blocked[b]:raise ValueError('Goal inside inflated obstacle')
    heap=[(0.,0.,a)];cost={a:0.};parent={};found=False
    while heap:
        _,g,c=heapq.heappop(heap)
        if g>cost[c]+1e-8:continue
        if c==b:found=True;break
        for dx,dy in ((1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)):
            n=(c[0]+dx,c[1]+dy)
            if not 0<=n[0]<shape[0] or not 0<=n[1]<shape[1] or blocked[n]:continue
            if dx and dy and (blocked[c[0]+dx,c[1]] or blocked[c[0],c[1]+dy]):continue
            ng=g+math.hypot(dx,dy)
            if ng>=cost.get(n,math.inf):continue
            cost[n]=ng;parent[n]=c
            heapq.heappush(heap,(ng+math.dist(n,b),ng,n))
    if not found:raise ValueError('No ground-truth route found')
    chain=[b]
    while chain[-1]!=a:chain.append(parent[chain[-1]])
    path=np.array([position(c) for c in reversed(chain)])
    path[0]=start;path[-1]=goal
    return path


def choose_velocity(position,velocity,goal,path,t,static,previous_command,dt,profile='baseline',return_prediction=False):
    position=np.asarray(position);velocity=np.asarray(velocity)
    # Follow an intermediate global waypoint, not a straight line through obstacles.
    nearest=int(np.argmin(np.linalg.norm(path-position,axis=1)))
    index=nearest
    while index+1<len(path) and np.linalg.norm(path[index]-position)<1.4:index+=1
    target=path[index]
    heading=math.atan2(target[1]-position[1],target[0]-position[0])
    angles=heading+np.linspace(-math.pi,math.pi,25)
    candidates=np.array([[speed*math.cos(a),speed*math.sin(a)] for speed in (.25,.45,.65) for a in angles]+[[0.,0.]])
    # Predict acceleration-limited response, then verify a braking tail as well.
    steps=np.arange(.1,4.01,.1)
    rollout=np.empty((len(candidates),len(steps),2));p=np.repeat(position[None,:],len(candidates),axis=0)
    v=np.repeat(velocity[None,:],len(candidates),axis=0)
    for i,s in enumerate(steps):
        desired=candidates if s<=2.5 else np.zeros_like(candidates)
        delta=desired-v
        delta*=np.minimum(1.,MAX_ACCEL*.1/np.maximum(np.linalg.norm(delta,axis=1),1e-9))[:,None]
        v+=delta;p+=v*.1;rollout[:,i]=p
    safety=clearance(rollout,t+steps[None,:],static,profile=profile)
    valid=np.all(safety>.05,axis=1)
    # Avoid backward steps unless needed to evade; prefer target progress and separation.
    distance=np.linalg.norm(rollout[:,24]-target,axis=1)
    goal_distance=np.linalg.norm(rollout[:,24]-goal,axis=1)
    score=distance+.15*goal_distance+.12*np.linalg.norm(candidates-previous_command,axis=1)-.1*np.minimum(safety.min(axis=1),2.)
    score[~valid]=math.inf
    if not np.isfinite(score).any():
        result=(np.zeros(2),False,float(safety.max()))
        return (*result,np.empty((0,2))) if return_prediction else result
    desired=candidates[int(np.argmin(score))]
    delta=desired-previous_command
    delta*=min(1.,MAX_ACCEL*max(.001,min(dt,.15))/max(np.linalg.norm(delta),1e-9))
    command=previous_command+delta
    # The command is lower acceleration than the rollout's steady desired speed.
    result=(command,True,float(safety[int(np.argmin(score))].min()))
    return (*result,rollout[int(np.argmin(score))].copy()) if return_prediction else result


def resolve_pin(start,requested,t,static,profile='baseline',max_snap=2.5):
    """Keep exact feasible pins; otherwise try nearby clear/reachable grid points."""
    requested=np.asarray(requested,dtype=float)
    if requested.shape!=(2,) or not np.isfinite(requested).all():raise ValueError('Invalid destination coordinates')
    if not (-2<=requested[0]<=16 and -10<=requested[1]<=10):raise ValueError('Pin outside course: X -2 to 16 m, Y -10 to 10 m')
    try:return requested.copy(),route(start,requested,t,static,profile),False
    except ValueError as original:
        resolution=.2 if profile!='baseline' else .4
        grid=np.stack(np.meshgrid(np.arange(-2,16.001,resolution),np.arange(-10,10.001,resolution)),axis=-1).reshape(-1,2)
        distances=np.linalg.norm(grid-requested,axis=1)
        safe=(distances<=max_snap)&(distances>.001)
        separation=np.linalg.norm(grid[:,None,:]-static[None,:,:2],axis=-1)-static[None,:,2]-DRONE_RADIUS-MARGIN
        safe &= np.all(separation>=.25,axis=1)
        for x,y,amplitude,omega,phase,radius in movers(profile):
            safe &= np.hypot(grid[:,0]-x,np.maximum(np.abs(grid[:,1]-y)-amplitude,0))>=radius+DRONE_RADIUS+MARGIN+.4
        candidates=np.where(safe)[0];candidates=candidates[np.argsort(distances[candidates])]
        for index in candidates[:4]:
            try:return grid[index].copy(),route(start,grid[index],t,static,profile),True
            except ValueError:pass
        raise ValueError(f'No reachable clear point within {max_snap:.1f} m of pin; choose a different spot ({original})') from original
