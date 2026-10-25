"""Bounded flat-floor frontier selection for a circular skid-steer footprint."""
from collections import deque
import math


def select_frontier(cells,width,height,start,resolution=.2,radius=.49,excluded=()):
    if len(cells)!=width*height or width*height>50000:raise ValueError('invalid bounded grid')
    def neighbors(index):
        x,y=index%width,index//width
        for dx,dy in ((1,0),(-1,0),(0,1),(0,-1)):
            nx,ny=x+dx,y+dy
            if 0<=nx<width and 0<=ny<height:yield ny*width+nx
    def component(seed,passable):
        if seed<0 or seed>=len(cells) or not passable(seed):return {}
        distances={seed:0};queue=deque([seed])
        while queue:
            here=queue.popleft()
            for index in neighbors(here):
                if index not in distances and passable(index):distances[index]=distances[here]+1;queue.append(index)
        return distances
    raw=component(start,lambda index:cells[index]==0)
    frontiers={i for i in raw if any(cells[j]<0 for j in neighbors(i))}
    envelope=math.ceil(radius/resolution)
    offsets=[(dx,dy) for dx in range(-envelope,envelope+1) for dy in range(-envelope,envelope+1) if math.hypot(dx,dy)*resolution<=radius+resolution*.71]
    cache={}
    def clear(index):
        if index in cache:return cache[index]
        x,y=index%width,index//width
        cache[index]=all(0<=x+dx<width and 0<=y+dy<height and cells[(y+dy)*width+x+dx]==0 for dx,dy in offsets)
        return cache[index]
    reachable=component(start,clear)
    if not reachable:return {'state':'waiting','reason':'need an observed-free footprint around the car','goal':None}
    if not frontiers:return {'state':'complete','reason':'no reachable observed-free/unknown boundary','goal':None}
    excluded=set(excluded);best=None;score=-math.inf
    # A viewpoint stays clear of unknown cells; rays look outward for frontiers.
    # Bound work independently of map dimensions with <=512 sampled viewpoints.
    candidates=list(reachable);stride=max(1,math.ceil(len(candidates)/512))
    for index in candidates[::stride]:
        if index in excluded:continue
        x,y=index%width,index//width;visible=set()
        for angle in range(24):
            a=angle*math.tau/24
            for step in range(1,16):
                nx=round(x+step*math.cos(a));ny=round(y+step*math.sin(a))
                if not (0<=nx<width and 0<=ny<height):break
                j=ny*width+nx
                if cells[j]>=65:break
                if cells[j]<0:visible.add(j);break
        gain=len(visible)
        if gain:
            value=gain/(1+.15*reachable[index])
            if value>score:score=value;best=index
    if best is None:return {'state':'blocked','reason':'frontiers remain, but no available clear sampled viewpoint','goal':None}
    return {'state':'exploring','reason':'observed-free viewpoint near unknown space','goal':best}
