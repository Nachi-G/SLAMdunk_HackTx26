"""Validated cubic trajectory evaluation for preview use; no ROS or flight I/O."""
from bisect import bisect_right
from dataclasses import dataclass
import math
import numpy as np


@dataclass(frozen=True)
class Setpoint:
    position: tuple
    velocity: tuple
    acceleration: tuple
    jerk: tuple

    def ned(self):
        def swap(v):
            return (v[1], v[0], -v[2])
        return Setpoint(*(swap(v) for v in (self.position,self.velocity,self.acceleration,self.jerk)))


def yaw_enu_to_ned(yaw):
    if not math.isfinite(yaw):
        raise ValueError('Nonfinite yaw')
    return math.atan2(math.sin(math.pi/2-yaw), math.cos(math.pi/2-yaw))


def candidates(polynomial, duration):
    """Endpoints and all real stationary roots within this segment."""
    derivative=np.polynomial.polynomial.polyder(polynomial)
    roots=np.polynomial.polynomial.polyroots(derivative) if len(derivative)>1 else []
    return [0.,duration]+[float(r.real) for r in roots
                          if abs(r.imag)<1e-8 and 0<r.real<duration]


def maximum_norm(components,duration):
    norm_squared=np.zeros(1)
    for component in components:
        norm_squared=np.polynomial.polynomial.polyadd(
            norm_squared,np.polynomial.polynomial.polymul(component,component))
    return math.sqrt(max(0.,max(float(np.polynomial.polynomial.polyval(t,norm_squared))
                               for t in candidates(norm_squared,duration))))


class CubicTrajectory:
    def __init__(self,times,coefficients):
        self.times=tuple(float(t) for t in times)
        self.coeff=np.asarray(coefficients,dtype=float).copy()
        count=len(self.times)-1
        if count<1 or self.coeff.shape!=(count,3,4):
            raise ValueError('Polynomial/segment dimensions mismatch')
        if not all(math.isfinite(t) for t in self.times) or not np.isfinite(self.coeff).all():
            raise ValueError('Nonfinite trajectory')
        durations=np.diff(self.times)
        if np.any(durations<=0) or sum(durations)>60:
            raise ValueError('Nonincreasing timestamps or horizon over 60 seconds')
        self.maxima=[0.,0.,0.]
        previous=None
        for i,duration in enumerate(durations):
            a,b,c,d=self.coeff[i].T
            velocity=np.column_stack((c,2*b,3*a))
            acceleration=np.column_stack((2*b,6*a))
            jerk=(6*a).reshape(3,1)
            for k,components in enumerate((velocity,acceleration,jerk)):
                self.maxima[k]=max(self.maxima[k],maximum_norm(components,duration))
            z=self.coeff[i,2,::-1]
            heights=[np.polynomial.polynomial.polyval(t,z) for t in candidates(z,duration)]
            if min(heights)<0.5-1e-6 or max(heights)>6+1e-6:
                raise ValueError('Trajectory exceeds altitude bounds')
            start=self.segment(i,0.)
            if previous is not None:
                for name,tolerance in [('position',0.02),('velocity',0.02),('acceleration',0.02)]:
                    if np.linalg.norm(np.array(getattr(start,name))-getattr(previous,name))>tolerance:
                        raise ValueError(f'Discontinuous segment {name}')
            previous=self.segment(i,duration)
        if any(value>limit+0.001 for value,limit in zip(self.maxima,(1.,1.,2.))):
            raise ValueError(f'Continuous dynamics limit exceeded: {self.maxima}')
        self.coeff.setflags(write=False)

    @classmethod
    def from_message(cls,msg,agent_id=1):
        if msg.mode!='pwp' or not msg.is_agent or msg.id!=agent_id:
            raise ValueError('Unexpected trajectory mode or agent ID')
        p=msg.pwp
        count=len(p.times)-1
        if any(len(c)!=count for c in (p.coeff_x,p.coeff_y,p.coeff_z)):
            raise ValueError('Coefficient count mismatch')
        return cls(p.times,[[[getattr(axis[i],n) for n in ('a','b','c','d')]
                             for axis in (p.coeff_x,p.coeff_y,p.coeff_z)] for i in range(count)])

    def segment(self,i,t):
        a,b,c,d=self.coeff[i].T
        return Setpoint(tuple(((a*t+b)*t+c)*t+d),tuple((3*a*t+2*b)*t+c),
                        tuple(6*a*t+2*b),tuple(6*a))

    def evaluate(self,now):
        if not math.isfinite(now) or not self.times[0]<=now<self.times[-1]:
            raise ValueError('Trajectory is not active at requested time')
        i=bisect_right(self.times,now)-1
        return self.segment(i,now-self.times[i])


class PreviewWatchdog:
    """Fail closed in preview. No hold, mode change, or other flight action."""
    def __init__(self):
        self.trajectory=None
        self.received=None
        self.last_clock=None
        self.blocked_reason='No validated trajectory'

    def block(self,reason):
        self.trajectory=None
        self.blocked_reason=reason

    def accept(self,trajectory,wall,monotonic,state_position):
        if not trajectory.times[0]<=wall<trajectory.times[-1]:
            self.block('Incoming trajectory is not currently active')
            raise ValueError(self.blocked_reason)
        new=trajectory.evaluate(wall)
        if np.linalg.norm(np.array(new.position)-state_position)>0.75:
            self.block('Initial setpoint is too far from measured state')
            raise ValueError(self.blocked_reason)
        if self.trajectory is not None and self.trajectory.times[0]<=wall<self.trajectory.times[-1]:
            old=self.trajectory.evaluate(wall)
            for name,tolerance in [('position',0.15),('velocity',0.3),('acceleration',0.5)]:
                if np.linalg.norm(np.array(getattr(new,name))-getattr(old,name))>tolerance:
                    self.block(f'Replan {name} jump')
                    raise ValueError(self.blocked_reason)
        self.trajectory=trajectory
        self.received=monotonic
        self.blocked_reason=None

    def tick(self,wall,monotonic,state_position,state_age):
        if self.last_clock is not None:
            dw=wall-self.last_clock[0]
            dm=monotonic-self.last_clock[1]
            if dm<0 or abs(dw-dm)>0.25:
                self.block('Wall clock jumped relative to monotonic clock')
            elif dm>0.2:
                self.block('Control timer stalled over 0.2 seconds')
        self.last_clock=(wall,monotonic)
        if self.trajectory is None:
            raise ValueError(self.blocked_reason)
        if state_age<0 or state_age>0.3 or not np.isfinite(state_position).all():
            self.block('Missing or stale valid state')
        elif monotonic-self.received>1.0:
            self.block('No accepted replan for over 1 second')
        elif self.trajectory.times[-1]-wall<0.15:
            self.block('Trajectory expired or less than 0.15 seconds remain')
        if self.trajectory is None:
            raise ValueError(self.blocked_reason)
        point=self.trajectory.evaluate(wall)
        if np.linalg.norm(np.array(point.position)-state_position)>1.0:
            self.block('Reference is over 1 metre from measured state')
            raise ValueError(self.blocked_reason)
        return point
