"""ROS-independent bounded planar control. Metres, radians, seconds."""
from dataclasses import dataclass
import math


def finite(*values):
    return all(math.isfinite(v) for v in values)


def wrap(angle):
    return math.atan2(math.sin(angle),math.cos(angle))


def source_stamp(sec,nanosec,now,max_age=.5):
    stamp=sec+nanosec*1e-9
    if not (0<=nanosec<1_000_000_000 and finite(stamp,now) and 0<stamp<=now+.05 and now-stamp<=max_age):
        raise ValueError('invalid, stale, future or incompatible-clock input')
    return stamp


@dataclass(frozen=True)
class Limits:
    max_speed: float=.4
    max_yaw_rate: float=.8
    acceleration: float=.5
    angular_acceleration: float=1.2
    radius: float=.37
    margin: float=.12
    timeout: float=.35


class DriveGuard:
    """One selected input, source freshness and surrounding-obstacle stop."""
    def __init__(self,limits=Limits()):
        self.limits=limits
        self.reset()

    def reset(self):
        self.request=(0.,0.);self.command=(0.,0.)
        self.request_at=self.scan_at=self.cloud_at=self.odom_at=-math.inf
        self.scan_distance=self.cloud_distance=math.inf
        self.actual_speed=0.;self.estopped=getattr(self,'estopped',False);self.reason='waiting for sensors'

    def set_request(self,v,w,now):
        if not finite(v,w,now):
            self.request_at=-math.inf
            return False
        self.request=(v,w);self.request_at=now
        return True

    def scan(self,ranges,minimum,maximum,now):
        if not finite(minimum,maximum) or not 0<minimum<maximum or not ranges:return False
        values=[r for r in ranges if math.isfinite(r) and minimum<=r<=maximum]
        # Positive infinity is a valid no-return; NaN and under-range returns
        # are not accepted as evidence of a clear surrounding region.
        if any(math.isnan(r) or r<minimum for r in ranges):return False
        self.scan_distance=min(values,default=math.inf);self.scan_at=now
        return True

    def cloud(self,points,now):
        distances=[]
        for x,y,z in points:
            # Ignore the known vehicle body and points outside its swept height.
            if not finite(x,y,z) or z<.04 or z>.68:continue
            if abs(x)<.30 and abs(y)<.24:continue
            distances.append(math.hypot(x,y))
        self.cloud_distance=min(distances,default=math.inf);self.cloud_at=now

    def odom(self,speed,now):
        if not finite(speed,now):return False
        self.actual_speed=abs(speed);self.odom_at=now;return True

    def step(self,now,dt):
        l=self.limits
        if self.estopped:self.reason='operator stop'
        elif any(now-t>l.timeout or now<t for t in (self.request_at,self.scan_at,self.cloud_at,self.odom_at)):
            self.reason='stale command or sensor'
        elif min(self.scan_distance,self.cloud_distance)<l.radius+l.margin+max(self.actual_speed,abs(self.command[0]))**2/(2*l.acceleration):
            self.reason='obstacle inside braking envelope'
        else:
            self.reason='ready'
            target=(max(-l.max_speed,min(l.max_speed,self.request[0])),max(-l.max_yaw_rate,min(l.max_yaw_rate,self.request[1])))
            rates=(l.acceleration,l.angular_acceleration)
            self.command=tuple(old+max(-rate*dt,min(rate*dt,want-old)) for old,want,rate in zip(self.command,target,rates))
            return self.command
        # Withdrawal sends zero immediately; the simulated drivetrain also has
        # acceleration limits, included in the braking envelope above.
        self.command=(0.,0.)
        return self.command


def track_setpoint(pose,target,velocity,yaw,dyaw,plane=.3,max_error=.8):
    if not finite(*pose,*target,*velocity,yaw,dyaw):raise ValueError('nonfinite setpoint')
    if abs(target[2]-plane)>.015 or abs(velocity[2])>.02:raise ValueError('nonplanar SANDO setpoint')
    dx,dy=target[0]-pose[0],target[1]-pose[1];distance=math.hypot(dx,dy)
    if distance>max_error:raise ValueError('tracking error exceeds bound; replan')
    heading=math.atan2(dy,dx) if distance>.06 else yaw
    error=wrap(heading-pose[2]);w=2.0*error+dyaw
    if abs(error)>.5:return 0.,w
    forward=dx*math.cos(pose[2])+dy*math.sin(pose[2])
    feedforward=velocity[0]*math.cos(pose[2])+velocity[1]*math.sin(pose[2])
    return max(0.,feedforward+1.4*forward),w
