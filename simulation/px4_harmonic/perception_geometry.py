"""Rigid sensor/model transforms and time interpolation, independent of ROS."""
from bisect import bisect_left
import math
import numpy as np


def normalize_quaternion(q):
    q=np.asarray(q,dtype=float)
    if q.shape!=(4,) or not np.isfinite(q).all():raise ValueError('Invalid quaternion')
    norm=np.linalg.norm(q)
    if norm<1e-8:raise ValueError('Zero quaternion')
    return q/norm


def rotation_matrix(q):
    w,x,y,z=normalize_quaternion(q)
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


def pose_matrix(position,quaternion):
    p=np.asarray(position,dtype=float)
    if p.shape!=(3,) or not np.isfinite(p).all():raise ValueError('Invalid position')
    t=np.eye(4);t[:3,:3]=rotation_matrix(quaternion);t[:3,3]=p
    return t


def json_pose(pose):
    p=pose.get('position',{});q=pose.get('orientation',{})
    return pose_matrix([p.get(k,0.) for k in 'xyz'],[q.get(k,0.) for k in ('w','x','y','z')])


def slerp(q0,q1,fraction):
    if not 0<=fraction<=1:raise ValueError('Extrapolation is not permitted')
    q0=normalize_quaternion(q0);q1=normalize_quaternion(q1)
    dot=float(q0@q1)
    if dot<0:q1=-q1;dot=-dot
    dot=min(1.,max(-1.,dot))
    if dot>.9995:return normalize_quaternion(q0+fraction*(q1-q0))
    angle=math.acos(dot)
    return (math.sin((1-fraction)*angle)*q0+math.sin(fraction*angle)*q1)/math.sin(angle)


class PoseHistory:
    def __init__(self,max_gap=.15,capacity=500):
        self.max_gap=max_gap;self.capacity=capacity;self.samples=[]

    def add(self,stamp,position,quaternion):
        if not math.isfinite(stamp):raise ValueError('Invalid acquisition stamp')
        p=np.asarray(position,dtype=float)
        pose_matrix(p,quaternion)
        if self.samples and stamp<=self.samples[-1][0]:
            raise ValueError('Pose acquisition time reset or out of order')
        self.samples.append((stamp,p,normalize_quaternion(quaternion)))
        self.samples=self.samples[-self.capacity:]

    def at(self,stamp):
        i=bisect_left([s[0] for s in self.samples],stamp)
        if i<len(self.samples) and self.samples[i][0]==stamp:
            _,p,q=self.samples[i];return pose_matrix(p,q)
        if i==0 or i==len(self.samples):raise ValueError('Pose would require extrapolation')
        t0,p0,q0=self.samples[i-1];t1,p1,q1=self.samples[i]
        if t1-t0>self.max_gap:raise ValueError('Pose interpolation gap too large')
        fraction=(stamp-t0)/(t1-t0)
        return pose_matrix(p0+fraction*(p1-p0),slerp(q0,q1,fraction))


def px4_pose_enu(position_ned,quaternion_ned_frd):
    enu_ned=np.array([[0.,1.,0.],[1.,0.,0.],[0.,0.,-1.]])
    frd_flu=np.diag([1.,-1.,-1.])
    result=np.eye(4)
    result[:3,:3]=enu_ned@rotation_matrix(quaternion_ned_frd)@frd_flu
    result[:3,3]=enu_ned@np.asarray(position_ned,dtype=float)
    if not np.isfinite(result).all():raise ValueError('Nonfinite PX4 pose')
    return result


def simulation_sample_time(timestamp_sample_us,estimated_offset_us):
    # PX4 generated serializers add agent/PX4 time offset to sample timestamps;
    # TimesyncStatus retains the opposite signed estimated_offset unchanged.
    result=(int(timestamp_sample_us)+int(estimated_offset_us))/1e6
    if not math.isfinite(result) or result<0:raise ValueError('Invalid mapped simulation sample time')
    return result


def mean_rigid_transform(transforms):
    values=np.asarray(transforms,dtype=float)
    if values.ndim!=3 or values.shape[1:]!=(4,4) or len(values)<1 or not np.isfinite(values).all():
        raise ValueError('Invalid rigid calibration samples')
    u,_,vh=np.linalg.svd(values[:,:3,:3].mean(axis=0))
    if np.linalg.det(u@vh)<0:u[:,-1]*=-1
    result=np.eye(4);result[:3,:3]=u@vh
    result[:3,3]=np.median(values[:,:3,3],axis=0)
    spread=float(np.linalg.norm(values[:,:3,3]-result[:3,3],axis=1).max())
    angles=[math.acos(np.clip((np.trace(result[:3,:3].T@r)-1)/2,-1,1)) for r in values[:,:3,:3]]
    if spread>.15 or max(angles)>.035:raise ValueError('Ground calibration samples are inconsistent')
    return result,spread,max(angles)
