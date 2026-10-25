"""Distinguish missing startup telemetry from a confirmed unsuitable flight state."""

def ground_readiness(armed,landed,status_age,land_age):
    if armed is None or status_age is None or status_age>1:
        return 'WAIT'
    if armed:return 'REJECT'
    if landed is None or land_age is None or land_age>2:
        return 'WAIT'
    return 'READY' if landed else 'REJECT'


def simulation_frame_candidate(world_body,px4_enu_pose):
    """Both position frames are geographic ENU in this heading=0 Gazebo world.

    Calibrate only the local-origin translation. Estimated attitude error is not
    a rotation of the geographic position axes.
    """
    import numpy as np
    world=np.asarray(world_body,dtype=float);px4=np.asarray(px4_enu_pose,dtype=float)
    if world.shape!=(4,4) or px4.shape!=(4,4) or not np.isfinite(world).all() or not np.isfinite(px4).all():
        raise ValueError('Invalid frame calibration pose')
    result=np.eye(4);result[:3,3]=px4[:3,3]-world[:3,3]
    return result


def feedback_timestamp(timestamp_us,received_monotonic,now_monotonic,max_age=.3):
    """Reuse PX4's DDS-adjusted clock domain, preserving transport age checks.

    Unlike an OS wall-clock stamp, this is independent of simulation-rate drift.
    Sending a delayed command still retains the old source stamp for PX4 checks.
    """
    import math
    if not isinstance(timestamp_us,int) or timestamp_us<=0:
        raise ValueError('Invalid PX4 feedback timestamp')
    age=now_monotonic-received_monotonic
    if not math.isfinite(age) or age<0 or age>max_age:
        raise ValueError('PX4 feedback too old for a setpoint timestamp')
    return timestamp_us
