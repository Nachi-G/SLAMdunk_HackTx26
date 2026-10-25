"""Offline trajectory checks for the isolated forest fixture, never flight control."""
import math
import time
import xml.etree.ElementTree as ET
import numpy as np
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]


def validate(msg):
    if msg.mode != 'pwp' or not msg.is_agent:
        raise ValueError('Expected an agent cubic trajectory')
    pwp = msg.pwp
    times = np.asarray(pwp.times, dtype=float)
    count = len(times) - 1
    if count < 1 or not np.isfinite(times).all() or np.any(np.diff(times) <= 0):
        raise ValueError('Invalid trajectory segment times')
    if any(len(c) != count for c in (pwp.coeff_x, pwp.coeff_y, pwp.coeff_z)):
        raise ValueError('Coefficient count does not match segment times')
    now = time.time()
    if not times[0] - 0.25 <= now < times[-1]:
        raise ValueError('Trajectory is future dated or already expired')
    points, speeds, accelerations, jerks = [], [], [], []
    boundary_errors = []
    previous_end = None
    for i, duration in enumerate(np.diff(times)):
        if duration > 60:
            raise ValueError('Unexpected segment duration')
        coeff = np.asarray([[getattr(axis[i], n) for n in ('a','b','c','d')]
                            for axis in (pwp.coeff_x, pwp.coeff_y, pwp.coeff_z)], dtype=float)
        if not np.isfinite(coeff).all():
            raise ValueError('Nonfinite polynomial coefficient')
        t = np.linspace(0, duration, max(2, math.ceil(duration / 0.02) + 1))[:, None]
        a,b,c,d = coeff.T
        pos = ((a*t+b)*t+c)*t+d
        vel = (3*a*t+2*b)*t+c
        acc = 6*a*t+2*b
        points.extend(pos)
        speeds.extend(np.linalg.norm(vel,axis=1))
        accelerations.extend(np.linalg.norm(acc,axis=1))
        jerks.append(float(np.linalg.norm(6*a)))
        start = (pos[0], vel[0], acc[0])
        if previous_end is not None:
            boundary_errors.append([float(np.linalg.norm(x-y)) for x,y in zip(start,previous_end)])
        previous_end = (pos[-1],vel[-1],acc[-1])
    points = np.asarray(points)
    clearances = []
    for model in ET.parse(ROOT/'gazebo_worlds/hacktx_forest_lidar_test.sdf').findall('world/model'):
        cylinder = model.find('link/visual/geometry/cylinder')
        if cylinder is None:
            continue
        center = np.asarray(model.findtext('pose').split(),dtype=float)[:3]
        # Horizontal circumscribed circle encloses the 0.8 x 0.8 m drone box
        # for every yaw; expand cylinder caps by the 0.2 m half-height.
        horizontal = np.linalg.norm(points[:,:2]-center[:2],axis=1) - float(cylinder.findtext('radius')) - math.hypot(0.4,0.4)
        vertical = np.abs(points[:,2]-center[2]) - float(cylinder.findtext('length'))/2 - 0.2
        clearances.append(float(np.max(np.column_stack((horizontal,vertical)),axis=1).min()))
    max_boundary = np.max(boundary_errors,axis=0).tolist() if boundary_errors else [0.,0.,0.]
    report = {'duration_s':float(times[-1]-times[0]),'remaining_s':float(times[-1]-now),
              'sampled_max_speed_m_s':float(max(speeds)),
              'sampled_max_acceleration_m_s2':float(max(accelerations)),
              'max_jerk_m_s3':max(jerks),
              'altitude_range_m':[float(points[:,2].min()),float(points[:,2].max())],
              'minimum_sampled_expanded_cylinder_separation_m':min(clearances),
              'maximum_segment_boundary_errors_pos_vel_acc':max_boundary,
              'end_position_enu_m':points[-1].tolist(),
              'goal_distance_at_end_m':float(np.linalg.norm(points[-1]-[10.,-4.44,2.])),
              'checked_tree_count':len(clearances),
              'limits':'Sampled polynomial and ground-truth cylinder check only. No tracking errors, unknown-space guarantee, or flight validation.'}
    passes = (max(speeds)<=1.01 and max(accelerations)<=1.01 and max(jerks)<=2.01
              and points[:,2].min()>=0.5 and points[:,2].max()<=6
              and min(clearances)>0.03 and max(max_boundary)<=0.02
              and report['goal_distance_at_end_m'] < math.hypot(10.,4.44)-0.25)
    report['status'] = 'PASS: sampled planner trajectory' if passes else 'FAIL: sampled trajectory limits/clearance/continuity'
    return report
