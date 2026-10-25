"""On-ground X500 sensor/geometry verification using captured model/link poses."""
import json
from pathlib import Path
import sys
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from perception_geometry import json_pose
from analyze_stationary_lidar import analyze_points

OUT=Path(__file__).resolve().parent/'x500_lidar'


def read_messages(path):
    raw=path.read_text();decoder=json.JSONDecoder();messages=[]
    while raw.strip():
        raw=raw.lstrip();value,n=decoder.raw_decode(raw);messages.append(value);raw=raw[n:]
    return messages


def scan_points(scan):
    width,height=int(scan['count']),int(scan['verticalCount'])
    r=np.asarray(scan['ranges'],dtype=float).reshape(height,width)
    h=float(scan['angleMin'])+np.arange(width)*float(scan['angleStep'])
    v=float(scan['verticalAngleMin'])+np.arange(height)*float(scan['verticalAngleStep'])
    row,col=np.nonzero(np.isfinite(r)&(r>float(scan['rangeMin']))&(r<float(scan['rangeMax'])))
    d=r[row,col]
    return np.column_stack((d*np.cos(v[row])*np.cos(h[col]),d*np.cos(v[row])*np.sin(h[col]),d*np.sin(v[row])))


def main():
    poses=read_messages(OUT/'poses.jsonl');scans=read_messages(OUT/'scans.jsonl')
    if not poses or not scans:raise ValueError('Missing scans or poses')
    transforms=[]
    for snapshot in poses:
        by_name={p['name']:p for p in snapshot['pose']}
        model=json_pose(by_name['x500_0']);base=json_pose(by_name['base_link']);sensor=json_pose(by_name['sando_lidar_link'])
        extrinsic=np.linalg.inv(base)@sensor
        if not np.allclose(extrinsic[:3,3],[0,0,.3],atol=1e-6) or not np.allclose(extrinsic[:3,:3],np.eye(3),atol=1e-6):
            raise ValueError('Active sensor extrinsic does not match configured 0.30 m mount')
        transforms.append(model@sensor)
    # This capture is sequential, so use it only with demonstrably stationary sensor poses.
    spread=max(np.linalg.norm(t[:3,3]-transforms[0][:3,3]) for t in transforms)
    rotation_spread=max(np.linalg.norm(t[:3,:3]-transforms[0][:3,:3]) for t in transforms)
    if spread>1e-4 or rotation_spread>1e-4:raise ValueError('Sequential capture is not stationary; synchronized pose interpolation required')
    status=json.loads((OUT/'status.json').read_text())
    if status.get('landed') is not True or status.get('px4_status',{}).get('armed') is not False:
        raise ValueError('Capture does not demonstrate landed and disarmed condition')
    reports=[analyze_points(scan_points(scan),transforms[0]) for scan in scans]
    report={'status':'PASS: grounded X500 LiDAR extrinsic and forest geometry' if all(r['status'].startswith('PASS') for r in reports) else 'FAIL: grounded sensor geometry mismatch',
            'scan_count':len(scans),'sensor_mount_relative_base_m':extrinsic[:3,3].tolist(),
            'sensor_world_position_m':transforms[0][:3,3].tolist(),
            'stationary_pose_spread_m':spread,'minimum_geometry_match_fraction':min(r['geometry_match_fraction'] for r in reports),
            'last_scan_geometry':reports[-1],
            'limits':'Sequential stationary scan/pose capture only. Scan worldPose is not used. Moving transforms and PX4 local-origin/timestamp alignment remain unvalidated.'}
    (OUT/'geometry_summary.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))

if __name__=='__main__':main()
