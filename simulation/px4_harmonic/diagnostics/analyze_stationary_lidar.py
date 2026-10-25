"""Validate a saved stationary scan against the copied forest's visual geometry.

This uses the known test fixture pose, not PX4 odometry or scan worldPose.
It does not establish a valid transform for a moving vehicle.
"""
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'diagnostics/stationary_lidar'


def analyze(scan):
    width, height = int(scan['count']), int(scan['verticalCount'])
    ranges = np.asarray(scan['ranges'], dtype=float)
    if ranges.size != width * height or width < 2 or height < 2:
        raise ValueError('Unexpected scan dimensions')
    ranges = ranges.reshape(height, width)
    horizontal = float(scan['angleMin']) + np.arange(width) * float(scan['angleStep'])
    vertical = float(scan['verticalAngleMin']) + np.arange(height) * float(scan['verticalAngleStep'])
    valid = (np.isfinite(ranges) & (ranges > float(scan['rangeMin'])) &
             (ranges < float(scan['rangeMax'])))
    row, col = np.nonzero(valid)
    distances = ranges[valid]
    points = np.column_stack((distances * np.cos(vertical[row]) * np.cos(horizontal[col]),
                              distances * np.cos(vertical[row]) * np.sin(horizontal[col]),
                              distances * np.sin(vertical[row])))
    report = analyze_points(points)
    report.update(total_rays=int(ranges.size), horizontal_count=width, vertical_count=height,
                  nearest_return_m=float(distances.min()), reported_world_pose=scan.get('worldPose'))
    return report


def analyze_points(sensor_points, sensor_transform=None):
    """Check finite sensor-frame XYZ points using this fixture's explicit transform."""
    world = ET.parse(ROOT / 'gazebo_worlds/hacktx_forest_lidar_test.sdf').find('world')
    probe = world.find("model[@name='stationary_lidar_probe']")
    pose = np.asarray(probe.findtext('pose').split(), dtype=float)
    if np.any(pose[3:] != 0):
        raise ValueError('This fixture analyzer requires zero probe rotation')
    points = np.asarray(sensor_points, dtype=float).copy()
    if sensor_transform is None:
        points += pose[:3]
    else:
        transform=np.asarray(sensor_transform,dtype=float)
        if transform.shape!=(4,4) or not np.isfinite(transform).all():
            raise ValueError('Invalid sensor world transform')
        rotation=transform[:3,:3]
        if not np.allclose(rotation.T@rotation,np.eye(3),atol=1e-6) or not np.isclose(np.linalg.det(rotation),1):
            raise ValueError('Sensor transform is not a rigid rotation')
        points=points@rotation.T+transform[:3,3]
        pose[:3]=transform[:3,3]
    if not len(points):
        raise ValueError('No finite in-range returns')
    surfaces, names = [], []
    for model in world.findall('model'):
        cylinder = model.find('link/visual/geometry/cylinder')
        if cylinder is None:
            continue
        center = np.asarray(model.findtext('pose').split(), dtype=float)
        link = model.find('link')
        if np.any(center[3:] != 0) or link.find('pose') is not None:
            raise ValueError('Unsupported tree pose in fixture analyzer')
        radius, length = float(cylinder.findtext('radius')), float(cylinder.findtext('length'))
        radial = np.hypot(points[:, 0] - center[0], points[:, 1] - center[1])
        vertical_distance = np.abs(points[:, 2] - center[2])
        side = np.hypot(np.abs(radial - radius), np.maximum(vertical_distance - length / 2, 0))
        caps = np.hypot(np.maximum(radial - radius, 0), np.abs(vertical_distance - length / 2))
        surfaces.append(np.minimum(side, caps))
        names.append(model.get('name'))
    # Ground plane is axis aligned and spans x=[-2,108], y=[-25,25].
    ground = world.find("model[@name='ground_plane_map']")
    ground_pose = np.asarray(ground.findtext('pose').split(), dtype=float)
    size = np.asarray(ground.findtext('link/visual/geometry/plane/size').split(), dtype=float)
    dx = np.maximum(np.abs(points[:, 0] - ground_pose[0]) - size[0] / 2, 0)
    dy = np.maximum(np.abs(points[:, 1] - ground_pose[1]) - size[1] / 2, 0)
    surfaces.append(np.sqrt(dx**2 + dy**2 + (points[:, 2] - ground_pose[2])**2))
    names.append('ground_plane_map')
    errors = np.asarray(surfaces)
    nearest = errors.argmin(axis=0)
    residual = errors.min(axis=0)
    tolerance = 0.10
    matched = residual < tolerance
    hits = {name: int(np.count_nonzero((nearest == index) & matched))
            for index, name in enumerate(names)}
    detected = {name: count for name, count in hits.items()
                if name != 'ground_plane_map' and count >= 10}
    report = {
        'finite_returns': int(len(points)),
        'fixture_sensor_position_enu_m': pose[:3].tolist(),
        'geometry_tolerance_m': tolerance, 'geometry_matched_returns': int(matched.sum()),
        'geometry_match_fraction': float(matched.mean()),
        'median_surface_error_m': float(np.median(residual)),
        'ground_returns': hits['ground_plane_map'], 'detected_tree_returns': detected,
        'status': 'PASS: stationary forest geometry' if matched.mean() >= 0.95 and len(detected) >= 2
                  else 'FAIL: stationary geometry needs investigation',
        'limits': 'One static scan; fixture pose used explicitly. Moving sensor transforms, acquisition timing, ROS delivery, and PX4/SANDO frame alignment remain unvalidated.',
    }
    return report


if __name__ == '__main__':
    report = analyze(json.loads((OUT / 'scan.json').read_text()))
    (OUT / 'summary.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))
