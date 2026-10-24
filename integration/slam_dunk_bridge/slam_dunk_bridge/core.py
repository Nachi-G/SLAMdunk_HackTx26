"""ROS-independent decoding and immutable latest-message storage."""

import json
import math
import struct
import threading
import time


def stamp_dict(stamp):
    return {"sec": int(stamp.sec), "nanosec": int(stamp.nanosec)}


def transform_point(point, translation, quaternion):
    """Apply target-from-source rigid transform; quaternion is x,y,z,w."""
    x, y, z, w = quaternion
    norm = math.sqrt(x*x + y*y + z*z + w*w)
    if not math.isfinite(norm) or norm < 1e-12:
        raise ValueError("Invalid transform quaternion")
    x, y, z, w = x/norm, y/norm, z/norm, w/norm
    px, py, pz = point
    tx, ty, tz = 2*(y*pz-z*py), 2*(z*px-x*pz), 2*(x*py-y*px)
    return (px+w*tx+y*tz-z*ty+translation[0],
            py+w*ty+z*tx-x*tz+translation[1],
            pz+w*tz+x*ty-y*tx+translation[2])


def quaternion_product(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    q = (aw*bx+ax*bw+ay*bz-az*by,
         aw*by-ax*bz+ay*bw+az*bx,
         aw*bz+ax*by-ay*bx+az*bw,
         aw*bw-ax*bx-ay*by-az*bz)
    n = math.sqrt(sum(v*v for v in q))
    if not math.isfinite(n) or n < 1e-12:
        raise ValueError("Invalid orientation quaternion")
    return [v/n for v in q]


def decode_xyz(cloud, max_points=30000, max_input_points=2000000):
    """Decode organized PointCloud2 honoring offsets, endian and row padding.

    Uniformly sample input indices to bound work as well as output size. An
    invalid sampled point is omitted; we do not rescan to fill the budget.
    """
    if max_points < 1 or max_input_points < 1:
        raise ValueError("Point limits must be positive")
    width, height = int(cloud.width), int(cloud.height)
    step, row_step = int(cloud.point_step), int(cloud.row_step)
    if width < 0 or height < 0 or step < 0 or row_step < 0:
        raise ValueError("Negative cloud dimensions")
    count = width * height
    if count > max_input_points:
        raise ValueError("Cloud exceeds input point limit")
    if count == 0:
        return [], {"input_points": 0, "sampled_points": 0, "invalid_points": 0}
    if step == 0 or row_step < width*step:
        raise ValueError("Invalid point/row stride")
    data = memoryview(cloud.data)
    if len(data) < (height-1)*row_step + width*step:
        raise ValueError("Truncated PointCloud2 data")
    fields = {f.name: f for f in cloud.fields}
    formats = []
    for name in ("x", "y", "z"):
        field = fields.get(name)
        if field is None or field.datatype not in (7, 8) or field.count != 1:
            raise ValueError("XYZ fields must be scalar FLOAT32/FLOAT64")
        fmt = "f" if field.datatype == 7 else "d"
        if field.offset < 0 or field.offset + struct.calcsize(fmt) > step:
            raise ValueError("XYZ field exceeds point stride")
        formats.append((field.offset, struct.Struct((">" if cloud.is_bigendian else "<") + fmt)))
    stride = max(1, math.ceil(count/max_points))
    points, invalid, sampled = [], 0, 0
    for index in range(0, count, stride):
        row, column = divmod(index, width)
        base = row*row_step + column*step
        point = tuple(parser.unpack_from(data, base+offset)[0] for offset, parser in formats)
        sampled += 1
        if all(math.isfinite(v) for v in point):
            points.append(point)
        else:
            invalid += 1
    return points, {"input_points": count, "sampled_points": sampled, "invalid_points": invalid}


class LatestStore:
    """One serialized message per type, independent of client count/speed."""

    def __init__(self):
        self._lock = threading.Lock()
        self._messages = {}
        self._sequence = 0

    def publish(self, message):
        payload = dict(message, protocol_version=1)
        encoded = json.dumps(payload, separators=(",", ":"), allow_nan=False)
        with self._lock:
            self._sequence += 1
            self._messages[payload["type"]] = (self._sequence, encoded)

    def snapshot(self):
        with self._lock:
            return dict(self._messages)


class MapPublisher:
    """Each cloud is a complete map replacement; never accumulates snapshots."""

    def __init__(self, store, frame_id="map", max_points=30000):
        self.store, self.frame_id, self.max_points = store, frame_id, max_points
        self.revision = 0

    def replace(self, points, stamp, source, statistics=None):
        points = list(points)
        if len(points) > self.max_points:
            raise ValueError("Map exceeds output point limit")
        if any(len(p) != 3 or not all(math.isfinite(v) for v in p) for p in points):
            raise ValueError("Map contains invalid points")
        self.revision += 1
        self.store.publish({"type": "map", "operation": "replace", "revision": self.revision,
                            "frame_id": self.frame_id, "stamp": stamp, "source": source,
                            "point_count": len(points), "points": points,
                            "statistics": statistics or {}})


def health_message(last_received, counters, frame_id):
    now = time.monotonic()
    return {"type": "status", "frame_id": frame_id, "mode": "map_snapshot",
            "age_seconds": {name: None if value is None else round(now-value, 3)
                            for name, value in last_received.items()},
            "counters": dict(counters)}
