"""ROS-independent, leased destination ownership for SANDO.

This publishes destinations, not actuator setpoints or a physical stop. A
configured waypoint provider supplies the initial autonomy; it is not frontier
exploration. All authority transitions and goal publication share one lock.
"""

import math
import re
import secrets
import threading
import time


def _finite(value):
    try:
        return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)
    except OverflowError:
        return False


def _integer(value):
    return isinstance(value, int) and not isinstance(value, bool) and 0 <= value <= 2**53-1


def _vector(value, length):
    return isinstance(value, (list, tuple)) and len(value) == length and all(_finite(v) for v in value)


class SteeringSupervisor:
    """Only writer of operator/autonomy destinations, with a monotonic lease."""

    DIRECTIONS = {"forward": (1, 0), "backward": (-1, 0), "left": (0, 1), "right": (0, -1)}

    def __init__(self, publish_goal, *, clock=time.monotonic, waypoints=(), enabled=False,
                 frame_id="map", bounds=(-200, 200, -200, 200, .5, 6), lookahead=3.0,
                 goal_seen_radius=2.0, ttl_seconds=.5, freshness_seconds=2.0,
                 refresh_hz=5.0, waypoint_radius=.7):
        if not callable(publish_goal) or not callable(clock):
            raise ValueError("publish_goal and clock must be callable")
        if (not isinstance(frame_id, str) or not frame_id or frame_id.startswith("/") or
                len(frame_id) > 128 or not _vector(bounds, 6) or
                any(bounds[i] >= bounds[i+1] for i in (0, 2, 4))):
            raise ValueError("Invalid planning frame or bounds")
        settings = (lookahead, goal_seen_radius, ttl_seconds, freshness_seconds, refresh_hz, waypoint_radius)
        if (not all(_finite(v) and v > 0 for v in settings) or
                lookahead <= goal_seen_radius+.1 or lookahead > 20 or
                not .1 <= ttl_seconds <= 2 or not .1 <= freshness_seconds <= 10 or
                not .1 <= refresh_hz <= 10 or not isinstance(enabled, bool)):
            raise ValueError("Invalid control limits; lookahead must exceed goal_seen_radius")
        if not isinstance(waypoints, (list, tuple)) or len(waypoints) > 1000:
            raise ValueError("Provide at most 1000 waypoints")
        self.bounds = tuple(bounds)
        if not all(_vector(p, 3) and self._in_bounds(p) for p in waypoints):
            raise ValueError("Waypoints must be finite and within planning bounds")
        self.publish_goal, self.clock = publish_goal, clock
        self.waypoints, self.enabled, self.frame_id = tuple(tuple(float(v) for v in p) for p in waypoints), enabled, frame_id
        self.authority_ready = True
        self.lookahead, self.goal_seen_radius = lookahead, goal_seen_radius
        self.ttl, self.freshness, self.period, self.waypoint_radius = ttl_seconds, freshness_seconds, 1/refresh_hz, waypoint_radius
        self.lock = threading.RLock()
        self.epoch, self.lease, self.index = 0, None, 0
        self.position, self.yaw = None, 0.0
        self.received = {"pose": None, "map": None}
        self.source_stamp = {"pose": -1.0, "map": -1.0}
        self.last_goal, self.last_publish, self.last_steer_position = None, -math.inf, None
        self.mode, self.reason, self.fault = "waiting", "Configure and enable exploration waypoints", None

    def _in_bounds(self, position):
        return all(self.bounds[2*i] <= position[i] <= self.bounds[2*i+1] for i in range(3))

    def _ready(self, now):
        return (not self.fault and self.authority_ready and self.enabled and bool(self.waypoints) and self.position is not None
                and all(t is not None and 0 <= now-t <= self.freshness for t in self.received.values()))

    def update_pose(self, position, orientation, stamp, frame_id, source_age_seconds=0.):
        with self.lock:
            if (not _vector(position, 3) or not self._in_bounds(position) or not _vector(orientation, 4)
                    or not _finite(stamp) or stamp <= self.source_stamp["pose"] or stamp <= 0
                    or frame_id != self.frame_id or not _finite(source_age_seconds)
                    or not 0 <= source_age_seconds <= self.freshness):
                return False
            norm = math.hypot(*orientation)
            if not math.isfinite(norm) or norm < 1e-9:
                return False
            x, y, z, w = (v/norm for v in orientation)
            self.position = tuple(float(v) for v in position)
            self.yaw = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
            self.source_stamp["pose"], self.received["pose"] = stamp, self.clock()-source_age_seconds
            return True

    def update_map(self, stamp, frame_id, source_age_seconds=0.):
        with self.lock:
            if (not _finite(stamp) or stamp <= 0 or stamp <= self.source_stamp["map"] or frame_id != self.frame_id
                    or not _finite(source_age_seconds) or not 0 <= source_age_seconds <= self.freshness):
                return False
            self.source_stamp["map"], self.received["map"] = stamp, self.clock()-source_age_seconds
            return True

    def set_authority_ready(self, available):
        """ROS adapter gates admission while another destination writer exists."""
        if not isinstance(available, bool):
            return False
        with self.lock:
            self.authority_ready = available
            if not available and self.lease:
                self._end_lease("Destination publisher conflict; steering authority withdrawn")
            return True

    def status(self, *, owner_id=None, accepted=None, request_id=None, reason=None):
        with self.lock:
            return {"type": "control_status", "protocol_version": 1, "epoch": self.epoch,
                    "mode": self.mode, "ready": bool(self._ready(self.clock())),
                    "lease_id": self.lease["id"] if self.lease else None,
                    "active_request_id": self.lease["request_id"] if self.lease else None,
                    "last_request_id": request_id, "accepted": accepted,
                    "reason": reason if reason is not None else self.reason,
                    "ttl_ms": round(self.ttl*1000), "owner_id": owner_id,
                    "capabilities": {"steering": True, "pause": False}}

    def _publish(self, target, *, force=False):
        if not force and self.last_goal is not None and math.dist(target, self.last_goal) <= .12:
            return False
        try:
            self.publish_goal(tuple(target), self.frame_id)
        except Exception as error:
            self.fault = f"Goal publication failed: {type(error).__name__}"
            self.mode, self.reason = "fault", self.fault
            if self.lease:
                self.lease = None
                self.epoch += 1
            return False
        self.last_goal, self.last_publish = tuple(target), self.clock()
        return True

    def _autonomous(self, *, force=False, reason="Autonomous waypoint exploration"):
        if not self._ready(self.clock()):
            self.mode = "fault" if self.fault else "waiting"
            self.reason = self.fault or ("Waiting for exclusive destination ownership; prior SANDO goal may remain active"
                                         if not self.authority_ready else
                                         "Waiting for enabled waypoints and fresh map/pose; prior SANDO goal may remain active")
            return
        # Advance through nearby points, bounded even if every waypoint is near.
        for _ in range(len(self.waypoints)):
            if math.dist(self.position, self.waypoints[self.index]) > self.waypoint_radius:
                break
            self.index = (self.index+1) % len(self.waypoints)
        self.mode, self.reason = "autonomous", reason
        self._publish(self.waypoints[self.index], force=force)

    def _end_lease(self, reason):
        self.lease = None
        self.epoch += 1
        self.last_steer_position = None
        self._autonomous(force=True, reason=reason)

    def _target(self, direction):
        dx, dy = direction
        target = (self.position[0]+dx*self.lookahead,
                  self.position[1]+dy*self.lookahead, self.position[2])
        target = tuple(max(self.bounds[2*i], min(self.bounds[2*i+1], v)) for i, v in enumerate(target))
        return target if math.dist(self.position, target) > self.goal_seen_radius+.1 else None

    def tick(self):
        with self.lock:
            now = self.clock()
            if self.lease and now >= self.lease["expires"]:
                self._end_lease("Steering expired; autonomous exploration resumed")
                return
            if not self._ready(now):
                if self.lease:
                    self._end_lease("Steering canceled because telemetry became stale")
                else:
                    self._autonomous()
                return
            if not self.lease:
                self._autonomous()
                return
            self.mode = "steering"
            if now-self.last_publish < self.period or (self.last_steer_position is not None and
                                                       math.dist(self.position, self.last_steer_position) < .15):
                return
            target = self._target(self.lease["direction"])
            if target is None:
                self._end_lease("Steering reached configured bounds; autonomous exploration resumed")
                return
            if self._publish(target):
                self.last_steer_position = self.position

    def disconnect(self, owner_id):
        with self.lock:
            if self.lease and self.lease["owner"] == owner_id:
                self._end_lease("Operator disconnected; autonomous exploration resumed")

    def handle(self, owner_id, message):
        with self.lock:
            self.tick()
            request = message.get("request_id") if isinstance(message, dict) else None
            def reply(ok, reason=None):
                return self.status(owner_id=owner_id, accepted=ok, request_id=request if isinstance(request, str) else None, reason=reason)
            if not isinstance(owner_id, str) or not owner_id or len(owner_id) > 128 or not isinstance(message, dict):
                return reply(False, "Invalid owner or message")
            if message.get("protocol_version") != 1 or isinstance(message.get("protocol_version"), bool):
                return reply(False, "Unsupported control protocol")
            kind = message.get("type")
            if kind not in ("steer_start", "steer_renew", "steer_release", "steer_cancel", "pause", "resume"):
                return reply(False, "Unsupported command")
            if not _integer(message.get("epoch")) or message["epoch"] != self.epoch:
                return reply(False, "Stale control epoch; read current status")
            if kind in ("pause", "resume"):
                return reply(False, "Pause/Resume unavailable: SANDO Stop/Hold backend is not implemented")
            if kind in ("steer_start", "steer_cancel") and (not isinstance(request, str) or not re.fullmatch(r"[A-Za-z0-9_.:-]{1,96}", request)):
                return reply(False, "A unique request_id is required")
            if kind == "steer_cancel":
                if self.lease:
                    if self.lease["owner"] != owner_id or self.lease["request_id"] != request:
                        return reply(False, "Cancel does not own the active steering request")
                    self._end_lease("Steering canceled; autonomous exploration resumed")
                else:
                    # Epoch invalidates a start with this request that is still in flight.
                    self.epoch += 1
                    self._autonomous(force=True)
                return reply(True)
            if kind == "steer_start":
                if self.lease:
                    return reply(False, "Steering already has an active owner")
                direction = message.get("direction")
                if not isinstance(direction, str) or direction not in self.DIRECTIONS:
                    return reply(False, "Invalid steering direction")
                if not self._ready(self.clock()):
                    return reply(False, "Fresh pose/map and enabled exploration waypoints are required")
                dx, dy = self.DIRECTIONS[direction]
                c, s = math.cos(self.yaw), math.sin(self.yaw)
                world = (c*dx-s*dy, s*dx+c*dy)
                target = self._target(world)
                if target is None:
                    return reply(False, "Insufficient configured space for the steering lookahead")
                self.lease = {"id": secrets.token_urlsafe(18), "owner": owner_id, "request_id": request,
                              "direction": world, "expires": self.clock()+self.ttl, "seq": 0}
                self.mode, self.reason = "steering", "Operator steering active"
                self.last_steer_position = self.position
                if not self._publish(target, force=True):
                    return reply(False, self.reason)
                return reply(True)
            if (not self.lease or self.lease["owner"] != owner_id or message.get("lease_id") != self.lease["id"]):
                return reply(False, "Steering lease is unavailable or belongs to another operator")
            if not _integer(message.get("seq")) or message["seq"] <= self.lease["seq"]:
                return reply(False, "Out-of-order steering sequence")
            self.lease["seq"] = message["seq"]
            if kind == "steer_renew":
                self.lease["expires"] = self.clock()+self.ttl
                return reply(True)
            self._end_lease("Direction released; autonomous exploration resumed")
            return reply(True)
