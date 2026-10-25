"""Bounded frontier exploration from timestamped range-sensor observations.

This chooses destinations; SANDO still owns trajectory feasibility and collision
avoidance. Unknown space is never inferred to be free from an occupied-only
snapshot. Free evidence comes exclusively from traversed, finite sensor rays.
"""

from collections import deque
import copy
from dataclasses import dataclass
import math
import threading
import time

UNKNOWN, FREE, OCCUPIED = 0, 1, 2
NEIGHBORS = ((1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1))


def finite_vector(value, size):
    try:
        return (isinstance(value, (list, tuple)) and len(value) == size and
                all(isinstance(v, (float, int)) and not isinstance(v, bool) and
                    math.isfinite(v) for v in value))
    except OverflowError:
        return False


@dataclass(frozen=True)
class ExplorationDecision:
    state: str
    reason: str
    goal: tuple = None


class VoxelEvidence:
    """Mission-bounded, persistent in-memory free/occupied/unknown evidence."""

    def __init__(self, bounds, resolution=.5, max_voxels=100000):
        if (not finite_vector(bounds, 6) or
                any(bounds[i] >= bounds[i + 1] for i in (0, 2, 4)) or
                not isinstance(resolution, (int, float)) or isinstance(resolution, bool) or
                not math.isfinite(resolution) or resolution <= 0 or
                not isinstance(max_voxels, int) or isinstance(max_voxels, bool) or
                not 1 <= max_voxels <= 500000):
            raise ValueError("Invalid exploration grid configuration")
        self.bounds, self.resolution = tuple(bounds), float(resolution)
        self.shape = tuple(math.ceil((bounds[2*i+1] - bounds[2*i])/resolution) for i in range(3))
        if math.prod(self.shape) > max_voxels:
            raise ValueError("Exploration bounds/resolution exceed max_voxels; use a smaller mission region")
        self.cells = bytearray(math.prod(self.shape))
        self.revision, self.scan_sequence = 0, 0

    def valid(self, cell):
        return all(0 <= cell[i] < self.shape[i] for i in range(3))

    def key(self, point):
        if not finite_vector(point, 3):
            return None
        if any(not self.bounds[2*i] <= point[i] < self.bounds[2*i+1] for i in range(3)):
            return None
        return tuple(math.floor((point[i] - self.bounds[2*i])/self.resolution) for i in range(3))

    def index(self, cell):
        return (cell[0]*self.shape[1] + cell[1])*self.shape[2] + cell[2]

    def value(self, cell):
        return self.cells[self.index(cell)] if self.valid(cell) else UNKNOWN

    def center(self, cell):
        return tuple(self.bounds[2*i] + (cell[i] + .5)*self.resolution for i in range(3))

    def neighbors(self, cell, horizontal=False):
        for offset in NEIGHBORS[:4] if horizontal else NEIGHBORS:
            candidate = tuple(cell[i] + offset[i] for i in range(3))
            if self.valid(candidate):
                yield candidate

    def ray(self, origin, endpoint):
        """3D voxel traversal; outside endpoints stop at the mission boundary."""
        cell = self.key(origin)
        if cell is None or not finite_vector(endpoint, 3):
            return
        delta = tuple(endpoint[i] - origin[i] for i in range(3))
        cell = list(cell)
        step, next_t, increments = [], [], []
        for i, direction in enumerate(delta):
            s = 1 if direction > 0 else -1 if direction < 0 else 0
            step.append(s)
            edge = self.bounds[2*i] + (cell[i] + (1 if s > 0 else 0))*self.resolution
            next_t.append(max(0., (edge - origin[i])/direction) if s else math.inf)
            increments.append(self.resolution/abs(direction) if s else math.inf)
        # Each step crosses a grid boundary, so traversal has a strict bound.
        for _ in range(sum(self.shape) + 3):
            if not self.valid(cell):
                break
            yield tuple(cell)
            crossing = min(next_t)
            if crossing > 1. or not math.isfinite(crossing):
                break
            for i in range(3):
                if next_t[i] <= crossing + 1e-12:
                    cell[i] += step[i]
                    next_t[i] += increments[i]

    def integrate(self, origin, endpoints, sensor_range):
        """Finite returns mark hits; ray interiors mark observed free space.

        Hit evidence wins over free evidence within the same scan. Points beyond
        sensor_range are discarded, not interpreted as empty max-range rays.
        """
        if self.key(origin) is None:
            return False
        free, occupied, rays = set(), set(), 0
        traversed_count = 0
        for endpoint in endpoints:
            if not finite_vector(endpoint, 3):
                continue
            distance = math.dist(origin, endpoint)
            if distance < self.resolution*.1 or distance > sensor_range:
                continue
            traversed = list(self.ray(origin, endpoint))
            traversed_count += len(traversed)
            if traversed_count > 100000:
                return False  # Reject atomically; never apply a partial scan.
            hit = self.key(endpoint)
            if not traversed:
                continue
            rays += 1
            free.update(traversed)
            if hit is not None:
                occupied.add(hit)
        if not rays:
            return False
        changed = False
        for value, cells in ((FREE, free - occupied), (OCCUPIED, occupied)):
            for cell in cells:
                index = self.index(cell)
                if self.cells[index] != value:
                    self.cells[index] = value
                    changed = True
        self.scan_sequence += 1
        if changed:
            self.revision += 1
        return True


class FrontierExplorer:
    """ROS-independent destination provider for SteeringSupervisor.

    Flight at one altitude is the conservative default, matching SANDO's
    force_goal_z configuration. Set altitude=None only for deliberately
    configured 3D flight. Completion is limited to the configured mission region
    and currently observed connected free space; residual unknown is reported.
    """

    def __init__(self, *, bounds, resolution=.5, clearance=.3, sensor_range=10.,
                 altitude=2., arrival_radius=.6, settle_seconds=2.,
                 progress_timeout=20., retry_seconds=10., max_attempts=2,
                 confirmation_scans=3, freshness_seconds=2., max_voxels=100000,
                 max_candidates=128, max_rays=1200, clock=time.monotonic):
        self.grid = VoxelEvidence(bounds, resolution, max_voxels)
        positive = (sensor_range, arrival_radius, settle_seconds, progress_timeout,
                    retry_seconds, freshness_seconds)
        if (not all(isinstance(v, (int, float)) and not isinstance(v, bool) and
                    math.isfinite(v) and v > 0 for v in positive) or
                not isinstance(clearance, (float, int)) or isinstance(clearance, bool) or
                not math.isfinite(clearance) or not 0 <= clearance <= 3 or
                not callable(clock) or progress_timeout <= settle_seconds or
                sensor_range > 100 or
                any(not isinstance(v, int) or isinstance(v, bool) for v in
                    (max_attempts, confirmation_scans, max_candidates, max_rays)) or
                not 1 <= max_attempts <= 10 or not 2 <= confirmation_scans <= 100 or
                not 1 <= max_candidates <= 256 or not 1 <= max_rays <= 5000 or
                clearance/resolution > 6 or sensor_range/resolution > 512):
            raise ValueError("Invalid frontier exploration limits")
        if altitude is not None and (not isinstance(altitude, (int, float)) or isinstance(altitude, bool) or
                                     not math.isfinite(altitude) or not bounds[4] < altitude < bounds[5]):
            raise ValueError("Flight altitude must be inside exploration bounds")
        self.clearance, self.sensor_range, self.altitude = clearance, sensor_range, altitude
        self.arrival_radius, self.settle = arrival_radius, settle_seconds
        self.timeout, self.retry, self.max_attempts = progress_timeout, retry_seconds, max_attempts
        self.confirmations, self.freshness = confirmation_scans, freshness_seconds
        self.max_candidates, self.max_rays, self.clock = max_candidates, max_rays, clock
        self.last_scan, self.last_source_stamp = None, -math.inf
        self.active, self.failures = None, {}
        self.confirm_count, self.last_confirm_sequence = 0, -1
        self.plan_at, self.last_decision = -math.inf, ExplorationDecision("waiting", "Waiting for range-sensor observations")
        self.frontier_count, self.reachable_count = 0, 0
        self._failure_revision = 0
        self.searcher = None  # ROS adapter supplies an asynchronous latest-only search worker.
        radius = math.ceil(clearance/resolution)
        # Cube envelope is deliberately conservative for a vehicle body.
        self.offsets = tuple((x, y, z) for x in range(-radius, radius + 1)
                             for y in range(-radius, radius + 1) for z in range(-radius, radius + 1))

    def ready(self, now=None):
        now = self.clock() if now is None else now
        return self.last_scan is not None and 0 <= now - self.last_scan <= self.freshness

    def observe(self, origin, endpoints, stamp, source_age_seconds=0.):
        if (not isinstance(stamp, (int, float)) or isinstance(stamp, bool) or
                not math.isfinite(stamp) or stamp <= 0 or stamp <= self.last_source_stamp or
                not isinstance(source_age_seconds, (int, float)) or isinstance(source_age_seconds, bool) or
                not math.isfinite(source_age_seconds) or not 0 <= source_age_seconds <= self.freshness or
                not isinstance(endpoints, (list, tuple)) or len(endpoints) > self.max_rays):
            return False
        if not self.grid.integrate(origin, endpoints, self.sensor_range):
            return False
        self.last_scan, self.last_source_stamp = self.clock() - source_age_seconds, stamp
        return True

    def interrupt(self):
        """Manual control/waiting suspend autonomy without charging a failed goal."""
        self.active = None
        self.confirm_count = 0
        self.plan_at = -math.inf
        self.last_decision = ExplorationDecision("suspended", "Autonomous frontier selection is suspended")
        if self.searcher is not None:
            self.searcher.invalidate()

    def publication_failed(self):
        self.interrupt()

    def _safe(self, cell, cache):
        if cell not in cache:
            cache[cell] = all(self.grid.value(tuple(cell[i] + offset[i] for i in range(3))) == FREE
                              for offset in self.offsets)
        return cache[cell]

    def _component(self, start, safe=False, cache=None):
        if self.grid.value(start) != FREE or (safe and not self._safe(start, cache)):
            return {}
        distance, queue = {start: 0}, deque([start])
        while queue:
            cell = queue.popleft()
            for neighbor in self.grid.neighbors(cell, horizontal=self.altitude is not None):
                if (neighbor not in distance and self.grid.value(neighbor) == FREE and
                        (not safe or self._safe(neighbor, cache))):
                    distance[neighbor] = distance[cell] + 1
                    queue.append(neighbor)
        return distance

    def _frontiers(self, component):
        # All face-adjacent unknown counts, including a single residual cell.
        # Fixed-altitude mode explores horizontal frontiers only.
        remaining = {cell for cell in component if any(self.grid.value(n) == UNKNOWN
                     for n in self.grid.neighbors(cell, horizontal=self.altitude is not None))}
        self.frontier_count = len(remaining)
        clusters = []
        for seed in sorted(remaining):
            if seed not in remaining:
                continue
            remaining.remove(seed)
            queue, cluster = deque([seed]), [seed]
            while queue:
                for neighbor in self.grid.neighbors(queue.popleft(), horizontal=self.altitude is not None):
                    if neighbor in remaining:
                        remaining.remove(neighbor)
                        queue.append(neighbor)
                        cluster.append(neighbor)
            clusters.append(sorted(cluster))
        return clusters

    def _visible_unknown(self, candidate, samples):
        visible = set()
        origin = self.grid.center(candidate)
        if self.altitude is not None:
            origin = (origin[0], origin[1], self.altitude)
        for frontier in samples:
            for unknown in self.grid.neighbors(frontier, horizontal=self.altitude is not None):
                if self.grid.value(unknown) != UNKNOWN:
                    continue
                end = self.grid.center(unknown)
                if math.dist(origin, end) > self.sensor_range:
                    continue
                for cell in self.grid.ray(origin, end):
                    value = self.grid.value(cell)
                    if value == OCCUPIED:
                        break
                    if value == UNKNOWN:
                        visible.add(cell)
                        break
        return len(visible)

    def _candidates(self, clusters, reachable, position):
        candidates, seen = [], set()
        # Prioritize large nearby clusters; a finite candidate budget bounds work.
        clusters.sort(key=lambda cluster: (-len(cluster), math.dist(position, self.grid.center(cluster[0]))))
        search_budget = self.max_candidates*200
        for cluster in clusters:
            sample_stride = max(1, math.ceil(len(cluster)/24))
            samples = cluster[::sample_stride]
            # Search around distributed frontier samples, not just the cluster
            # centroid (which may sit in an already observed room center).
            near = set()
            for sample in samples:
                queue, visited, found = deque([sample]), {sample}, 0
                local_budget = min(400, search_budget)
                while queue and local_budget > 0 and found < 4:
                    cell = queue.popleft()
                    local_budget -= 1
                    search_budget -= 1
                    if cell in reachable and cell not in seen and cell not in near:
                        near.add(cell)
                        found += 1
                    for neighbor in self.grid.neighbors(cell, horizontal=self.altitude is not None):
                        if (neighbor not in visited and self.grid.value(neighbor) == FREE and
                                math.dist(self.grid.center(neighbor), self.grid.center(sample)) <= self.sensor_range):
                            visited.add(neighbor)
                            queue.append(neighbor)
            for candidate in sorted(near):
                if candidate in seen:
                    continue
                seen.add(candidate)
                gain = self._visible_unknown(candidate, samples)
                if not gain:
                    continue
                point = self.grid.center(candidate)
                if self.altitude is not None:
                    point = (point[0], point[1], self.altitude)
                # Prefer a new view over repeatedly selecting the current pose.
                travel = reachable[candidate]*self.grid.resolution
                score = gain/(1. + travel) - (2. if math.dist(position, point) <= self.arrival_radius else 0.)
                candidates.append((score, candidate, point))
                if len(candidates) >= self.max_candidates:
                    return sorted(candidates, reverse=True)
            if search_budget <= 0:
                break
        return sorted(candidates, reverse=True)

    def search(self, position):
        """Pure search on this grid version; async worker calls it on a copy."""
        start, cache = self.grid.key(position), {}
        component = self._component(start)
        reachable = self._component(start, safe=True, cache=cache)
        clusters = self._frontiers(component)
        return {"frontier_count": self.frontier_count, "reachable_count": len(reachable),
                "cluster_count": len(clusters),
                "candidates": self._candidates(clusters, reachable, position) if clusters else []}

    def _decision(self, state, reason, goal=None):
        self.last_decision = ExplorationDecision(state, reason, goal)
        return self.last_decision

    def _fail_active(self, now):
        cell = self.active["cell"]
        count, _ = self.failures.get(cell, (0, 0.))
        self.failures[cell] = (count + 1, now + self.retry)
        self.active = None
        self.plan_at = -math.inf

    def next_goal(self, position):
        now = self.clock()
        if not self.ready(now) or not finite_vector(position, 3):
            self.interrupt()
            return self._decision("waiting", "Waiting for fresh timestamped range observations")
        start = self.grid.key(position)
        if start is None or (self.altitude is not None and abs(position[2] - self.altitude) > self.arrival_radius):
            self.interrupt()
            return self._decision("waiting", "Pose is outside the mission region or configured flight altitude")
        cache = {}
        if self.active:
            target = self.active
            if not self._safe(target["cell"], cache):
                self._fail_active(now)
            else:
                distance = math.dist(position, target["goal"])
                if distance + .2 < target["best_distance"]:
                    target["best_distance"], target["progress_at"] = distance, now
                if distance <= self.arrival_radius:
                    if target["arrived_at"] is None:
                        target["arrived_at"], target["arrived_scan"] = now, self.grid.scan_sequence
                    if now - target["arrived_at"] < self.settle or self.grid.scan_sequence <= target["arrived_scan"]:
                        return self._decision("observing", "At frontier viewpoint; waiting for a new sensor observation", target["goal"])
                    # Re-evaluate frontiers after sensing, never mark unknown as
                    # covered merely because a destination was reached.
                    self._fail_active(now)
                elif now - target["progress_at"] >= self.timeout:
                    self._fail_active(now)
                else:
                    return self._decision("exploring", "Navigating to the next frontier viewpoint", target["goal"])
        if now - self.plan_at < 1.:
            return self.last_decision
        self.plan_at = now
        if not self._safe(start, cache):
            return self._decision("waiting", "Insufficient observed free clearance around the current pose")
        search = self.searcher.request(self, position) if self.searcher is not None else self.search(position)
        if search is None:
            return self._decision("planning", "Computing frontier viewpoints in the background")
        self.reachable_count, self.frontier_count = search["reachable_count"], search["frontier_count"]
        if not search["cluster_count"]:
            if self.grid.scan_sequence != self.last_confirm_sequence:
                self.last_confirm_sequence = self.grid.scan_sequence
                self.confirm_count += 1
            if self.confirm_count >= self.confirmations:
                return self._decision("complete", "No reachable frontiers remain in the configured mission region; residual unknown is not certified explored")
            return self._decision("confirming", "Rechecking frontier exhaustion with distinct sensor observations")
        self.confirm_count = 0
        # A genuine map change allows previously failed destinations to be
        # reconsidered (for example, a newly opened doorway). No-change scans
        # alone do not reset the retry budget.
        if self.grid.revision != self._failure_revision:
            self.failures.clear()
            self._failure_revision = self.grid.revision
        for _, cell, point in search["candidates"]:
            attempts, after = self.failures.get(cell, (0, -math.inf))
            if attempts >= self.max_attempts or now < after:
                continue
            self.active = {"cell": cell, "goal": point, "best_distance": math.dist(position, point),
                           "progress_at": now, "arrived_at": None, "arrived_scan": None}
            return self._decision("exploring", "Selected a reachable known-free frontier viewpoint", point)
        return self._decision("blocked", "Unknown frontiers remain, but candidate viewpoints lack clearance or exhausted their progress/retry budget; waiting for map changes")

    def status(self):
        return {"mode": "frontier", "state": self.last_decision.state,
                "reason": self.last_decision.reason, "map_revision": self.grid.revision,
                "scan_sequence": self.grid.scan_sequence, "frontier_cells": self.frontier_count,
                "reachable_cells": self.reachable_count, "unknown_voxels": self.grid.cells.count(UNKNOWN),
                "free_voxels": self.grid.cells.count(FREE), "occupied_voxels": self.grid.cells.count(OCCUPIED),
                "goal": list(self.active["goal"]) if self.active else None,
                "failed_viewpoints": len(self.failures),
                "completion_scope": "configured bounds, horizontal slice" if self.altitude is not None else "configured bounds, 3D"}


class BackgroundFrontierSearch:
    """One search worker and one pending immutable grid snapshot.

    Slow graph/viewpoint searches never hold the supervisor's ownership lock or
    run in its lease watchdog. Results are accepted only for the current grid
    revision, pose voxel, and control generation. Interrupting manual authority
    invalidates any in-flight result rather than reviving its old destination.
    """

    def __init__(self):
        self.lock = threading.Lock()
        self.event, self.stopped = threading.Event(), threading.Event()
        self.generation, self.signature = 0, None
        self.pending, self.result, self.error = None, None, None
        self.thread = threading.Thread(target=self._run, name="frontier-search", daemon=True)

    def start(self):
        self.thread.start()

    def invalidate(self):
        with self.lock:
            self.generation += 1
            self.signature, self.pending, self.result, self.error = None, None, None, None

    def request(self, explorer, position):
        with self.lock:
            signature = (self.generation, explorer.grid.revision, explorer.grid.key(position))
            if signature == self.signature:
                if self.error:
                    raise RuntimeError("Background frontier search failed") from self.error
                return self.result
            snapshot = copy.copy(explorer)
            snapshot.grid = copy.copy(explorer.grid)
            snapshot.grid.cells = explorer.grid.cells.copy()
            snapshot.searcher = None
            self.signature, self.result, self.error = signature, None, None
            self.pending = (signature, snapshot, tuple(position))
            self.event.set()
        return None

    def _run(self):
        while not self.stopped.is_set():
            self.event.wait(.1)
            self.event.clear()
            with self.lock:
                job, self.pending = self.pending, None
            if job is None:
                continue
            signature, snapshot, position = job
            try:
                result, error = snapshot.search(position), None
            except Exception as caught:
                result, error = None, caught
            with self.lock:
                if self.signature == signature:
                    self.result, self.error = result, error

    def close(self):
        self.stopped.set()
        self.event.set()
        self.thread.join(timeout=2.)
