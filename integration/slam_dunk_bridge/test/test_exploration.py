"""Deterministic exploration/evidence tests; no ROS or real-flight claim."""

import math
import threading
import unittest
from unittest.mock import patch

from slam_dunk_bridge.control import SteeringSupervisor
from slam_dunk_bridge.exploration import (FREE, OCCUPIED, UNKNOWN, BackgroundFrontierSearch,
                                          FrontierExplorer, VoxelEvidence)


class Clock:
    def __init__(self):
        self.now = 100.

    def __call__(self):
        return self.now

    def advance(self, seconds=1.1):
        self.now += seconds


def explorer_fixture(**kwargs):
    clock = Clock()
    defaults = dict(bounds=(0, 12, 0, 8, 0, 3), resolution=1., clearance=0.,
                    altitude=1.5, sensor_range=20., settle_seconds=.2,
                    progress_timeout=3., retry_seconds=1., arrival_radius=.4,
                    freshness_seconds=2., clock=clock)
    defaults.update(kwargs)
    explorer = FrontierExplorer(**defaults)
    return explorer, clock


def seed(explorer, free=(), occupied=(), unknown=()):
    for value, cells in ((FREE, free), (OCCUPIED, occupied), (UNKNOWN, unknown)):
        for cell in cells:
            explorer.grid.cells[explorer.grid.index(cell)] = value
    explorer.grid.revision += 1
    refresh(explorer)


def refresh(explorer):
    explorer.grid.scan_sequence += 1
    explorer.last_scan = explorer.clock()


def room(explorer, x0=0, x1=12, y0=0, y1=8):
    free, occupied = [], []
    for x in range(x0, x1):
        for y in range(y0, y1):
            target = occupied if x in (x0, x1-1) or y in (y0, y1-1) else free
            target.append((x, y, 1))
    seed(explorer, free, occupied)


class EvidenceTests(unittest.TestCase):
    def test_only_ray_interiors_become_free_and_hits_win(self):
        grid = VoxelEvidence((0, 6, 0, 6, 0, 3), 1.)
        self.assertTrue(grid.integrate((.5, .5, 1.5), [(3.5, .5, 1.5), (5.5, .5, 1.5)], 10))
        self.assertEqual(grid.value((1, 0, 1)), FREE)
        self.assertEqual(grid.value((3, 0, 1)), OCCUPIED)
        self.assertEqual(grid.value((5, 0, 1)), OCCUPIED)
        self.assertEqual(grid.value((1, 1, 1)), UNKNOWN)
        # Later direct ray evidence can clear an obstacle that moved away.
        grid.integrate((.5, .5, 1.5), [(5.5, .5, 1.5)], 10)
        self.assertEqual(grid.value((3, 0, 1)), FREE)

    def test_clipped_mission_boundary_does_not_create_fake_obstacles(self):
        grid = VoxelEvidence((0, 4, 0, 4, 0, 3), 1.)
        grid.integrate((.5, .5, 1.5), [(6.5, .5, 1.5)], 10)
        self.assertEqual(grid.value((3, 0, 1)), FREE)
        self.assertEqual(len(grid.cells), 48)
        self.assertEqual(grid.value((4, 0, 1)), UNKNOWN)

    def test_out_of_range_invalid_and_empty_scans_do_not_clear_or_refresh(self):
        explorer, clock = explorer_fixture(sensor_range=3.)
        self.assertFalse(explorer.ready())
        for endpoints in ([], [(math.nan, 0, 0)], [(8, .5, 1.5)], [(.5, .5, 1.5)]):
            self.assertFalse(explorer.observe((.5, .5, 1.5), endpoints, 10.))
        self.assertEqual(explorer.grid.cells.count(FREE), 0)
        self.assertIsNone(explorer.last_scan)
        self.assertTrue(explorer.observe((.5, .5, 1.5), [(2.5, .5, 1.5)], 11., .5))
        self.assertEqual(explorer.last_scan, clock()-.5)
        self.assertFalse(explorer.observe((.5, .5, 1.5), [(2.5, .5, 1.5)], 11.))
        self.assertFalse(explorer.observe((.5, .5, 1.5), [(2.5, .5, 1.5)], 12., 3.))
        clock.advance(1.6)
        self.assertFalse(explorer.ready())

    def test_observations_persist_outside_later_sensor_footprints(self):
        grid = VoxelEvidence((0, 12, 0, 4, 0, 3), 1.)
        grid.integrate((.5, .5, 1.5), [(3.5, .5, 1.5)], 5.)
        grid.integrate((8.5, .5, 1.5), [(11.5, .5, 1.5)], 5.)
        self.assertEqual(grid.value((1, 0, 1)), FREE)
        self.assertEqual(grid.value((9, 0, 1)), FREE)
        self.assertEqual(grid.value((6, 0, 1)), UNKNOWN)

    def test_grid_and_search_configuration_is_bounded(self):
        for kwargs in ({"bounds": (0, 1000, 0, 1000, 0, 1000)},
                       {"resolution": 0}, {"clearance": -1}, {"altitude": 20},
                       {"confirmation_scans": 1}, {"max_rays": 100000},
                       {"max_candidates": 100000}, {"progress_timeout": .1}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                explorer_fixture(**kwargs)


class FrontierTests(unittest.TestCase):
    def test_range_scan_mission_enters_second_room_without_preset_waypoints(self):
        explorer, clock = explorer_fixture()
        truth = VoxelEvidence(explorer.grid.bounds, 1.)
        obstacles = []
        for x in range(12):
            for y in range(8):
                for z in range(3):
                    occupied = (x in (0, 11) or y in (0, 7) or z in (0, 2)
                                or (x == 5 and y not in (3, 4)))
                    truth.cells[truth.index((x, y, z))] = OCCUPIED if occupied else FREE
                    if occupied:
                        obstacles.append(truth.center((x, y, z)))
        position, destinations = (2.5, 2.5, 1.5), []
        for _ in range(40):
            # Idealized finite sensor returns: only the first obstacle on a ray
            # is reported, so the second room is initially occluded.
            hits = set()
            for target in obstacles:
                for cell in truth.ray(position, target):
                    if truth.value(cell) == OCCUPIED:
                        hits.add(truth.center(cell))
                        break
            clock.advance(.6)
            self.assertTrue(explorer.observe(position, list(hits), clock()))
            decision = explorer.next_goal(position)
            if decision.goal is not None and math.dist(position, decision.goal) > .1:
                destinations.append(decision.goal)
                position = decision.goal  # Idealized motion, not flight dynamics.
            if decision.state == "complete":
                break
        self.assertEqual(decision.state, "complete")
        self.assertTrue(any(goal[0] > 5 for goal in destinations))
        self.assertTrue(all(truth.value(truth.key(goal)) == FREE for goal in destinations))

    def test_frontier_goal_is_known_free_and_not_across_closed_wall(self):
        explorer, _ = explorer_fixture()
        free = [(x, y, 1) for x in range(1, 5) for y in range(1, 7)]
        wall = [(5, y, 1) for y in range(8)]
        seed(explorer, free, wall)
        decision = explorer.next_goal((1.5, 3.5, 1.5))
        self.assertEqual(decision.state, "exploring")
        self.assertEqual(explorer.grid.value(explorer.grid.key(decision.goal)), FREE)
        self.assertLess(decision.goal[0], 5)

    def test_keeps_selecting_goals_as_new_room_is_revealed_then_completes(self):
        explorer, clock = explorer_fixture()
        # First room observed; its open doorway leads to an unknown second room.
        free = [(x, y, 1) for x in range(1, 5) for y in range(1, 7)]
        boundary = ([(0, y, 1) for y in range(8)] +
                    [(x, y, 1) for x in range(6) for y in (0, 7)] +
                    [(5, y, 1) for y in range(8) if y not in (3, 4)])
        seed(explorer, free, boundary)
        position = (2.5, 3.5, 1.5)
        first = explorer.next_goal(position)
        self.assertEqual(first.state, "exploring")
        self.assertIsNotNone(first.goal)
        position = first.goal
        clock.advance(.1); refresh(explorer)
        self.assertEqual(explorer.next_goal(position).state, "observing")
        # A real scan update reveals the doorway and part of the next room.
        seed(explorer, [(x, y, 1) for x in range(5, 8) for y in (3, 4)])
        clock.advance(.3); refresh(explorer)
        second = explorer.next_goal(position)
        self.assertEqual(second.state, "exploring")
        self.assertNotEqual(first.goal, second.goal)
        self.assertNotEqual(second.state, "complete")
        # After the second room is surveyed, no reachable horizontal frontier remains.
        room(explorer)
        position = second.goal
        explorer.interrupt()
        states = []
        for _ in range(4):
            clock.advance(); refresh(explorer)
            states.append(explorer.next_goal(position).state)
        self.assertEqual(states[:2], ["confirming", "confirming"])
        self.assertEqual(states[-1], "complete")
        # Untested vertical space remains explicitly unknown in slice mode.
        self.assertGreater(explorer.status()["unknown_voxels"], 0)
        self.assertIn("horizontal slice", explorer.status()["completion_scope"])

    def test_reaching_a_goal_does_not_mark_unknown_as_observed(self):
        explorer, clock = explorer_fixture()
        seed(explorer, [(x, y, 1) for x in range(1, 5) for y in range(1, 7)])
        goal = explorer.next_goal((1.5, 3.5, 1.5)).goal
        unknown = explorer.grid.cells.count(UNKNOWN)
        clock.advance(.1); refresh(explorer)
        explorer.next_goal(goal)
        clock.advance(.4); refresh(explorer)
        self.assertNotEqual(explorer.next_goal(goal).state, "complete")
        self.assertEqual(explorer.grid.cells.count(UNKNOWN), unknown)

    def test_distinct_sensor_observations_required_for_completion(self):
        explorer, clock = explorer_fixture()
        room(explorer)
        self.assertEqual(explorer.next_goal((2.5, 3.5, 1.5)).state, "confirming")
        clock.advance(1.1)
        self.assertEqual(explorer.next_goal((2.5, 3.5, 1.5)).state, "confirming")
        self.assertEqual(explorer.confirm_count, 1)
        clock.advance(1.1)
        self.assertEqual(explorer.next_goal((2.5, 3.5, 1.5)).state, "waiting")

    def test_disconnected_unknown_is_reported_not_falsely_counted_as_mapped(self):
        explorer, clock = explorer_fixture()
        room(explorer, x1=6)
        for _ in range(3):
            clock.advance(); refresh(explorer)
            decision = explorer.next_goal((2.5, 3.5, 1.5))
        self.assertEqual(decision.state, "complete")
        self.assertEqual(explorer.grid.value((9, 3, 1)), UNKNOWN)
        self.assertGreater(explorer.status()["unknown_voxels"], 0)
        self.assertIn("residual unknown", decision.reason)

    def test_failed_candidates_are_retried_then_blocked_never_completed(self):
        explorer, clock = explorer_fixture(max_candidates=1)
        seed(explorer, [(x, y, 1) for x in range(1, 5) for y in range(1, 7)])
        position = (1.5, 3.5, 1.5)
        self.assertIsNotNone(explorer.next_goal(position).goal)
        for _ in range(12):
            clock.advance(); refresh(explorer)
            decision = explorer.next_goal(position)
        self.assertEqual(decision.state, "blocked")
        self.assertEqual(explorer.status()["failed_viewpoints"], 1)
        self.assertGreater(explorer.frontier_count, 0)

    def test_map_change_reopens_completed_exploration(self):
        explorer, clock = explorer_fixture()
        room(explorer)
        for _ in range(3):
            clock.advance(); refresh(explorer)
            explorer.next_goal((2.5, 3.5, 1.5))
        seed(explorer, unknown=[(6, 4, 1)])
        clock.advance(); refresh(explorer)
        self.assertEqual(explorer.next_goal((2.5, 3.5, 1.5)).state, "exploring")

    def test_clearance_and_altitude_gate_destinations(self):
        explorer, _ = explorer_fixture(clearance=.2)
        # Centerline free alone doesn't certify clearance for the drone body.
        seed(explorer, [(x, 3, 1) for x in range(1, 10)])
        self.assertEqual(explorer.next_goal((2.5, 3.5, 1.5)).state, "waiting")
        self.assertIsNone(explorer.last_decision.goal)
        self.assertEqual(explorer.next_goal((2.5, 3.5, 2.5)).state, "waiting")

    def test_3d_mode_selects_vertical_frontier(self):
        explorer, _ = explorer_fixture(altitude=None)
        seed(explorer, [(x, y, 1) for x in range(1, 11) for y in range(1, 7)],
             [(x, y, z) for x in range(12) for y in range(8) for z in (0,)
              if x in (0, 11) or y in (0, 7)])
        self.assertEqual(explorer.next_goal((2.5, 3.5, 1.5)).state, "exploring")
        self.assertIn("3D", explorer.status()["completion_scope"])


class SupervisorExplorationTests(unittest.TestCase):
    def fixture(self):
        explorer, clock = explorer_fixture()
        seed(explorer, [(x, y, 1) for x in range(1, 5) for y in range(1, 7)])
        goals = []
        supervisor = SteeringSupervisor(lambda p, f: goals.append((p, f)), clock=clock,
            enabled=True, bounds=explorer.grid.bounds, autonomy_provider=explorer)
        supervisor.update_pose((2.5, 3.5, 1.5), (0, 0, 0, 1), 100., "map")
        supervisor.update_map(100., "map")
        supervisor.tick()
        return supervisor, explorer, clock, goals

    def test_manual_override_and_release_reselect_frontier(self):
        supervisor, explorer, _, goals = self.fixture()
        self.assertEqual(supervisor.status()["exploration"]["state"], "exploring")
        status = supervisor.handle("operator", {"type": "steer_start", "protocol_version": 1,
            "epoch": supervisor.epoch, "request_id": "press", "direction": "forward"})
        self.assertTrue(status["accepted"])
        self.assertIsNone(explorer.active)
        result = supervisor.handle("operator", {"type": "steer_release", "protocol_version": 1,
            "epoch": supervisor.epoch, "lease_id": status["lease_id"], "seq": 1})
        self.assertTrue(result["accepted"])
        self.assertEqual(result["mode"], "autonomous")
        self.assertEqual(result["exploration"]["state"], "exploring")
        self.assertEqual(goals[-1][0], explorer.active["goal"])

    def test_stale_ranges_with_fresh_nav_data_withdraw_new_goals(self):
        supervisor, explorer, clock, goals = self.fixture()
        before = len(goals)
        clock.advance(2.1)
        supervisor.update_pose((2.5, 3.5, 1.5), (0, 0, 0, 1), 101., "map")
        supervisor.update_map(101., "map")
        supervisor.tick()
        self.assertFalse(supervisor.status()["ready"])
        self.assertEqual(supervisor.status()["mode"], "waiting")
        self.assertIsNone(explorer.active)
        self.assertEqual(len(goals), before)

    def test_wrong_frame_observation_and_conflicting_autonomy_rejected(self):
        supervisor, explorer, _, _ = self.fixture()
        self.assertFalse(supervisor.update_observation((2.5, 3.5, 1.5), [(3.5, 3.5, 1.5)], 110., "sensor"))
        with self.assertRaises(ValueError):
            SteeringSupervisor(lambda *_: None, autonomy_provider=explorer, waypoints=[(1, 1, 1)])

    def test_slow_background_search_does_not_lock_out_manual_steering(self):
        supervisor, explorer, _, _ = self.fixture()
        explorer.interrupt()
        started, release, finished = threading.Event(), threading.Event(), threading.Event()
        original_search = FrontierExplorer.search

        def slow_search(snapshot, position):
            started.set()
            release.wait(2.)
            try:
                return original_search(snapshot, position)
            finally:
                finished.set()

        worker = BackgroundFrontierSearch()
        explorer.searcher = worker
        with patch.object(FrontierExplorer, "search", slow_search):
            worker.start()
            try:
                supervisor.tick()
                self.assertTrue(started.wait(1.))
                self.assertEqual(supervisor.status()["exploration"]["state"], "planning")
                response = supervisor.handle("operator", {"type": "steer_start", "protocol_version": 1,
                    "epoch": supervisor.epoch, "request_id": "during-search", "direction": "forward"})
                self.assertTrue(response["accepted"])
                self.assertFalse(finished.is_set())
                release.set()
                self.assertTrue(finished.wait(1.))
                self.assertEqual(supervisor.status()["mode"], "steering")
                self.assertIsNone(explorer.active)
                # A stale planning result cannot undo the manual generation.
                self.assertIsNone(worker.result)
            finally:
                release.set()
                worker.close()

    def test_background_search_rejects_old_map_results(self):
        explorer, _ = explorer_fixture()
        room(explorer)
        worker = BackgroundFrontierSearch()
        explorer.searcher = worker
        worker.start()
        try:
            position = (2.5, 3.5, 1.5)
            self.assertIsNone(worker.request(explorer, position))
            # Wait for a result without assuming a specific machine speed.
            for _ in range(100):
                result = worker.request(explorer, position)
                if result is not None:
                    break
                threading.Event().wait(.01)
            self.assertEqual(result["cluster_count"], 0)
            seed(explorer, unknown=[(6, 4, 1)])
            self.assertIsNone(worker.request(explorer, position))
            for _ in range(100):
                result = worker.request(explorer, position)
                if result is not None:
                    break
                threading.Event().wait(.01)
            self.assertGreater(result["cluster_count"], 0)
        finally:
            worker.close()


if __name__ == "__main__":
    unittest.main()
