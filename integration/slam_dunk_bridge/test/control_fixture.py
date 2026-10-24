"""Test-only sidecar and fake destination sink for the JavaScript wire smoke."""
import json
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from slam_dunk_bridge.command_server import CommandServer
from slam_dunk_bridge.control import SteeringSupervisor


def main():
    goals = []
    supervisor = SteeringSupervisor(lambda target, frame: goals.append(list(target)),
                                    waypoints=[[10., 10., 2.]], enabled=True)
    server = CommandServer(supervisor, port=0)
    stop = threading.Event()

    def update():
        while not stop.is_set():
            stamp = time.time()
            supervisor.update_pose([0., 0., 2.], [0., 0., 0., 1.], stamp, "map")
            supervisor.update_map(stamp, "map")
            supervisor.tick()
            stop.wait(.02)

    worker = threading.Thread(target=update, daemon=True)
    worker.start()
    server.start()
    print(json.dumps({"port": server.port}), flush=True)
    try:
        sys.stdin.readline()
    finally:
        server.stop()
        stop.set()
        worker.join(timeout=2)
    print(json.dumps({"goals": goals, "mode": supervisor.mode}), flush=True)


if __name__ == "__main__":
    main()
