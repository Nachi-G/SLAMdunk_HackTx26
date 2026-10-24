"""Synthetic room demo: no ROS/Gazebo required; not a sensor validation."""

import argparse
import math
import time

from .core import LatestStore, MapPublisher, health_message
from .transport import BridgeServer


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    args = parser.parse_args()
    store = LatestStore()
    maps = MapPublisher(store)
    server = BridgeServer(store, args.host, args.port)
    server.start()
    print(f"Synthetic demo: http://{args.host}:{args.port}/ (Ctrl+C to stop)", flush=True)
    start, last_map, last_health = time.monotonic(), -1, -1
    try:
        while True:
            elapsed = time.monotonic()-start
            now = time.time_ns()
            stamp = {"sec": now//1000000000, "nanosec": now % 1000000000}
            yaw = elapsed*0.35
            store.publish({"type": "pose", "frame_id": "map", "child_frame_id": "demo/drone",
                           "stamp": stamp, "source": "synthetic_demo",
                           "position": [2*math.cos(yaw), 1.5*math.sin(yaw), 1.2],
                           "orientation": [0,0,math.sin(yaw/2),math.cos(yaw/2)]})
            if int(elapsed*2) != last_map:
                last_map = int(elapsed*2)
                # Increasing visibility illustrates replacements, not scan accumulation.
                visible = min(160, 20+int(elapsed*8))
                points = []
                for i in range(visible):
                    a = i/160*2*math.pi
                    x, y = 4*math.cos(a), 3*math.sin(a)
                    for j in range(13):
                        points.append((x,y,j*0.2))
                maps.replace(points, stamp, "synthetic_demo")
            if int(elapsed) != last_health:
                last_health = int(elapsed)
                store.publish(dict(health_message({"map": time.monotonic(), "pose": time.monotonic()},
                                                   {}, "map"), mode="synthetic_demo"))
            if server.error:
                raise RuntimeError("Demo transport failed") from server.error
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        server.stop()


if __name__ == "__main__":
    main()
