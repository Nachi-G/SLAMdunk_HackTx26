"""Real HTTP/WebSocket smoke test against an already running bridge demo."""
import asyncio
import json
import sys
from urllib.request import urlopen


async def main(port):
    from websockets.legacy.client import connect
    base = f"http://127.0.0.1:{port}"
    for path, content in (("/?source=live", "text/html"), ("/styles.css", "text/css"),
                          *((f"/{name}.mjs", "text/javascript") for name in
                            ("app", "scene", "model", "adapter", "demo", "camera")),
                          ("/legacy", "text/html")):
        with urlopen(base + path, timeout=5) as response:
            assert response.status == 200
            assert response.headers.get_content_type() == content
            assert len(response.read()) > 0
    async with connect(f"ws://127.0.0.1:{port}/stream") as stream:
        kinds = set()
        for _ in range(10):
            message = json.loads(await asyncio.wait_for(stream.recv(), 3))
            assert message["protocol_version"] == 1
            kinds.add(message["type"])
            if kinds == {"map", "pose", "status"}:
                break
        assert kinds == {"map", "pose", "status"}, kinds
    print("PASS: 10 real HTTP asset requests and map/pose/status over WebSocket")


if __name__ == "__main__":
    asyncio.run(main(int(sys.argv[1]) if len(sys.argv) > 1 else 8765))
