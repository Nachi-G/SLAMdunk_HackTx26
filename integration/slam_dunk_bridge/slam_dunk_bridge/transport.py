"""Bounded-rate WebSocket output and bundled read-only viewer."""

import asyncio
from http import HTTPStatus
from pathlib import Path
import threading
from urllib.parse import urlsplit


# Deliberate allowlist: never expose arbitrary package or workspace files.
WEB_ASSETS = {
    "/": ("index.html", "text/html; charset=utf-8"),
    "/mission-map.html": ("mission-map.html", "text/html; charset=utf-8"),
    "/flight-room.html": ("flight-room.html", "text/html; charset=utf-8"),
    "/dashboard.html": ("dashboard.html", "text/html; charset=utf-8"),
    "/room.css": ("room.css", "text/css; charset=utf-8"),
    "/index.html": ("index.html", "text/html; charset=utf-8"),
    "/styles.css": ("styles.css", "text/css; charset=utf-8"),
    **{f"/{name}.mjs": (f"{name}.mjs", "text/javascript; charset=utf-8")
       for name in ("app", "scene", "model", "adapter", "demo", "camera", "planner", "pov", "room", "mission-log", "mission-map", "room-layout", "unknown")},
}


async def send_latest(websocket, store, seen, timeout=1.0):
    """Send at most one current message per type; close slow peers upstream."""
    for kind, (sequence, payload) in store.snapshot().items():
        if seen.get(kind) != sequence:
            await asyncio.wait_for(websocket.send(payload), timeout)
            seen[kind] = sequence


class BridgeServer:
    def __init__(self, store, host="127.0.0.1", port=8765, rate_hz=10.0, max_clients=8):
        if not 0 < rate_hz <= 10 or not 1 <= max_clients <= 64:
            raise ValueError("rate_hz must be (0,10], max_clients 1..64")
        self.store, self.host, self.port = store, host, port
        self.period, self.max_clients = 1/rate_hz, max_clients
        self.clients = set()
        self.loop = None
        self.stop_event = None
        self.thread = None
        self.error = None
        self.ready = threading.Event()

    async def _handler(self, websocket, path):
        if path != "/stream" or len(self.clients) >= self.max_clients:
            await websocket.close(code=1013, reason="Endpoint unavailable or client limit reached")
            return
        self.clients.add(websocket)
        seen = {}
        try:
            # This is an output-only channel; stop clients from building an inbound queue.
            async def consume():
                async for _ in websocket:
                    await websocket.close(code=1008, reason="Read-only stream")
            consumer = asyncio.create_task(consume())
            try:
                while not websocket.closed:
                    await send_latest(websocket, self.store, seen)
                    await asyncio.sleep(self.period)
            finally:
                consumer.cancel()
                await asyncio.gather(consumer, return_exceptions=True)
        except Exception:
            await websocket.close(code=1013, reason="Slow or disconnected client")
        finally:
            self.clients.discard(websocket)

    async def _http(self, path, headers):
        if path == "/stream" and headers.get("Upgrade", "").lower() == "websocket":
            return None
        try:
            route = urlsplit(path).path
        except ValueError:
            route = ""
        if route in WEB_ASSETS:
            filename, content_type = WEB_ASSETS[route]
            asset = Path(__file__).with_name("web") / filename
            if asset.is_file():
                return HTTPStatus.OK, [("Content-Type", content_type),
                                       ("Cache-Control", "no-store"),
                                       ("X-Content-Type-Options", "nosniff")], asset.read_bytes()
        if route in ("/legacy", "/legacy.html"):
            body = Path(__file__).with_name("viewer.html").read_bytes()
            return HTTPStatus.OK, [("Content-Type", "text/html; charset=utf-8"),
                                   ("Cache-Control", "no-store")], body
        return HTTPStatus.NOT_FOUND, [("Content-Type", "text/plain")], b"Not found\n"

    async def run(self):
        # Pin legacy API through websockets <14 in package documentation.
        from websockets.legacy.server import serve
        self.loop = asyncio.get_running_loop()
        self.stop_event = asyncio.Event()
        async with serve(self._handler, self.host, self.port,
                                    process_request=self._http, max_size=1024,
                                    max_queue=1, write_limit=65536,
                                    ping_interval=10, ping_timeout=10, close_timeout=1):
            self.ready.set()
            await self.stop_event.wait()

    def start(self):
        def worker():
            try:
                asyncio.run(self.run())
            except Exception as exc:
                self.error = exc
                self.ready.set()
        self.thread = threading.Thread(target=worker, name="map-websocket", daemon=True)
        self.thread.start()
        if not self.ready.wait(5):
            raise RuntimeError("WebSocket server startup timed out")
        if self.error:
            raise RuntimeError("WebSocket server startup failed") from self.error

    def stop(self):
        if self.loop and self.stop_event and self.loop.is_running():
            self.loop.call_soon_threadsafe(self.stop_event.set)
        if self.thread:
            self.thread.join(timeout=5)
