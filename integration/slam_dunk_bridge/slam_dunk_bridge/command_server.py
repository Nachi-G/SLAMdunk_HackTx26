"""Optional loopback control sidecar; never added to the telemetry stream."""

import asyncio
from http import HTTPStatus
import json
import secrets
import threading
import time


DEFAULT_ORIGINS = ("http://127.0.0.1:8080", "http://localhost:8080",
                   "http://127.0.0.1:8765", "http://localhost:8765")


class CommandServer:
    def __init__(self, supervisor, host="127.0.0.1", port=8766, *, origins=DEFAULT_ORIGINS, max_clients=4):
        if supervisor.ttl != .5:
            raise ValueError("Browser control protocol requires a fixed 500 ms lease")
        if host not in ("127.0.0.1", "localhost", "::1"):
            raise ValueError("Control sidecar is loopback-only; use an SSH tunnel for remote operation")
        if (not isinstance(port, int) or isinstance(port, bool) or not 0 <= port <= 65535 or
                not isinstance(max_clients, int) or not 1 <= max_clients <= 16 or
                not isinstance(origins, (list, tuple)) or not origins or len(origins) > 32 or
                not all(isinstance(o, str) and o.startswith(("http://", "https://")) and "*" not in o for o in origins)):
            raise ValueError("Invalid control endpoint limits/origins")
        self.supervisor, self.host, self.port = supervisor, host, port
        self.origins, self.max_clients = list(origins), max_clients
        self.clients = set()
        self.loop, self.stop_event, self.thread = None, None, None
        self.error, self.ready = None, threading.Event()

    async def _http(self, path, headers):
        if path == "/control" and headers.get("Upgrade", "").lower() == "websocket":
            return None
        return HTTPStatus.NOT_FOUND, [("Content-Type", "text/plain")], b"Control WebSocket: /control\n"

    async def _handler(self, socket, path):
        if path != "/control" or len(self.clients) >= self.max_clients:
            await socket.close(code=1013, reason="Control endpoint unavailable or client limit reached")
            return
        owner = secrets.token_urlsafe(15)
        self.clients.add(socket)
        send_lock = asyncio.Lock()

        async def send(make_status):
            async with send_lock:
                # Snapshot/mutate only after earlier sends finish. A queued old
                # autonomous snapshot must not arrive after a steering ack.
                status = make_status()
                await asyncio.wait_for(socket.send(json.dumps(status, allow_nan=False)), 1)

        async def updates():
            try:
                while not socket.closed:
                    await send(lambda: self.supervisor.status(owner_id=owner))
                    await asyncio.sleep(.1)
            except Exception:
                await socket.close(code=1013, reason="Control status client stalled")

        task = asyncio.create_task(updates())
        window, count = time.monotonic(), 0
        try:
            async for raw in socket:
                if time.monotonic()-window >= 1:
                    window, count = time.monotonic(), 0
                count += 1
                if count > 40:
                    await socket.close(code=1008, reason="Control message rate exceeded")
                    break
                try:
                    if not isinstance(raw, str):
                        raise ValueError("JSON text required")
                    message = json.loads(raw, parse_constant=lambda _: (_ for _ in ()).throw(ValueError("Nonfinite JSON")))
                except (ValueError, TypeError):
                    message = None
                await send(lambda: self.supervisor.handle(owner, message))
        except Exception:
            await socket.close(code=1013, reason="Control client disconnected or stalled")
        finally:
            task.cancel()
            await asyncio.gather(task, return_exceptions=True)
            self.supervisor.disconnect(owner)
            self.clients.discard(socket)

    async def run(self):
        from websockets.legacy.server import serve
        self.loop = asyncio.get_running_loop()
        self.stop_event = asyncio.Event()
        async with serve(self._handler, self.host, self.port, origins=self.origins,
                         process_request=self._http, max_size=2048, max_queue=4,
                         write_limit=16384, ping_interval=5, ping_timeout=5, close_timeout=1) as server:
            self.port = server.sockets[0].getsockname()[1]
            self.ready.set()
            await self.stop_event.wait()

    def start(self):
        def worker():
            try:
                asyncio.run(self.run())
            except Exception as error:
                self.error = error
                self.ready.set()
        self.thread = threading.Thread(target=worker, name="steering-command-server", daemon=True)
        self.thread.start()
        if not self.ready.wait(5):
            raise RuntimeError("Control server startup timed out")
        if self.error:
            raise RuntimeError("Control server startup failed") from self.error

    def stop(self):
        if self.loop and self.stop_event and self.loop.is_running():
            self.loop.call_soon_threadsafe(self.stop_event.set)
        if self.thread:
            self.thread.join(timeout=5)
