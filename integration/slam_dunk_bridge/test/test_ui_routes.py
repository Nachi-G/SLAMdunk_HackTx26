"""Hundreds of routing cases without ROS or a running socket server."""

import unittest
from http import HTTPStatus

from slam_dunk_bridge.core import LatestStore
from slam_dunk_bridge.transport import BridgeServer, WEB_ASSETS


class DashboardRouteTests(unittest.IsolatedAsyncioTestCase):
    async def test_every_asset_and_query(self):
        server = BridgeServer(LatestStore())
        for route, (_, mime) in WEB_ASSETS.items():
            for i in range(40):
                with self.subTest(route=route, query=i):
                    status, headers, body = await server._http(f"{route}?source=live&case={i}", {})
                    self.assertEqual(status, HTTPStatus.OK)
                    self.assertEqual(dict(headers)["Content-Type"], mime)
                    self.assertEqual(dict(headers)["X-Content-Type-Options"], "nosniff")
                    self.assertGreater(len(body), 0)

    async def test_room_stylesheet_contains_layout_not_javascript(self):
        server = BridgeServer(LatestStore())
        status, headers, body = await server._http("/room.css?v=room-fix-2", {})
        self.assertEqual(status, HTTPStatus.OK)
        self.assertEqual(dict(headers)["Content-Type"], "text/css; charset=utf-8")
        self.assertRegex(body, rb":root\s*\{")
        self.assertRegex(body, rb"\.room-canvas-wrap\s*\{")
        self.assertRegex(body, rb"#selected-object\s*\{")
        self.assertNotRegex(body, rb"(?m)^\s*(?:import |export )")

    async def test_private_paths_and_unknown_assets_are_never_served(self):
        server = BridgeServer(LatestStore())
        fragments = ("../core.py", "../transport.py", "../node.py", "../viewer.html",
                     "model.test.mjs", "%2e%2e/core.py", "..%5ccore.py", ".git/config",
                     "web/../../setup.py", "app.mjs/../core.py")
        for fragment in fragments:
            for i in range(40):
                with self.subTest(fragment=fragment, query=i):
                    # Different nesting depths and suffixes exercise exact route
                    # matching, not merely ignored query-string variations.
                    prefix = "../" * (i % 8)
                    suffix = ("", "/", "%00", ".mjs", "/index.html")[i // 8]
                    status, _, _ = await server._http(f"/{prefix}{fragment}{suffix}", {})
                    self.assertEqual(status, HTTPStatus.NOT_FOUND)

    async def test_websocket_upgrade_and_legacy(self):
        server = BridgeServer(LatestStore())
        for i in range(200):
            self.assertIsNone(await server._http("/stream", {"Upgrade": "WebSocket"}))
            status, _, body = await server._http(f"/legacy?case={i}", {})
            self.assertEqual(status, HTTPStatus.OK)
            self.assertIn(b"<canvas", body)
            status, _, _ = await server._http("/stream", {})
            self.assertEqual(status, HTTPStatus.NOT_FOUND)


if __name__ == "__main__":
    unittest.main()
