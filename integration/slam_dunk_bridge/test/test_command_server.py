import asyncio
import json
import unittest

from slam_dunk_bridge.command_server import CommandServer
from slam_dunk_bridge.control import SteeringSupervisor


class CommandServerTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.now, self.goals = 10., []
        self.supervisor = SteeringSupervisor(lambda p,f:self.goals.append((p,f)),
            clock=lambda:self.now, waypoints=((8,0,2),(8,8,2)), enabled=True)
        self.supervisor.update_pose((0,0,2),(0,0,0,1),100,"map")
        self.supervisor.update_map(100,"map")
        self.supervisor.tick()
        self.server=CommandServer(self.supervisor,port=0)
        self.server.start()
        self.url=f"ws://127.0.0.1:{self.server.port}/control"

    async def asyncTearDown(self):
        await asyncio.to_thread(self.server.stop)

    async def response(self, socket, request):
        for _ in range(8):
            message=json.loads(await asyncio.wait_for(socket.recv(),1))
            if message["last_request_id"] == request and message["accepted"] is not None:
                return message
        self.fail("No correlated command response")

    async def test_real_socket_control_handoff_300(self):
        from websockets.legacy.client import connect
        for i in range(300):
            with self.subTest(i=i):
                async with connect(self.url,origin="http://127.0.0.1:8080") as socket:
                    initial=json.loads(await socket.recv())
                    self.assertTrue(initial["ready"])
                    request=f"socket-{i}"
                    await socket.send(json.dumps({"protocol_version":1,"type":"steer_start","epoch":initial["epoch"],"request_id":request,"direction":tuple(self.supervisor.DIRECTIONS)[i%4]}))
                    started=await self.response(socket,request)
                    self.assertTrue(started["accepted"])
                    self.assertEqual(started["mode"],"steering")
                    await socket.send(json.dumps({"protocol_version":1,"type":"steer_renew","epoch":started["epoch"],"request_id":request+"r","lease_id":started["lease_id"],"seq":1}))
                    self.assertTrue((await self.response(socket,request+"r"))["accepted"])
                    if i%3 == 0:
                        await socket.send(json.dumps({"protocol_version":1,"type":"steer_release","epoch":started["epoch"],"request_id":request+"e","lease_id":started["lease_id"],"seq":2}))
                        self.assertTrue((await self.response(socket,request+"e"))["accepted"])
                    elif i%3 == 1:
                        self.now += .501
                        # Keep telemetry fresh while expiring only the command lease.
                        self.supervisor.update_pose((0,0,2),(0,0,0,1),101+i,"map")
                        self.supervisor.update_map(101+i,"map")
                        self.supervisor.tick()
                    # Third branch deliberately drops a socket while actively steering.
                for _ in range(30):
                    if self.supervisor.status()["lease_id"] is None:
                        break
                    await asyncio.sleep(.005)
                self.assertIsNone(self.supervisor.status()["lease_id"])
                self.assertEqual(self.supervisor.status()["mode"],"autonomous")
                self.assertEqual(self.goals[-1][0],(8,0,2))

    async def test_origin_path_and_nonjson_rejection(self):
        from websockets.legacy.client import connect
        from websockets.exceptions import InvalidStatusCode
        for origin in (None,"http://evil.example","https://127.0.0.1:8080","http://127.0.0.1:8080.evil"):
            with self.subTest(origin=origin):
                with self.assertRaises(InvalidStatusCode):
                    async with connect(self.url,origin=origin): pass
        with self.assertRaises(InvalidStatusCode):
            async with connect(self.url.replace('/control','/stream'),origin="http://127.0.0.1:8080"): pass
        async with connect(self.url,origin="http://localhost:8080") as socket:
            await socket.recv()
            for payload in ('{bad', '[]', '{"protocol_version":NaN}', b'not text'):
                await socket.send(payload)
                result=await self.response(socket,None)
                self.assertFalse(result["accepted"])

    def test_loopback_configuration(self):
        for host in ("0.0.0.0","192.168.1.2","example.com",""):
            with self.assertRaises(ValueError): CommandServer(self.supervisor,host=host)
        for origins in ([],["*"],[None],["file://"]):
            with self.assertRaises(ValueError): CommandServer(self.supervisor,origins=origins)


if __name__ == "__main__": unittest.main()
