import asyncio
import importlib
import json
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import patch
sys.path.insert(0,str(Path('../../integration/slam_dunk_bridge').resolve()))
from slam_dunk_bridge.core import LatestStore
from frontend_adapter.control_proxy import DashboardServer

class Socket:
    def __init__(self,messages=(),wait=False,origin='http://localhost:8765'):
        self.messages=messages;self.wait=wait;self.sent=[];self.closed=[];self.request_headers={'Origin':origin}
    async def send(self,raw):self.sent.append(raw)
    async def close(self,**kwargs):self.closed.append(kwargs)
    def __aiter__(self):
        async def iterator():
            for message in self.messages:yield message
            if self.wait:await asyncio.Event().wait()
        return iterator()

class ProxyTests(unittest.IsolatedAsyncioTestCase):
    async def test_same_port_delivers_command_and_controller_ack(self):
        request=json.dumps({'type':'navigate','position':[2.8,2.8]})
        ack=json.dumps({'type':'control_status','accepted':True})
        browser=Socket([request],wait=True);upstream=Socket([ack])
        class Connection:
            async def __aenter__(self):return upstream
            async def __aexit__(self,*_):pass
        client=types.ModuleType('websockets.legacy.client');client.connect=lambda *_,**__:Connection()
        legacy=types.ModuleType('websockets.legacy');legacy.__path__=[]
        package=types.ModuleType('websockets');package.__path__=[]
        with patch.dict(sys.modules,{'websockets':package,'websockets.legacy':legacy,'websockets.legacy.client':client}):
            server=DashboardServer(LatestStore());await server._handler(browser,'/control')
        self.assertEqual(upstream.sent,[request]);self.assertEqual(browser.sent,[ack]);self.assertFalse(server.clients)
    async def test_foreign_origin_is_rejected(self):
        browser=Socket(origin='http://unknown.example');server=DashboardServer(LatestStore())
        await server._handler(browser,'/control');self.assertEqual(browser.closed[0]['code'],1008)

if __name__=='__main__':unittest.main()
