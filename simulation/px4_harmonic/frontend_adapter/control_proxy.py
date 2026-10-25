"""Same-origin bounded relay; the PX4 controller retains all command authority."""
import asyncio
import json
from slam_dunk_bridge.transport import BridgeServer

class DashboardServer(BridgeServer):
    async def _http(self,path,headers):
        if path=='/control' and headers.get('Upgrade','').lower()=='websocket':return None
        return await super()._http(path,headers)

    async def _handler(self,socket,path):
        if path!='/control':return await super()._handler(socket,path)
        origin=socket.request_headers.get('Origin')
        if origin not in ('http://localhost:8765','http://127.0.0.1:8765'):
            await socket.close(code=1008,reason='Local dashboard origin required');return
        if len(self.clients)>=self.max_clients:
            await socket.close(code=1013,reason='Client limit reached');return
        self.clients.add(socket)
        try:
            from websockets.legacy.client import connect
            async with connect('ws://127.0.0.1:8766/control',origin='http://127.0.0.1:8765',
                               open_timeout=2,max_size=16384,max_queue=1,close_timeout=1) as upstream:
                print('CONTROL LINK: browser connected to PX4 gateway through dashboard',flush=True)
                async def forward(source,target,commands=False):
                    async for raw in source:
                        if not isinstance(raw,str) or len(raw)>(2048 if commands else 16384):
                            raise ValueError('Control message exceeds bounds')
                        if commands:
                            try:
                                message=json.loads(raw)
                                if message.get('type')!='steer_renew':print('CONTROL REQUEST: '+str(message.get('type'))+' '+str(message.get('position','')),flush=True)
                            except (ValueError,AttributeError):pass
                        await asyncio.wait_for(target.send(raw),1)
                tasks=[asyncio.create_task(forward(socket,upstream,True)),asyncio.create_task(forward(upstream,socket))]
                try:
                    await asyncio.wait(tasks,return_when=asyncio.FIRST_COMPLETED)
                finally:
                    for task in tasks:task.cancel()
                    await asyncio.gather(*tasks,return_exceptions=True)
        except Exception as error:
            print('CONTROL LINK unavailable: '+str(error),flush=True)
        finally:
            self.clients.discard(socket)
            await socket.close(code=1013,reason='PX4 control gateway disconnected; retry')
