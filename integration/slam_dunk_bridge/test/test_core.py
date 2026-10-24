import asyncio
import json
import math
import struct
import unittest
from types import SimpleNamespace as NS

from slam_dunk_bridge.core import (LatestStore, MapPublisher, decode_xyz,
                                   quaternion_product, transform_point)
from slam_dunk_bridge.transport import send_latest


def cloud(points, endian=False, padding=0, field_type=7):
    fmt = "d" if field_type == 8 else "f"
    size = struct.calcsize(fmt)
    fields = [NS(name=name, datatype=field_type, count=1, offset=4+i*size)
              for i,name in enumerate(("x","y","z"))]
    step=4+3*size
    # One point per row exercises organized row padding and nonzero field offsets.
    data=b"".join(b"HEAD"+struct.pack((">" if endian else "<")+fmt*3,*p)+b"P"*padding for p in points)
    return NS(width=1, height=len(points), point_step=step, row_step=step+padding,
              data=data, fields=fields, is_bigendian=endian)


class CoreTests(unittest.TestCase):
    def test_layout_variants(self):
        for endian in (False,True):
            for datatype in (7,8):
                decoded,_=decode_xyz(cloud([(1,2,3),(-4,5,6)],endian,7,datatype))
                self.assertEqual(decoded,[(1,2,3),(-4,5,6)])

    def test_invalid_and_bounded_sampling(self):
        points=[(i,0,0) for i in range(100)]
        points[0]=(math.nan,0,0)
        decoded,stats=decode_xyz(cloud(points),max_points=9)
        self.assertLessEqual(len(decoded),9)
        self.assertEqual(stats["invalid_points"],1)
        with self.assertRaises(ValueError): decode_xyz(cloud(points),max_input_points=99)

    def test_corrupt_layout_rejected(self):
        c=cloud([(1,2,3)])
        c.data=c.data[:-1]
        with self.assertRaises(ValueError): decode_xyz(c)
        c=cloud([(1,2,3)]);c.fields[0].offset=c.point_step
        with self.assertRaises(ValueError): decode_xyz(c)
        c=cloud([(1,2,3)]);c.fields[0].datatype=6
        with self.assertRaises(ValueError): decode_xyz(c)

    def test_transform_composition(self):
        q=(0,0,math.sin(math.pi/4),math.cos(math.pi/4))
        point=transform_point((1,0,0),(3,4,5),q)
        for actual,expected in zip(point,(3,5,5)): self.assertAlmostEqual(actual,expected)
        out=quaternion_product(q,q)
        self.assertAlmostEqual(abs(out[2]),1)
        with self.assertRaises(ValueError): transform_point((1,2,3),(0,0,0),(0,0,0,0))

    def test_map_replacement_and_clear(self):
        store=LatestStore();publisher=MapPublisher(store,max_points=2)
        publisher.replace([(1,2,3)],{"sec":1,"nanosec":0},"test")
        publisher.replace([], {"sec":2,"nanosec":0},"test")
        self.assertEqual(len(store.snapshot()),1)
        output=json.loads(store.snapshot()["map"][1])
        self.assertEqual(output["points"],[])
        self.assertEqual(output["revision"],2)
        self.assertEqual(output["operation"],"replace")
        with self.assertRaises(ValueError): publisher.replace([(0,0,0)]*3,{},"test")
        with self.assertRaises(ValueError): publisher.replace([(math.inf,0,0)],{},"test")


class TransportTests(unittest.IsolatedAsyncioTestCase):
    async def test_only_latest_per_kind_and_no_repeats(self):
        store=LatestStore()
        for i in range(100): store.publish({"type":"map","revision":i})
        store.publish({"type":"pose","position":[0,0,0]})
        class Socket:
            def __init__(self): self.sent=[]
            async def send(self,payload): self.sent.append(json.loads(payload))
        socket=Socket();seen={}
        await send_latest(socket,store,seen)
        await send_latest(socket,store,seen)
        self.assertEqual(len(socket.sent),2)
        self.assertEqual(socket.sent[0]["revision"],99)

    async def test_slow_sender_times_out(self):
        store=LatestStore();store.publish({"type":"map"})
        class Slow:
            async def send(self,payload): await asyncio.sleep(10)
        with self.assertRaises(asyncio.TimeoutError):
            await send_latest(Slow(),store,{},timeout=.01)


if __name__ == "__main__": unittest.main()
