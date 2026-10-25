"""Isolated native RPC for only named simulator obstacles; no drone commands."""
from pathlib import Path
import os
import selectors
import subprocess
import time

NAMES=tuple(f'quick_demo_crossing_{i+1}' for i in range(12))
ROOT=Path(__file__).resolve().parents[1]


class ObstacleRPC:
    def __init__(self,profile='baseline'):
        self.profile=profile
        log=ROOT/'diagnostics/quick_px4_demo/obstacle_rpc.log'
        self.log=log.open('w')
        # Use system Gazebo transport libraries, independent of ROS vendor paths.
        env=dict(os.environ)
        env['LD_LIBRARY_PATH']=':'.join(p for p in env.get('LD_LIBRARY_PATH','').split(':') if p and not p.startswith('/opt/ros/'))
        command=[str(ROOT/'quick_px4_demo/obstacle_rpc')]+(['--'+profile] if profile!='baseline' else [])
        self.process=subprocess.Popen(command,stdin=subprocess.PIPE,
                                      stdout=subprocess.PIPE,stderr=self.log,text=True,bufsize=1,env=env)
        self.selector=selectors.DefaultSelector();self.selector.register(self.process.stdout,selectors.EVENT_READ)

    def request(self,line,timeout):
        if self.process.poll() is not None:raise RuntimeError('Obstacle RPC helper exited')
        self.process.stdin.write(line+'\n');self.process.stdin.flush()
        if not self.selector.select(timeout):raise RuntimeError('Obstacle RPC helper response timeout')
        response=self.process.stdout.readline().strip()
        if response!='OK':raise RuntimeError('Obstacle RPC: '+response)

    def create(self,names):
        count={'baseline':2,'slalom':3,'challenge':12}[self.profile]
        existing=names()
        for i in range(count,len(NAMES)):
            if NAMES[i] in existing:self.request(f'R {i}',2)
        for i,name in enumerate(NAMES[:count]):
            if name in names():continue
            try:self.request(f'C {i}',5)
            except RuntimeError:
                # Async create can complete even when the acknowledgement is lost.
                until=time.monotonic()+1
                while time.monotonic()<until and name not in names():time.sleep(.02)
                if name not in names():raise

    def move(self,t):self.request(f'M {float(t):.9f}',.7)

    def close(self):
        if self.process.poll() is None:
            self.process.stdin.close()
            try:self.process.wait(timeout=1)
            except subprocess.TimeoutExpired:self.process.terminate();self.process.wait(timeout=1)
        self.selector.close();self.log.close()
