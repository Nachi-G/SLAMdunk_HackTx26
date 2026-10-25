import ast
import json
from pathlib import Path
import tempfile
import threading
import time
from types import SimpleNamespace as S
import unittest
import numpy as np
from test_dashboard_goals import controller_method
from quick_px4_demo.manual import ManualSteering
from quick_px4_demo.planner import movers,DRONE_RADIUS,MARGIN

class MissionTests(unittest.TestCase):
    def test_start_standby_never_advances_before_button(self):
        source=ast.parse(Path('quick_px4_demo/controller.py').read_text())
        cls=next(n for n in source.body if isinstance(n,ast.ClassDef) and n.name=='Demo')
        tick=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='control_tick')
        block=next(n for n in ast.walk(tick) if isinstance(n,ast.If) and ast.unparse(n.test)=="self.phase == 'PREFLIGHT'")
        wrapper=ast.parse('def preflight(self):\n    pass').body[0];wrapper.body=[block];namespace={'now':100.}
        exec(compile(ast.fix_missing_locations(ast.Module(body=[wrapper],type_ignores=[])),'controller.py','exec'),namespace)
        transitions=[];state=S(phase='PREFLIGHT',wait_for_start=True,start_requested=False,transition=transitions.append,
                               get_publishers_info_by_topic=lambda _:[],get_name=lambda:'demo')
        namespace['preflight'](state);self.assertEqual(transitions,[])
        state.start_requested=True;namespace['preflight'](state);self.assertEqual(transitions,['WARMUP']);self.assertFalse(state.start_requested)

    def test_start_only_requests_controller_transition_and_home_is_exact(self):
        execute,_=controller_method('mission_command');calls=[]
        state=S(mission_status=lambda:{'start_ready':True,'return_ready':True},start_requested=False,home_world=np.array([1.,-2.,.2]),
                navigate=lambda xy,allow_snap,for_landing=False:(calls.append((xy,allow_snap)) or {'position':xy,'reason':'ok'}))
        result=execute(state,'start');self.assertTrue(state.start_requested);self.assertIn('Start accepted',result['reason']);self.assertEqual(calls,[])
        result=execute(state,'return_home');self.assertEqual(calls,[([1.,-2.],False)]);self.assertIn('landing',result['reason'])
        state.mission_status=lambda:{'start_ready':False,'return_ready':False}
        with self.assertRaises(ValueError):execute(state,'start')
        with self.assertRaises(ValueError):execute(state,'return_home')

    def test_gateway_mission_ack_and_stale_epoch_rejection(self):
        actions=[];gateway=ManualSteering(clock=lambda:10.,mission_command=lambda action:(actions.append(action) or {'reason':'accepted'}),mission_status=lambda:{'start_ready':True,'return_ready':True})
        request=dict(protocol_version=1,type='start',request_id='s',epoch=0)
        self.assertTrue(gateway.handle('browser',request)['accepted']);self.assertEqual(actions,['start'])
        self.assertFalse(gateway.handle('browser',request)['accepted']);self.assertEqual(actions,['start'])
        gateway.observe((0,0,2),0,True)
        gateway.handle('browser',dict(protocol_version=1,type='steer_start',request_id='hold',epoch=1,direction='left'))
        result=gateway.handle('browser',dict(protocol_version=1,type='return_home',request_id='home',epoch=1))
        self.assertTrue(result['accepted']);self.assertIsNone(gateway.velocity());self.assertEqual(actions,['start','return_home'])

    def test_actual_plan_export_velocity_and_stale_omission(self):
        execute,namespace=controller_method('publish_dashboard_plan')
        namespace.update(json=json,movers=movers,DRONE_RADIUS=DRONE_RADIUS,MARGIN=MARGIN)
        now=time.monotonic()
        state=S(plan_output_at=-100.,phase='NAVIGATE',failure=None,manual=None,world_body=np.eye(4),lock=threading.Lock(),path=np.array([[0.,0.],[1.,1.]]),
                prediction=np.empty((0,2)),prediction_at=now,goal=np.array([1.,1.]),flight_started=now-3,flight_elapsed=0.,battery=None,battery_at=None,
                calibration=np.eye(4),odom=S(velocity=[1.,2.,3.]),odom_at=now,recovery_goal=None,landing_target=None,plan_valid=True,static=np.array([[3.,3.,.3]]),profile='baseline')
        with tempfile.TemporaryDirectory() as directory:
            namespace['OUT']=Path(directory);execute(state)
            data=json.loads((Path(directory)/'live_plan.json').read_text())
            self.assertEqual(data['velocity_world'],[2.,1.,-3.]);self.assertGreaterEqual(data['flight_seconds'],3)
            state.plan_output_at=-100;state.odom_at=now-2;execute(state)
            self.assertIsNone(json.loads((Path(directory)/'live_plan.json').read_text())['velocity_world'])

if __name__=='__main__':unittest.main()
