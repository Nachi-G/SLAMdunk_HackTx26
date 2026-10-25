import ast
from pathlib import Path
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
import numpy as np
from quick_px4_demo.manual import ManualSteering
from quick_px4_demo.landing_recovery import landing_site
from quick_px4_demo.planner import trees,route,resolve_pin,choose_velocity,clearance

def controller_method(name):
    # Execute the actual pure controller method without constructing ROS publishers.
    cls=next(n for n in ast.parse(Path('quick_px4_demo/controller.py').read_text()).body if isinstance(n,ast.ClassDef) and n.name=='Demo')
    method=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name==name)
    namespace={'np':np,'time':time,'route':route,'resolve_pin':resolve_pin,'landing_site':landing_site}
    exec(compile(ast.Module(body=[method],type_ignores=[]),'controller.py','exec'),namespace)
    return namespace[name],namespace

class GoalTests(unittest.TestCase):
    def test_navigate_replaces_remaining_mission_with_validated_pin(self):
        execute,_=controller_method('navigate');events=[]
        state=SimpleNamespace(phase='NAVIGATE',failure=None,world_body=np.eye(4),sim_time=10.,motion_start=0.,static=trees(),profile='challenge',
                              lock=threading.Lock(),plan_revision=3,event=events.append)
        execute(state,[13.,0.]);self.assertEqual(state.mission,[(13.,0.)]);self.assertEqual(state.plan_revision,4)
        self.assertTrue(state.manual_rejoin);self.assertFalse(state.plan_valid)
        np.testing.assert_allclose(state.path[-1],[13,0])
        with self.assertRaisesRegex(ValueError,'Pin rejected'):execute(state,[100.,100.])
        self.assertEqual(state.mission,[(13.,0.)])
        state.phase='LANDING'
        with self.assertRaises(ValueError):execute(state,[0,0])

    def test_pin_gateway_redirects_planner_to_new_goal(self):
        execute,_=controller_method('navigate');body=np.eye(4);body[:2,3]=[6.4,-.4]
        state=SimpleNamespace(phase='NAVIGATE',failure=None,world_body=body,sim_time=20.,motion_start=0.,static=trees(),profile='challenge',
                              lock=threading.Lock(),plan_revision=3,event=lambda _:None,mission=[(13.,0.)])
        gateway=ManualSteering(clock=lambda:10.,navigate=lambda xy:execute(state,xy))
        gateway.observe((6.4,-.4,2),0,True)
        reply=gateway.handle('browser',dict(protocol_version=1,type='navigate',request_id='pin',epoch=0,position=[2.8,2.8]))
        self.assertTrue(reply['accepted']);position=body[:2,3].copy();velocity=np.zeros(2);command=np.zeros(2)
        for i in range(600):
            command,valid,_=choose_velocity(position,velocity,state.goal,state.path,20+i*.1,state.static,command,.1,profile='challenge')
            self.assertTrue(valid);velocity+=(command-velocity)*.4;position+=velocity*.1
            if np.linalg.norm(position-state.goal)<.4:break
        self.assertLess(np.linalg.norm(position-[2.8,2.8]),.4)
        self.assertGreater(np.linalg.norm(position-[13,0]),5)
        self.assertEqual(state.mission,[(2.8,2.8)])

    def test_blocked_pin_snaps_to_reachable_goal_and_reports_actual_target(self):
        execute,_=controller_method('navigate');state=SimpleNamespace(phase='NAVIGATE',failure=None,world_body=np.eye(4),sim_time=20.,motion_start=0.,static=trees(),profile='challenge',lock=threading.Lock(),plan_revision=0,event=lambda _:None)
        gateway=ManualSteering(clock=lambda:10.,navigate=lambda xy:execute(state,xy));gateway.observe((0,0,2),0,True)
        reply=gateway.handle('browser',dict(protocol_version=1,type='navigate',request_id='blocked',epoch=0,position=[2.8,0.]))
        self.assertTrue(reply['accepted']);self.assertIn('adjusted',reply['reason'])
        np.testing.assert_allclose(reply['destination'],state.goal)
        self.assertLessEqual(np.linalg.norm(state.goal-[2.8,0]),2.5)
        self.assertGreater(np.linalg.norm(state.goal-[2.8,0]),.5)
        np.testing.assert_allclose(state.path[-1],state.goal)
        position=np.zeros(2);velocity=np.zeros(2);command=np.zeros(2)
        for i in range(600):
            command,valid,_=choose_velocity(position,velocity,state.goal,state.path,20+i*.1,state.static,command,.1,profile='challenge')
            self.assertTrue(valid);velocity+=(command-velocity)*.4;position+=velocity*.1
            if np.linalg.norm(position-state.goal)<.4:break
        self.assertLess(np.linalg.norm(position-state.goal),.4)
        with self.assertRaisesRegex(ValueError,'outside course'):execute(state,[100,100])

    def test_goal_protocol_rejects_stale_epoch_held_and_invalid_targets(self):
        now=[10.];goals=[];gateway=ManualSteering(clock=lambda:now[0],navigate=goals.append)
        gateway.observe((0,0,2),0,True)
        def command(**extra):return dict(protocol_version=1,type='navigate',request_id='pin',epoch=gateway.epoch,position=[13.,0.],**extra)
        request=command();result=gateway.handle('browser',request);self.assertTrue(result['accepted']);self.assertEqual(goals,[[13.,0.]])
        self.assertFalse(gateway.handle('browser',request)['accepted'])
        for value in ([True,0],[float('nan'),0],[0],[0,0,2]):
            bad=command();bad['position']=value;self.assertFalse(gateway.handle('browser',bad)['accepted'])
        gateway.handle('browser',dict(protocol_version=1,type='steer_start',request_id='hold',epoch=gateway.epoch,direction='left'))
        self.assertFalse(gateway.handle('browser',command())['accepted'])

    def test_destinations_ready_before_manual_steering_and_rejection_is_visible(self):
        goals=[];gateway=ManualSteering(clock=lambda:10.,navigate=goals.append,navigation_ready=lambda:True)
        gateway.observe((0,0,2),0,False)
        self.assertFalse(gateway.status()['ready']);self.assertTrue(gateway.status()['navigation_ready'])
        result=gateway.handle('browser',dict(protocol_version=1,type='navigate',request_id='pin',epoch=0,position=[5.,0.]))
        self.assertTrue(result['accepted']);self.assertEqual(goals,[[5.,0.]])
        def reject(_):raise ValueError('Pin rejected: Goal inside inflated obstacle')
        gateway.navigate=reject
        result=gateway.handle('browser',dict(protocol_version=1,type='navigate',request_id='blocked',epoch=1,position=[6.,0.]))
        self.assertFalse(result['accepted']);self.assertIn('inflated obstacle',result['reason'])
        self.assertEqual(gateway.epoch,1)

    def test_selected_prediction_is_checked_against_actual_moving_phase(self):
        static=trees();position=np.zeros(2);path=route(position,[12,0],0,static)
        result=choose_velocity(position,np.zeros(2),np.array([12,0]),path,0,static,np.zeros(2),.1,return_prediction=True)
        command,valid,margin,prediction=result
        self.assertTrue(valid);self.assertEqual(prediction.shape,(40,2));self.assertGreater(margin,.05)
        self.assertTrue(np.all(clearance(prediction,np.arange(.1,4.01,.1),static)>.05))
        self.assertEqual(len(choose_velocity(position,np.zeros(2),np.array([12,0]),path,0,static,np.zeros(2),.1)),3)

if __name__=='__main__':unittest.main()
