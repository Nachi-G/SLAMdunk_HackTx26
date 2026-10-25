import unittest
import numpy as np
from quick_px4_demo.planner import trees,route,choose_velocity
from quick_px4_demo.landing_recovery import (recovery_target,recovery_velocity,swept_distances,landing_site,landing_segment_clear,RECOVERY_SPEED,LANDING_CLEARANCE,BODY_FLOOR)

class RecoveryTests(unittest.TestCase):
    def test_recorded_stuck_position_recovers_then_reaches_original_goal(self):
        static=trees()
        for phase in (0.,15.,60.):
            with self.subTest(phase=phase):
                position=np.array([9.37,.49]);velocity=np.zeros(2);command=np.zeros(2);goal=np.array([0.,0.])
                initial=swept_distances(position,static,'challenge').min()
                escape,path=recovery_target(position,goal,phase,static,'challenge')
                minimum=initial
                for i in range(240):
                    command,valid,prediction=recovery_velocity(position,velocity,escape,command,.1,static,'challenge')
                    self.assertTrue(valid);self.assertLessEqual(np.linalg.norm(command),RECOVERY_SPEED+1e-8)
                    self.assertGreaterEqual(swept_distances(prediction,static,'challenge').min(),BODY_FLOOR)
                    velocity+=(command-velocity)*.25;position+=velocity*.1
                    minimum=min(minimum,swept_distances(position,static,'challenge').min())
                    if swept_distances(position,static,'challenge').min()>=.70:break
                self.assertGreaterEqual(swept_distances(position,static,'challenge').min(),.70)
                self.assertGreaterEqual(minimum,initial-.002)
                path=route(position,goal,phase+i*.1,static,'challenge')
                for j in range(700):
                    command,valid,_=choose_velocity(position,velocity,goal,path,phase+(i+j)*.1,static,command,.1,'challenge')
                    self.assertTrue(valid);velocity+=(command-velocity)*.25;position+=velocity*.1
                    if np.linalg.norm(position-goal)<.4:break
                self.assertLess(np.linalg.norm(position-goal),.4)

    def test_real_body_collision_or_excessive_speed_is_not_recovery_authority(self):
        static=trees();inside=static[0,:2].copy()
        with self.assertRaises(ValueError):recovery_target(inside,[0,0],0,static,'challenge')
        command,valid,prediction=recovery_velocity([9.37,.49],[1.,0.],[9.57,.09],[0,0],.1,static,'challenge')
        self.assertFalse(valid);self.assertEqual(len(prediction),0)

    def test_landing_near_recorded_problem_gets_restartable_clear_site(self):
        static=trees();position=np.array([9.37,.49]);target,path=landing_site(position,position,0,static,'challenge')
        self.assertLess(np.linalg.norm(target-position),2.5)
        self.assertGreaterEqual(swept_distances(target,static,'challenge').min(),LANDING_CLEARANCE)
        self.assertTrue(landing_segment_clear(target+[.1,0],target,static,'challenge'))
        # A landing tracking error still leaves ordinary planning clearance for restart.
        for dx,dy in ((0,0),(.12,0),(-.12,0),(0,.12),(0,-.12)):
            landed=target+[dx,dy];next_path=route(landed,[0,0],0,static,'challenge')
            _,valid,_=choose_velocity(landed,np.zeros(2),np.zeros(2),next_path,0,static,np.zeros(2),.1,'challenge')
            self.assertTrue(valid)

    def test_full_swept_movers_excluded_from_landing(self):
        static=trees();target,_=landing_site([0,0],[2.8,0],0,static,'challenge')
        self.assertGreaterEqual(swept_distances(target,static,'challenge').min(),LANDING_CLEARANCE)
        self.assertFalse(landing_segment_clear([2.8,0],target,static,'challenge'))


class ControllerRecoveryTests(unittest.TestCase):
    def test_actual_worker_accepts_checked_recovery_in_standby(self):
        import threading
        from types import SimpleNamespace as S
        from test_dashboard_goals import controller_method
        execute,namespace=controller_method('work');stop=threading.Event()
        class RPC:
            def __init__(self,*_):pass
            def create(self,*_):pass
            def move(self,*_):pass
        namespace.update(obstacles=S(ObstacleRPC=RPC),VehicleStatus=S(ARMING_STATE_DISARMED=2),
                         time=S(monotonic=lambda:10.,sleep=lambda _:stop.set()),
                         choose_velocity=choose_velocity,swept_distances=swept_distances,recovery_target=recovery_target,recovery_velocity=recovery_velocity)
        body=np.eye(4);body[:2,3]=[9.37,.49]
        state=S(stop_thread=stop,world_body=body,status=S(arming_state=2),status_at=10.,landed=True,land_at=10.,profile='challenge',names=set(),sim_time=20.,
                phase='PREFLIGHT',manual=None,calibration=np.eye(4),odom=S(velocity=[0.,0.,0.]),lock=threading.Lock(),goal=np.zeros(2),waypoint_index=0,plan_revision=0,
                last_plan_sim=None,recovery_goal=None,recovery_since=None,path=None,path_goal_index=-1,path_time=-100.,static=trees(),command=np.zeros(2),
                manual_rejoin=False,plan_blocks=0,worker_error=None)
        execute(state)
        self.assertIsNone(state.worker_error);self.assertTrue(state.plan_valid);self.assertIsNotNone(state.recovery_goal)
        np.testing.assert_array_equal(state.command,[0,0]) # Standby still issues no movement.

    def test_actual_landing_hold_requires_alignment_and_settling(self):
        import ast
        from pathlib import Path
        from types import SimpleNamespace as S
        source=ast.parse(Path('quick_px4_demo/controller.py').read_text());cls=next(n for n in source.body if isinstance(n,ast.ClassDef) and n.name=='Demo')
        tick=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='control_tick')
        branch=next(n for n in ast.walk(tick) if isinstance(n,ast.If) and ast.unparse(n.test)=="self.phase == 'GOAL_HOLD'")
        wrapper=ast.parse('def hold(self):\n    pass').body[0];wrapper.body=[branch]
        namespace=dict(np=np,landing_segment_clear=landing_segment_clear,VehicleCommand=S(VEHICLE_CMD_NAV_LAND=21),now=100.)
        exec(compile(ast.fix_missing_locations(ast.Module(body=[wrapper],type_ignores=[])),'controller.py','exec'),namespace)
        commands=[];streams=[];body=np.eye(4);body[:3,3]=[.3,0,2]
        state=S(phase='GOAL_HOLD',phase_at=100.,landing_target=np.zeros(2),world_body=body,static=trees(),profile='challenge',calibration=np.eye(4),
                takeoff_target=np.array([0,0,-2.]),odom=S(velocity=[0,0,0]),hold_since=None,stream=lambda **kwargs:streams.append(kwargs),
                vehicle_command=commands.append,transition=lambda phase:None,fail=lambda reason:self.fail(reason))
        namespace['hold'](state);self.assertEqual(commands,[]);self.assertIsNone(state.hold_since)
        body[0,3]=.05;namespace['hold'](state);self.assertEqual(commands,[])
        namespace['now']=102.;namespace['hold'](state);self.assertEqual(commands,[21])
        np.testing.assert_allclose(streams[-1]['position'],[0,0,-2])

if __name__=='__main__':unittest.main()
