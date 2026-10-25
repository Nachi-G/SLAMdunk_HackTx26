import ast
import unittest
from pathlib import Path
import numpy as np
from quick_px4_demo.planner import trees,route,choose_velocity,clearance,MAX_SPEED
from quick_px4_demo.readiness import ground_readiness,simulation_frame_candidate,feedback_timestamp


class DemoTests(unittest.TestCase):
    def test_command_clock_uses_fresh_feedback_and_rejects_stale_data(self):
        # The timestamp remains in PX4's DDS clock domain even with wall-clock drift.
        value=1792907638000000
        self.assertEqual(feedback_timestamp(value,10.,10.04),value)
        with self.assertRaisesRegex(ValueError,'too old'):feedback_timestamp(value,10.,10.31)
        with self.assertRaises(ValueError):feedback_timestamp(value,10.,9.)
        with self.assertRaises(ValueError):feedback_timestamp(0,10.,10.01)

    def test_attitude_bias_does_not_rotate_geographic_position_axes(self):
        from perception_geometry import pose_matrix
        world=pose_matrix([0,0,.227],[1,0,0,0])
        px4=pose_matrix([0,0,0],[np.cos(.075),0,0,np.sin(.075)])
        calibration=simulation_frame_candidate(world,px4)
        np.testing.assert_allclose(calibration[:3,:3],np.eye(3))
        moved=pose_matrix([2,0,2.227],[1,0,0,0])
        np.testing.assert_allclose((calibration@moved)[:3,3],[2,0,2],atol=1e-10)

    def test_waits_for_landed_message_and_rejects_confirmed_airborne(self):
        self.assertEqual(ground_readiness(False,None,.02,None),'WAIT')
        self.assertEqual(ground_readiness(False,True,.02,3.),'WAIT')
        self.assertEqual(ground_readiness(False,True,.02,.5),'READY')
        self.assertEqual(ground_readiness(False,False,.02,.5),'REJECT')
        self.assertEqual(ground_readiness(True,True,.02,.5),'REJECT')

    def test_static_and_dynamic_obstacles_reject_direct_path(self):
        static=trees()
        self.assertLess(float(clearance(np.array([4.,0.]),0.,static)),0)
        self.assertLess(float(clearance(np.array([6.33318,-2.81082]),0.,static)),0)
        self.assertGreater(float(clearance(np.zeros(2),0.,static)),0)

    def test_reaches_goal_with_moving_obstacles_and_tracking_lag(self):
        static=trees()
        for offset in (0.,12.):
            with self.subTest(obstacle_phase=offset):
                position=np.zeros(2);velocity=np.zeros(2);command=np.zeros(2);goal=np.array([12.,0.]);minimum=100.
                for i in range(1000):
                    t=offset+i*.1
                    if i%15==0:
                        try:path=route(position,goal,t,static)
                        except ValueError:pass
                    command,valid,_=choose_velocity(position,velocity,goal,path,t,static,command,.1)
                    self.assertLessEqual(float(np.linalg.norm(command)),MAX_SPEED+1e-6)
                    # First-order inner-loop lag, absent from the planner's kinematic model.
                    velocity+=(command-velocity)*.4
                    position+=velocity*.1
                    minimum=min(minimum,float(clearance(position,t+.1,static,margin=0)))
                    if np.linalg.norm(position-goal)<.45:break
                self.assertLess(float(np.linalg.norm(position-goal)),.5)
                self.assertGreater(minimum,.2)

    def test_preview_does_not_create_control_publishers(self):
        tree=ast.parse(Path('quick_px4_demo/controller.py').read_text())
        classes=[n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='Demo']
        init=next(n for n in classes[0].body if isinstance(n,ast.FunctionDef) and n.name=='__init__')
        publishers=[]
        for statement in init.body:
            calls=[n for n in ast.walk(statement) if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute) and n.func.attr=='create_publisher']
            if calls:
                self.assertIsInstance(statement,ast.If)
                self.assertIsInstance(statement.test,ast.Name)
                self.assertEqual(statement.test.id,'fly')
                publishers.extend(calls)
        self.assertEqual(len(publishers),3)

if __name__=='__main__':unittest.main()
