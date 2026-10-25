import math
import unittest
from slamdunk_car.mapping import project_grid,columns,transform_point
from slamdunk_car.exploration import select_frontier

class MappingTests(unittest.TestCase):
    def project(self,data):return project_grid(data,2,2,.1,(0.,0.),bounds=(0.,.4,0.,.2),step=.2)
    def test_unknown_outside_slam_extent_stays_unknown(self):
        occupied,unknown,cells,nx,ny=self.project([0,0,0,0]);self.assertEqual(cells,[0,-1]);self.assertEqual(len(unknown),1)
    def test_one_occupied_source_cell_blocks_larger_cell(self):
        self.assertEqual(self.project([0,100,0,0])[2],[100,-1])
    def test_partial_or_uncertain_observation_does_not_become_free(self):
        for data in [[0,-1,0,0],[0,50,0,0]]:self.assertEqual(self.project(data)[2],[-1,-1])
    def test_lidar_hit_can_add_obstacle_below_2d_scan(self):
        self.assertEqual(project_grid([0]*4,2,2,.1,(0.,0.),(0.,.2,0.,.2),.2,hits=[(0,0)])[2],[100])
    def test_columns_block_all_planning_heights(self):
        cloud=columns([(.1,.1)]);self.assertEqual({round(p[2],2) for p in cloud},{0.,.2,.4,.6,.8,1.})
    def test_invalid_or_unbounded_map_rejected(self):
        for args in [([],2,2,.1,(0,0)),([0]*4,2,2,-.1,(0,0)),([101]*4,2,2,.1,(0,0))]:
            with self.assertRaises(ValueError):project_grid(*args)
    def test_sensor_pose_rotation(self):
        p=transform_point((1.,0.,0.),(2.,3.,4.),(0.,0.,math.sin(math.pi/4),math.cos(math.pi/4)))
        for a,b in zip(p,(2.,4.,4.)):self.assertAlmostEqual(a,b)
    def test_frontier_viewpoint_is_reachable_and_clear(self):
        width=25;cells=[100]*(width*width)
        for y in range(2,23):
            for x in range(2,23):cells[y*width+x]=0 if x<18 else -1
        result=select_frontier(cells,width,width,12*width+8)
        self.assertEqual(result['state'],'exploring');self.assertEqual(cells[result['goal']],0)
        x,y=result['goal']%width,result['goal']//width
        self.assertLessEqual(x,14);self.assertGreaterEqual(y,5)
    def test_fully_observed_enclosure_completes(self):
        width=15;cells=[100]*(width*width)
        for y in range(1,14):
            for x in range(1,14):cells[y*width+x]=0
        self.assertEqual(select_frontier(cells,width,width,7*width+7)['state'],'complete')
    def test_frontier_attempt_exhaustion_is_blocked_not_complete(self):
        width=20;cells=[0 if x<15 else -1 for y in range(width) for x in range(width)]
        result=select_frontier(cells,width,width,10*width+8,excluded=range(len(cells)))
        self.assertEqual(result['state'],'blocked')
    def test_unknown_footprint_waits(self):
        self.assertEqual(select_frontier([-1]*100,10,10,55)['state'],'waiting')
