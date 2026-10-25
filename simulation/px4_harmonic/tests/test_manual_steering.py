import math
import unittest
import xml.etree.ElementTree as ET
from quick_px4_demo.manual import ManualSteering

class ManualTests(unittest.TestCase):
    def setUp(self):
        self.now=10.;self.gateway=ManualSteering(clock=lambda:self.now)
        self.gateway.observe((0,0,2),math.pi/2,True)
    def send(self,kind,owner='browser',**extra):
        return self.gateway.handle(owner,dict(protocol_version=1,type=kind,request_id='req',epoch=self.gateway.epoch,**extra))
    def test_body_direction_and_release(self):
        result=self.send('steer_start',direction='forward');self.assertTrue(result['accepted'])
        vx,vy=self.gateway.velocity();self.assertAlmostEqual(vx,0,places=9);self.assertAlmostEqual(vy,.8)
        self.assertTrue(self.send('steer_release',lease_id=result['lease_id'],seq=1)['accepted'])
        self.assertIsNone(self.gateway.velocity());self.assertEqual(self.gateway.mode,'autonomous')
    def test_lease_expiry_and_no_revival(self):
        result=self.send('steer_start',direction='left');epoch=self.gateway.epoch
        self.now+=.51;self.gateway.observe((0,0,2),0,True)
        self.assertIsNone(self.gateway.velocity())
        self.assertFalse(self.gateway.handle('browser',dict(protocol_version=1,type='steer_renew',request_id='r',epoch=epoch,lease_id=result['lease_id'],seq=1))['accepted'])
    def test_renewal_owner_and_sequence(self):
        result=self.send('steer_start',direction='right')
        self.assertFalse(self.send('steer_renew',owner='other',lease_id=result['lease_id'],seq=1)['accepted'])
        self.assertTrue(self.send('steer_renew',lease_id=result['lease_id'],seq=1)['accepted'])
        self.assertFalse(self.send('steer_renew',lease_id=result['lease_id'],seq=1)['accepted'])
    def test_disconnect_and_stale_feedback(self):
        self.send('steer_start',direction='backward');self.gateway.disconnect('browser')
        self.assertIsNone(self.gateway.velocity())
        self.send('steer_start',direction='forward');self.now+=.31
        self.assertIsNone(self.gateway.velocity())
    def test_collision_latches_and_disables_restarts(self):
        self.send('steer_start',direction='forward');self.gateway.stop('COLLISION DETECTED')
        self.gateway.observe((0,0,2),0,True)
        self.assertIsNone(self.gateway.velocity());self.assertEqual(self.gateway.mode,'fault')
        self.assertFalse(self.send('steer_start',direction='left')['accepted'])
    def test_cancel_before_ack_invalidates_old_epoch(self):
        epoch=self.gateway.epoch;self.assertTrue(self.send('steer_cancel')['accepted'])
        self.assertFalse(self.gateway.handle('browser',dict(protocol_version=1,type='steer_start',request_id='req',epoch=epoch,direction='forward'))['accepted'])
    def test_no_navigation_authority_no_manual(self):
        self.gateway.observe((0,0,2),0,False)
        self.assertFalse(self.send('steer_start',direction='forward')['accepted'])
    def test_solid_world_has_forces_and_contact_reporting(self):
        world=ET.parse('gazebo_worlds/hacktx_forest_solid.sdf').find('world')
        self.assertTrue(all(e.text=='false' for e in world.findall('.//collide_without_contact')))
        self.assertEqual(len(world.findall("model/link/sensor[@type='contact']")),41)
        self.assertTrue(any(p.attrib['name']=='gz::sim::systems::Contact' for p in world.findall('plugin')))

if __name__=='__main__':unittest.main()
