from pathlib import Path
import json
import unittest
import xml.etree.ElementTree as E
ROOT=Path(__file__).resolve().parents[1]

class AssetsTests(unittest.TestCase):
    def test_car_is_wheel_driven_and_has_real_sensors(self):
        model=E.parse(ROOT/'models/car/model.sdf').find('model')
        self.assertEqual(len(model.findall('joint')),4)
        drive=model.find("plugin[@name='gz::sim::systems::DiffDrive']")
        self.assertEqual(len(drive.findall('left_joint')),2);self.assertEqual(len(drive.findall('right_joint')),2)
        self.assertEqual(drive.findtext('wheel_radius'),'0.095')
        types={s.attrib['type'] for s in model.findall('.//sensor')}
        self.assertEqual(types,{'gpu_lidar','rgbd_camera','imu'})
        self.assertFalse(model.findall('.//plugin[@filename="libgazebo_ros_diff_drive.so"]'))
        for i in model.findall('.//inertial'):
            self.assertGreater(float(i.findtext('mass')),0)
            for axis in ('ixx','iyy','izz'):self.assertGreater(float(i.findtext('inertia/'+axis)),0)
    def test_bundled_house_has_single_car_and_required_systems(self):
        world=E.parse(ROOT/'worlds/house_car.sdf').find('world')
        self.assertEqual(len(world.findall("model[@name='car']")),1)
        systems={p.attrib['name'] for p in world.findall('plugin')}
        self.assertIn('gz::sim::systems::Physics',systems);self.assertIn('gz::sim::systems::Sensors',systems)
        self.assertNotIn('gazebo_ros',str(E.tostring(world)))
    def test_isolated_planner_keeps_provenance_and_has_no_classic_build_dependency(self):
        root=ROOT/'planner/car_sando_planner';manifest=json.loads((root/'source_manifest.json').read_text())
        self.assertEqual(manifest['adaptation'],'CAR_PLANAR_V1')
        self.assertTrue((root/'LICENSE').exists())
        cmake=(root/'CMakeLists.txt').read_text();self.assertNotIn('find_package(gazebo',cmake)
        for source in manifest['source_files_sha256']:self.assertTrue((root/source).exists())
