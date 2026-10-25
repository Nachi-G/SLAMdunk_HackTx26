"""Generate additive LiDAR assets; leave all existing working files untouched."""
from copy import deepcopy
from pathlib import Path
import xml.etree.ElementTree as ET
ROOT = Path(__file__).resolve().parents[1]
SENSOR = '''<sensor name="sando_lidar" type="gpu_lidar">
  <pose>0 0 0 0 0 0</pose>
  <topic>/sando/lidar/scan</topic>
  <gz_frame_id>sando_lidar_link</gz_frame_id>
  <always_on>true</always_on><update_rate>10</update_rate><visualize>false</visualize>
  <lidar><scan>
    <horizontal><samples>720</samples><resolution>1</resolution><min_angle>-3.141592653589793</min_angle><max_angle>3.141592653589793</max_angle></horizontal>
    <vertical><samples>32</samples><resolution>1</resolution><min_angle>-0.523598775598299</min_angle><max_angle>0.523598775598299</max_angle></vertical>
  </scan><range><min>0.3</min><max>25</max><resolution>0.01</resolution></range></lidar>
</sensor>'''
variant = ET.fromstring('''<sdf version="1.9"><model name="x500_sando">
<include merge="true"><uri>model://x500</uri></include>
<link name="sando_lidar_link"><pose relative_to="base_link">0 0 0.30 0 0 0</pose>
<inertial><mass>0.001</mass><inertia><ixx>0.00001</ixx><iyy>0.00001</iyy><izz>0.00001</izz></inertia></inertial></link>
<joint name="sando_lidar_joint" type="fixed"><parent>base_link</parent><child>sando_lidar_link</child></joint>
</model></sdf>''')
variant.find('model/link').append(ET.fromstring(SENSOR))
ET.indent(variant)
ET.ElementTree(variant).write(ROOT/'gazebo_models/x500_sando/model.sdf', encoding='unicode', xml_declaration=True)
(ROOT/'gazebo_models/x500_sando/model.config').write_text('''<?xml version="1.0"?><model><name>x500_sando</name><version>1.0</version><sdf version="1.9">model.sdf</sdf><description>X500 with 360 degree 32 layer SANDO LiDAR. Not flight validated.</description></model>\n''')
world_tree = ET.parse(ROOT/'gazebo_worlds/hacktx_forest.sdf')
world = world_tree.getroot().find('world')
world.set('name', 'sando_lidar_test')
probe = ET.SubElement(world, 'model', name='stationary_lidar_probe')
ET.SubElement(probe, 'static').text = 'true'
ET.SubElement(probe, 'pose').text = '0 0 2.3 0 0 0'
link = ET.SubElement(probe, 'link', name='sando_lidar_link')
link.append(ET.fromstring(SENSOR))
ET.indent(world_tree)
world_tree.write(ROOT/'gazebo_worlds/hacktx_forest_lidar_test.sdf', encoding='unicode', xml_declaration=True)
print('Generated separate model and stationary forest sensor test.')
