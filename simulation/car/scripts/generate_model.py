#!/usr/bin/env python3
"""Generate mesh-free Harmonic model and a self-contained static house world."""
from pathlib import Path
import copy
import math
import xml.etree.ElementTree as E

ROOT = Path(__file__).resolve().parents[1]
def sub(parent, tag, text=None, **attrs):
    node=E.SubElement(parent,tag,attrs)
    if text is not None: node.text=str(text)
    return node

def main():
    sdf=E.Element('sdf',version='1.9');model=sub(sdf,'model',name='car')
    sub(model,'pose','0 0 0.02 0 0 0');sub(model,'self_collide','false')
    body=sub(model,'link',name='base_link')
    def inertia(link,mass,values,pose='0 0 0 0 0 0'):
        i=sub(link,'inertial');sub(i,'pose',pose);sub(i,'mass',mass);tensor=sub(i,'inertia')
        for key,value in zip(('ixx','iyy','izz'),values):sub(tensor,key,value)
        for key in ('ixy','ixz','iyz'):sub(tensor,key,0)
    inertia(body,5,(.08,.16,.19),'0 0 .20 0 0 0')
    def shape(link,name,kind,values,pose,color,collision=True):
        for tag in (['visual','collision'] if collision else ['visual']):
            el=sub(link,tag,name=name+'_'+tag);sub(el,'pose',pose);g=sub(sub(el,'geometry'),kind)
            if kind=='box':sub(g,'size',values)
            else:sub(g,'radius',values[0]);sub(g,'length',values[1])
            if tag=='visual':
                mat=sub(el,'material');sub(mat,'ambient',color);sub(mat,'diffuse',color)
            elif name=='tire':
                ode=sub(sub(sub(el,'surface'),'friction'),'ode');sub(ode,'mu',.9);sub(ode,'mu2',.15)
    shape(body,'chassis','box','.54 .34 .14','0 0 .19 0 0 0','.12 .20 .16 1')
    shape(body,'deck','box','.46 .36 .025','0 0 .275 0 0 0','.63 .76 .37 1')
    shape(body,'computer','box','.30 .23 .08','-.035 0 .325 0 0 0','.08 .12 .10 1')
    for x in (-.28,.28):shape(body,'bumper_'+str(x),'box','.025 .35 .055',f'{x} 0 .18 0 0 0','.06 .06 .06 1')
    shape(body,'mast','cylinder',(.018,.26),'0 0 .475 0 0 0','.25 .28 .25 1')
    shape(body,'lidar','cylinder',(.055,.04),'0 0 .625 0 0 0','.57 .76 .30 1')
    shape(body,'camera','box','.05 .12 .045','.25 0 .315 0 0 0','.50 .60 .44 1')
    for y in (-.035,.035):shape(body,'lens_'+str(y),'cylinder',(.014,.008),f'.278 {y} .315 0 {math.pi/2} 0','.03 .04 .05 1',False)
    for x,axle in ((.17,'front'),(-.17,'rear')):
        for y,side in ((.205,'left'),(-.205,'right')):
            name=f'{axle}_{side}_wheel';link=sub(model,'link',name=name)
            sub(link,'pose',f'{x} {y} .095 0 0 0');inertia(link,.25,(.00063,.00113,.00063))
            shape(link,'tire','cylinder',(.095,.055),f'0 0 0 {math.pi/2} 0 0','.045 .045 .045 1')
            shape(link,'hub','cylinder',(.055,.058),f'0 0 0 {math.pi/2} 0 0','.58 .72 .35 1',False)
            joint=sub(model,'joint',name=name+'_joint',type='revolute');sub(joint,'parent','base_link');sub(joint,'child',name)
            axis=sub(joint,'axis');sub(axis,'xyz','0 1 0');limit=sub(axis,'limit');sub(limit,'lower',-1e16);sub(limit,'upper',1e16);sub(limit,'effort',8);sub(limit,'velocity',10)
            sub(sub(axis,'dynamics'),'damping',.01)
    def lidar(name,topic,vertical):
        s=sub(body,'sensor',name=name,type='gpu_lidar');sub(s,'pose','0 0 .60 0 0 0');sub(s,'topic',topic);sub(s,'gz_frame_id',name);sub(s,'update_rate',15);sub(s,'always_on','true')
        l=sub(s,'lidar');scan=sub(l,'scan');h=sub(scan,'horizontal')
        for k,v in [('samples',360),('resolution',1),('min_angle',-math.pi),('max_angle',math.pi)]:sub(h,k,v)
        if vertical:
            v=sub(scan,'vertical')
            for k,val in [('samples',16),('resolution',1),('min_angle',-.65),('max_angle',.35)]:sub(v,k,val)
        r=sub(l,'range')
        for k,v in [('min',.08),('max',15),('resolution',.01)]:sub(r,k,v)
    lidar('laser','/car/scan',False);lidar('lidar3d','/car/lidar',True)
    imu=sub(body,'sensor',name='imu',type='imu');sub(imu,'pose','0 0 .20 0 0 0');sub(imu,'topic','/car/imu');sub(imu,'gz_frame_id','imu_link');sub(imu,'update_rate',100);sub(imu,'always_on','true')
    camera=sub(body,'sensor',name='rgbd',type='rgbd_camera');sub(camera,'pose','.28 0 .315 0 0 0');sub(camera,'topic','/car/rgbd');sub(camera,'gz_frame_id','camera_optical');sub(camera,'update_rate',15);sub(camera,'always_on','true')
    cam=sub(camera,'camera');sub(cam,'horizontal_fov',1.2);im=sub(cam,'image');sub(im,'width',640);sub(im,'height',480);sub(im,'format','R8G8B8');cl=sub(cam,'clip');sub(cl,'near',.08);sub(cl,'far',15)
    drive=sub(model,'plugin',filename='gz-sim-diff-drive-system',name='gz::sim::systems::DiffDrive')
    for axle in ('front','rear'):sub(drive,'left_joint',axle+'_left_wheel_joint');sub(drive,'right_joint',axle+'_right_wheel_joint')
    for k,v in [('wheel_separation',.41),('wheel_radius',.095),('topic','/car/cmd_vel'),('odom_topic','/car/odom'),('tf_topic','/car/tf'),('frame_id','odom'),('child_frame_id','base_link'),('odom_publish_frequency',50),('max_linear_velocity',.4),('min_linear_velocity',-.4),('max_angular_velocity',.8),('min_angular_velocity',-.8),('max_linear_acceleration',.5),('min_linear_acceleration',-.5),('max_angular_acceleration',1.2),('min_angular_acceleration',-1.2)]:sub(drive,k,v)
    E.indent(sdf);E.ElementTree(sdf).write(ROOT/'models/car/model.sdf',encoding='utf-8',xml_declaration=True)
    house=E.parse(ROOT/'worlds/house.sdf');world=house.getroot().find('world');world.append(copy.deepcopy(model))
    E.indent(house);house.write(ROOT/'worlds/house_car.sdf',encoding='utf-8',xml_declaration=True)
    print('Generated model and static house with car at hallway origin.')
if __name__=='__main__':main()
