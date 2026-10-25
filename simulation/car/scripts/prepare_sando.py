#!/usr/bin/env python3
"""Copy the planner core into a separate package; never edit the drone checkout."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import xml.etree.ElementTree as E

SOURCES=['src/sando/sando_node.cpp','src/sando/sando.cpp','src/hgp/hgp_manager.cpp','src/hgp/utils.cpp','src/hgp/hgp_planner.cpp','src/hgp/graph_search.cpp','src/sando/utils.cpp','src/sando/gurobi_solver.cpp']
DEPS=['rclcpp','dynus_interfaces','visualization_msgs','decomp_util','decomp_ros_msgs','sensor_msgs','std_msgs','geometry_msgs','nav_msgs','message_filters','pcl_conversions','pcl_ros','tf2','tf2_ros','tf2_geometry_msgs','tf2_eigen','std_srvs','nlohmann_json']

def patched_files(source):
    files={p:(source/p).read_text() for p in SOURCES}
    header='include/sando/gurobi_solver.hpp';files[header]=(source/header).read_text()
    code=files['src/sando/sando.cpp']
    for name,count in [('local_A',2),('local_G',1),('local_E',1)]:
        old=f'{name}.pos[2] = 1.0;'
        if code.count(old)!=count:raise ValueError('Unrecognized SANDO ground-height implementation: '+name)
        code=code.replace(old,f'{name}.pos[2] = par_.default_goal_z;\n    {name}.vel[2] = 0.0;\n    {name}.accel[2] = 0.0;')
    files['src/sando/sando.cpp']=code
    files['src/sando/sando_node.cpp']=files['src/sando/sando_node.cpp'].replace('"Vehicle Type: %d", par_.vehicle_type','"Vehicle Type: %s", par_.vehicle_type.c_str()')
    old='  double z_min_;';new=old+'\n  bool car_planar_ = false;\n  double car_plane_z_ = 0.0;'
    if files[header].count(old)!=1:raise ValueError('Unrecognized solver header')
    files[header]=files[header].replace(old,new)
    solver=files['src/sando/gurobi_solver.cpp'];old='  z_min_ = par.z_min;'
    if solver.count(old)!=1:raise ValueError('Unrecognized solver initialization')
    solver=solver.replace(old,old+'\n  car_planar_ = par.vehicle_type != "uav";\n  car_plane_z_ = par.default_goal_z;')
    old='    // Set the constraints for the map size (x, y, and z)'
    if solver.count(old)!=1:raise ValueError('Unrecognized solver map constraints')
    solver=solver.replace(old,'''    // CAR_PLANAR_V1: equal Z for every cubic Bezier control point makes
    // the ENTIRE segment planar, including its velocity/acceleration/jerk.
    if (car_planar_) {
      for (const auto& cp : {cp0, cp1, cp2, cp3}) {
        map_cons_.push_back(m_.addConstr(cp[2] == car_plane_z_));
      }
    }

'''+old)
    files['src/sando/gurobi_solver.cpp']=solver
    return files


def prepare(source,output):
    source=source.resolve();output=output.resolve()
    if source==output or source in output.parents or output in source.parents:raise ValueError('Output must be separate from the source checkout')
    if output.exists():raise FileExistsError('Output exists; use a new directory to avoid overwriting work')
    files=patched_files(source) # Validate before creating any output.
    for path in SOURCES+['FindGUROBI.cmake','LICENSE','config/sando.yaml']:
        if not (source/path).is_file():raise FileNotFoundError(path)
    output.mkdir(parents=True)
    shutil.copytree(source/'include',output/'include')
    utility=Path(__file__).resolve().parents[1]/'planner/dependencies/DecompROS2/decomp_rviz_plugins/include/decomp_rviz_plugins/data_ros_utils.hpp'
    (output/'include/decomp_rviz_plugins').mkdir(exist_ok=True)
    shutil.copy2(utility,output/'include/decomp_rviz_plugins/data_ros_utils.hpp')
    for path in SOURCES+['FindGUROBI.cmake','LICENSE','config/sando.yaml']:
        target=output/path;target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(source/path,target)
    for path,content in files.items():(output/path).write_text(content)
    finder=output/'FindGUROBI.cmake';finder.write_text(finder.read_text().replace('NAMES gurobi gurobi100 gurobi110','NAMES gurobi gurobi100 gurobi110 gurobi120 gurobi130'))
    cmake='''cmake_minimum_required(VERSION 3.16)
project(car_sando_planner)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-O2)
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenMP REQUIRED)
'''
    cmake+='\n'.join(f'find_package({d} REQUIRED)' for d in DEPS)+'\n'
    cmake+='''list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}")
find_package(GUROBI REQUIRED)
file(GLOB CAR_GUROBI_SO "$ENV{GUROBI_HOME}/lib/libgurobi*[0-9].so")
if(NOT CAR_GUROBI_SO)
  message(FATAL_ERROR "Set GUROBI_HOME to an installed Linux Gurobi distribution")
endif()
include_directories(include ${EIGEN3_INCLUDE_DIR} ${PCL_INCLUDE_DIRS} ${DECOMP_UTIL_INCLUDE_DIRS} ${GUROBI_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
'''
    cmake+='add_executable(sando '+' '.join(SOURCES)+')\n'
    cmake+='ament_target_dependencies(sando '+' '.join(DEPS)+')\n'
    cmake+='''target_link_libraries(sando "$ENV{GUROBI_HOME}/lib/libgurobi_c++.a" ${CAR_GUROBI_SO} ${PCL_LIBRARIES} OpenMP::OpenMP_CXX Eigen3::Eigen nlohmann_json::nlohmann_json)
install(TARGETS sando DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES LICENSE source_manifest.json DESTINATION share/${PROJECT_NAME})
ament_package()
'''
    (output/'CMakeLists.txt').write_text(cmake)
    package=E.Element('package',format='3')
    for key,value in [('name','car_sando_planner'),('version','0.1.0'),('description','Isolated planar SANDO core for the SLAMdunk car'),('license','BSD-3-Clause')]:E.SubElement(package,key).text=value
    E.SubElement(package,'maintainer',email='team@example.com').text='SLAMdunk team'
    E.SubElement(package,'buildtool_depend').text='ament_cmake'
    for dep in DEPS+['eigen','libpcl-all-dev']:E.SubElement(package,'depend').text=dep
    E.SubElement(E.SubElement(package,'export'),'build_type').text='ament_cmake'
    E.indent(package);E.ElementTree(package).write(output/'package.xml',encoding='utf-8',xml_declaration=True)
    commit=subprocess.run(['git','rev-parse','HEAD'],cwd=source,capture_output=True,text=True,check=True).stdout.strip()
    manifest={'source_commit':commit,'source_files_sha256':{p:hashlib.sha256((source/p).read_bytes()).hexdigest() for p in SOURCES},'adaptation':'CAR_PLANAR_V1','changes':['ground endpoints use default_goal_z with zero vertical derivatives','all cubic Bezier Z control points constrained to ground plane','planner-only build; no Gazebo Classic plugins or fake_sim','planner-only dependency metadata and copied unchanged DecompROS header','correct vehicle_type string logging'],'limits':'Skid-steer tracking is separate; upstream UAV tracking/safety guarantees are not established for this car.'}
    (output/'source_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print(output)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--source',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();prepare(a.source,a.output)
