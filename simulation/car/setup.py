from glob import glob
from setuptools import setup

setup(name="slamdunk_car",version="0.1.0",packages=["slamdunk_car"],
    data_files=[("share/ament_index/resource_index/packages",["resource/slamdunk_car"]),
      ("share/slamdunk_car",["package.xml","README.md","LICENSE"]),
      ("share/slamdunk_car/launch",glob("launch/*.launch.py")),
      ("share/slamdunk_car/config",glob("config/*.yaml")),
      ("share/slamdunk_car/models/car",glob("models/car/*")),
      ("share/slamdunk_car/worlds",glob("worlds/*.sdf")),
      ("share/slamdunk_car/scripts",glob("scripts/*.py"))],
    install_requires=["setuptools"],zip_safe=False,
    maintainer="SLAMdunk team",maintainer_email="team@example.com",
    description="Gazebo Harmonic car and ground SANDO adapters",license="MIT",
    entry_points={"console_scripts":[
      "drive_guard=slamdunk_car.guard_node:main",
      "car_teleop=slamdunk_car.teleop:main",
      "car_explorer=slamdunk_car.explorer_node:main",
      "sando_adapter=slamdunk_car.sando_node:main"]})
