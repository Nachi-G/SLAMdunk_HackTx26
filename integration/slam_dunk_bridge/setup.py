from setuptools import setup

setup(
    name="slam_dunk_bridge", version="0.1.0",
    packages=["slam_dunk_bridge"],
    package_data={"slam_dunk_bridge": ["viewer.html", "web/*.html", "web/*.css", "web/*.mjs"]},
    data_files=[("share/ament_index/resource_index/packages", ["resource/slam_dunk_bridge"]),
                ("share/slam_dunk_bridge", ["package.xml", "README.md", "OPERATOR_UI.md", "SANDO_CONTROL.md"]),
                ("share/slam_dunk_bridge/config", ["config/operator_supervisor.yaml"])],
    install_requires=["setuptools", "websockets==13.1"],
    zip_safe=False, maintainer="SLAMdunk HackTX team", maintainer_email="team@example.com",
    description="Bounded read-only live SANDO map and pose WebSocket bridge",
    license="MIT",
    entry_points={"console_scripts": ["live_map_bridge=slam_dunk_bridge.node:main",
                                      "live_map_demo=slam_dunk_bridge.demo:main",
                                      "mission_supervisor=slam_dunk_bridge.supervisor_node:main"]},
)
