import os
from glob import glob

from setuptools import setup

package_name = "oamr_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="mustafaerkam",
    maintainer_email="hekimhanerkam@gmail.com",
    description="ROS 2 ile OAMR ESP32 low-level firmware arasindaki V1 seri koprusu.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "serial_bridge = oamr_bridge.serial_bridge_node:main",
        ],
    },
)
