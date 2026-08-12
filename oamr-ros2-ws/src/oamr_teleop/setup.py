from setuptools import setup

package_name = "oamr_teleop"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="mustafaerkam",
    maintainer_email="hekimhanerkam@gmail.com",
    description="Ackermann araci icin canli klavye teleop; /cmd_vel yayinlar.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "ackermann_keyboard = oamr_teleop.ackermann_keyboard_node:main",
        ],
    },
)
