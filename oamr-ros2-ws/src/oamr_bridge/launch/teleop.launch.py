"""Seri kopruyu baslatir; klavye teleop'u ayri terminalde calistirilir.

Teleop bilerek bu launch dosyasina dahil edilmemistir: curses tam bir TTY ister
ve launch tarafindan yonetilen surecler ciktilari birlestirdigi icin ekran
bozulur. Dogru kullanim iki terminaldir:

  1. ros2 launch oamr_bridge teleop.launch.py
  2. ros2 run oamr_teleop ackermann_keyboard --ros-args \
         --params-file <bu paketin config/oamr_params.yaml yolu>
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    default_params = os.path.join(
        get_package_share_directory("oamr_bridge"), "config", "oamr_params.yaml")

    params_file_argument = DeclareLaunchArgument(
        "params_file",
        default_value=default_params,
        description="Bridge ve teleop parametrelerini iceren YAML dosyasi.")

    serial_port_argument = DeclareLaunchArgument(
        "serial_port",
        default_value="/dev/ttyUSB0",
        description="ESP32'nin bagli oldugu seri port; YAML degerini ezer.")

    bridge = Node(
        package="oamr_bridge",
        executable="serial_bridge",
        name="oamr_serial_bridge",
        output="screen",
        emulate_tty=True,
        parameters=[
            LaunchConfiguration("params_file"),
            {"serial_port": LaunchConfiguration("serial_port")},
        ],
    )

    return LaunchDescription([
        params_file_argument,
        serial_port_argument,
        bridge,
    ])
