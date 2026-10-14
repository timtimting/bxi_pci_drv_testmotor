from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    defaults = {
        "interface": "enp86s0", "init_pci": "false", "auto_power_on": "false",
        "auto_scan": "true", "feedback_hz": "100.0", "default_hold_ms": "1000",
        "config_file": os.path.join(get_package_share_directory("bxi_motor_control"), "config", "motor_console.yaml"),
    }
    types = {"init_pci": bool, "auto_power_on": bool, "auto_scan": bool,
             "feedback_hz": float, "default_hold_ms": int}
    return LaunchDescription(
        [DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()] +
        [Node(package="bxi_motor_control", executable="motor_node", name="bxi_motor_control",
              output="screen", cwd=get_package_share_directory("bxi_motor_control"), parameters=[{
                  k: ParameterValue(LaunchConfiguration(k), value_type=types.get(k, str))
                  for k in defaults
              }])]
    )
