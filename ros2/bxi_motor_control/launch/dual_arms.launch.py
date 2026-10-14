"""Two independent EtherLab masters; right starts after left PDO is ready."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessIO, OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('bxi_motor_control')
    defaults = {'left_interface': 'enp86s0', 'right_interface': 'enx6c1ff7c503f3',
                'init_pci': 'true', 'auto_power_on': 'true'}
    joints = ['shoulder_y_joint', 'shoulder_x_joint', 'shoulder_z_joint',
              'elbow_y_joint', 'wrist_x_joint', 'wrist_y_joint', 'wrist_z_joint']

    def arm(side):
        left = side == 'left'
        return Node(package='bxi_motor_control', executable='motor_node',
                    name=side + '_motor_control', output='screen', cwd=share,
                    parameters=[{
                        'interface': LaunchConfiguration(side + '_interface'),
                        'config_file': os.path.join(share, 'config', 'motor_console.yaml'),
                        'joint_names': [('l_' if left else 'r_') + j for j in joints],
                        'slave_ids': list(range(1, 8)), 'auto_scan': True,
                        'init_pci': ParameterValue(LaunchConfiguration('init_pci'), value_type=bool) if left else False,
                        'auto_power_on': ParameterValue(LaunchConfiguration('auto_power_on'), value_type=bool) if left else False,
                        'topic_prefix': '/hardware/', 'ignore_unmapped_joints': True,
                    }], remappings=[('/hardware/actuator_states', '/hardware/' + side + '/actuator_states')])

    left, right = arm('left'), arm('right')
    started = False
    pending = ''

    def left_output(event):
        nonlocal started, pending
        pending = (pending + event.text.decode(errors='replace'))[-8192:]
        if not started and 'Ready: PDO' in pending:
            started = True
            return [right]
        return []

    return LaunchDescription(
        [DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()] + [
            RegisterEventHandler(OnProcessIO(target_action=left, on_stdout=left_output, on_stderr=left_output)),
            RegisterEventHandler(OnProcessExit(target_action=left,
                on_exit=[EmitEvent(event=Shutdown(reason='Left motor backend exited'))])),
            RegisterEventHandler(OnProcessExit(target_action=right,
                on_exit=[EmitEvent(event=Shutdown(reason='Right motor backend exited'))])),
            left,
        ])
