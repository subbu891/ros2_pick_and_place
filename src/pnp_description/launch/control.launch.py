import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('pnp_description')
    xacro_file = os.path.join(pkg, 'urdf', 'ur5e_pnp.urdf.xacro')
    controllers = os.path.join(pkg, 'config', 'ros2_controllers.yaml')
    robot_description = {
        'robot_description': ParameterValue(Command(['xacro ', xacro_file]), value_type=str)}

    # One spawner per controller: loads, configures and activates it
    spawners = [
        Node(package='controller_manager', executable='spawner',
             arguments=[name, '--controller-manager', '/controller_manager'])
        for name in ['joint_state_broadcaster', 'arm_controller', 'gripper_controller']
    ]

    return LaunchDescription([
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[robot_description]),
        # The controller manager + mock hardware
        Node(package='controller_manager', executable='ros2_control_node',
             parameters=[robot_description, controllers], output='screen'),
        *spawners,
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', os.path.join(pkg, 'rviz', 'view.rviz')]),
    ])
