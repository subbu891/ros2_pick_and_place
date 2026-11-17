import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    desc_pkg = get_package_share_directory('pnp_description')
    moveit_pkg = get_package_share_directory('pnp_moveit_config')

    # Collects URDF + SRDF + all config files into one set of parameters
    cfg = (
        MoveItConfigsBuilder('ur5e_pnp', package_name='pnp_moveit_config')
        .robot_description(file_path=os.path.join(desc_pkg, 'urdf', 'ur5e_pnp.urdf.xacro'))
        .robot_description_semantic(file_path='config/ur5e_pnp.srdf')
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        .joint_limits(file_path='config/joint_limits.yaml')
        .trajectory_execution(file_path='config/moveit_controllers.yaml')
        .planning_pipelines(pipelines=['ompl'])
        .to_moveit_configs()
    )

    controllers = os.path.join(desc_pkg, 'config', 'ros2_controllers.yaml')
    spawners = [
        Node(package='controller_manager', executable='spawner',
             arguments=[name, '--controller-manager', '/controller_manager'])
        for name in ['joint_state_broadcaster', 'arm_controller', 'gripper_controller']
    ]

    return LaunchDescription([
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[cfg.robot_description]),
        # Controller manager now reads the URDF from the topic (fixes the deprecation warning)
        Node(package='controller_manager', executable='ros2_control_node',
             parameters=[controllers],
             remappings=[('~/robot_description', '/robot_description')],
             output='screen'),
        *spawners,
        # The MoveIt brain
        Node(package='moveit_ros_move_group', executable='move_group',
             parameters=[cfg.to_dict()], output='screen'),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', os.path.join(moveit_pkg, 'rviz', 'moveit.rviz')],
             parameters=[cfg.robot_description, cfg.robot_description_semantic, cfg.planning_pipelines]),
    ])
