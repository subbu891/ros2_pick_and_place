import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, IncludeLaunchDescription,
                            RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    desc_pkg = get_package_share_directory('pnp_description')
    moveit_pkg = get_package_share_directory('pnp_moveit_config')
    bringup_pkg = get_package_share_directory('pnp_bringup')
    world = os.path.join(bringup_pkg, 'worlds', 'pnp_world.sdf')
    sim_time = {'use_sim_time': True}

    # Same MoveIt config as the demo, but the robot description in Gazebo mode
    cfg = (
        MoveItConfigsBuilder('ur5e_pnp', package_name='pnp_moveit_config')
        .robot_description(file_path=os.path.join(desc_pkg, 'urdf', 'ur5e_pnp.urdf.xacro'),
                           mappings={'sim_mode': 'gazebo'})
        .robot_description_semantic(file_path='config/ur5e_pnp.srdf')
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        .joint_limits(file_path='config/joint_limits.yaml')
        .trajectory_execution(file_path='config/moveit_controllers.yaml')
        .planning_pipelines(pipelines=['ompl'])
        .to_moveit_configs()
    )

    # Gazebo must be able to find the UR meshes (package://ur_description/...)
    mesh_path = AppendEnvironmentVariable(
        'IGN_GAZEBO_RESOURCE_PATH',
        os.path.dirname(get_package_share_directory('ur_description')))

    # Start Gazebo with our world; -r = start running immediately
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r {world}'}.items())

    # Put the robot into Gazebo, built from the /robot_description topic
    spawn = Node(package='ros_gz_sim', executable='create', output='screen',
                 arguments=['-topic', 'robot_description', '-name', 'ur5e_pnp'])

    # Gazebo's clock -> ROS /clock  ("[" means: Gazebo to ROS direction only)
    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge',
                  arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
                  output='screen')

    spawners = [
        Node(package='controller_manager', executable='spawner',
             arguments=[name, '--controller-manager', '/controller_manager'],
             parameters=[sim_time])
        for name in ['joint_state_broadcaster', 'arm_controller', 'gripper_controller']
    ]
    # The controller manager lives inside Gazebo, so it only exists after spawning
    start_controllers = RegisterEventHandler(
        OnProcessExit(target_action=spawn, on_exit=spawners))

    return LaunchDescription([
        mesh_path,
        gazebo,
        bridge,
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[cfg.robot_description, sim_time]),
        spawn,
        start_controllers,
        Node(package='moveit_ros_move_group', executable='move_group',
             parameters=[cfg.to_dict(), sim_time], output='screen'),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', os.path.join(moveit_pkg, 'rviz', 'moveit.rviz')],
             parameters=[cfg.robot_description, cfg.robot_description_semantic,
                         cfg.planning_pipelines, sim_time]),
    ])
