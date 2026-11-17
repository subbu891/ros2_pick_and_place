import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription, RegisterEventHandler,
                            SetEnvironmentVariable)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    desc_pkg = get_package_share_directory('pnp_description')
    moveit_pkg = get_package_share_directory('pnp_moveit_config')
    bringup_pkg = get_package_share_directory('pnp_bringup')
    world = os.path.join(bringup_pkg, 'worlds', 'pnp_world.sdf')
    sim_time = {'use_sim_time': True}

    # ---------------- MoveIt configuration (robot in Gazebo mode) ----------------
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

    # ---------------- Launch arguments ----------------
    headless_arg = DeclareLaunchArgument(
        'headless', default_value='true',
        description='Run Gazebo without its GUI window (true/false)')
    headless = LaunchConfiguration('headless', default='true')

    # ---------------- Environment ----------------
    # Skip the VMware 3D driver (no acceleration) and render on the CPU instead
    software_gl = SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1')
    # Gazebo must find the UR meshes (package://ur_description/...)
    mesh_path = AppendEnvironmentVariable(
        'IGN_GAZEBO_RESOURCE_PATH',
        os.path.dirname(get_package_share_directory('ur_description')))
    # ...and our own models (model://cube_red, model://bin, ...)
    model_path = AppendEnvironmentVariable(
        'IGN_GAZEBO_RESOURCE_PATH', os.path.join(bringup_pkg, 'models'))

    # ---------------- Gazebo ----------------
    # -r: start running immediately; -s: server only (added when headless:=true)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': PythonExpression([
            "'-r ", world, "' + (' -s' if '", headless, "' == 'true' else '')"])}.items())

    # Put the robot into Gazebo, built from the /robot_description topic
    spawn = Node(package='ros_gz_sim', executable='create', output='screen',
                 arguments=['-topic', 'robot_description', '-name', 'ur5e_pnp'])

    # Gazebo -> ROS topics ("[" = Gazebo to ROS only)
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/camera/image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/camera/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
        ],
        parameters=[sim_time])

    # Camera position in TF: must match the overhead_camera pose in the world file.
    #   camera_link: the camera body (X = viewing direction, as in Gazebo)
    #   camera_optical_frame: image convention (Z forward, X right, Y down)
    camera_tf = [
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['--x', '0.50', '--y', '0.05', '--z', '1.40',
                        '--roll', '0', '--pitch', '1.5708', '--yaw', '0',
                        '--frame-id', 'world', '--child-frame-id', 'camera_link'],
             parameters=[sim_time]),
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['--roll', '-1.5708', '--pitch', '0', '--yaw', '-1.5708',
                        '--frame-id', 'camera_link',
                        '--child-frame-id', 'camera_optical_frame'],
             parameters=[sim_time]),
    ]

    # ---------------- Robot state + controllers ----------------
    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[cfg.robot_description, sim_time])

    spawners = [
        Node(package='controller_manager', executable='spawner',
             arguments=[name, '--controller-manager', '/controller_manager'],
             parameters=[sim_time])
        for name in ['joint_state_broadcaster', 'arm_controller', 'gripper_controller']
    ]
    # The controller manager lives inside Gazebo -> spawn controllers after the robot
    start_controllers = RegisterEventHandler(
        OnProcessExit(target_action=spawn, on_exit=spawners))

    # ---------------- MoveIt + RViz ----------------
    move_group = Node(
        package='moveit_ros_move_group', executable='move_group', output='screen',
        parameters=[cfg.to_dict(), sim_time])

    rviz = Node(
        package='rviz2', executable='rviz2',
        arguments=['-d', os.path.join(moveit_pkg, 'rviz', 'moveit.rviz')],
        parameters=[cfg.robot_description, cfg.robot_description_semantic,
                    cfg.planning_pipelines, sim_time])

    # ---------------- Order matters: arguments and environment first ----------------
    return LaunchDescription([
        headless_arg,
        software_gl,
        mesh_path,
        model_path,
        gazebo,
        bridge,
        *camera_tf,
        robot_state_publisher,
        spawn,
        start_controllers,
        move_group,
        rviz,
    ])
