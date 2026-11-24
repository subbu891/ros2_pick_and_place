import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription, RegisterEventHandler,
                            SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


CUBES = ['cube_red_0', 'cube_red_1', 'cube_green_0', 'cube_green_1',
         'cube_blue_0', 'cube_blue_1']


def generate_launch_description():
    desc_pkg = get_package_share_directory('pnp_description')
    moveit_pkg = get_package_share_directory('pnp_moveit_config')
    bringup_pkg = get_package_share_directory('pnp_bringup')
    perception_pkg = get_package_share_directory('pnp_perception')
    manipulation_pkg = get_package_share_directory('pnp_manipulation')
    world = os.path.join(bringup_pkg, 'worlds', 'pnp_world.sdf')
    sim_time = {'use_sim_time': True}

    
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

    #Launch arguments 
    headless_arg = DeclareLaunchArgument(
        'headless', default_value='true',
        description='Run Gazebo without its GUI window (true/false)')
    task_arg = DeclareLaunchArgument(
        'task', default_value='true',
        description='Start the autonomous sorting task (true/false)')
    headless = LaunchConfiguration('headless', default='true')

    #Environment
    software_gl = SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1')
    mesh_path = AppendEnvironmentVariable(
        'IGN_GAZEBO_RESOURCE_PATH',
        os.path.dirname(get_package_share_directory('ur_description')))
    model_path = AppendEnvironmentVariable(
        'IGN_GAZEBO_RESOURCE_PATH', os.path.join(bringup_pkg, 'models'))

    #Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': PythonExpression([
            "'-r ", world, "' + (' -s' if '", headless, "' == 'true' else '')"])}.items())

    spawn = Node(package='ros_gz_sim', executable='create', output='screen',
                 arguments=['-topic', 'robot_description', '-name', 'ur5e_pnp'])

    
    bridge_args = [
        '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
        '/camera/image@sensor_msgs/msg/Image[ignition.msgs.Image',
        '/camera/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image',
        '/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
    ]
    for c in CUBES:
        bridge_args += [
            f'/model/{c}/pose@geometry_msgs/msg/PoseStamped[ignition.msgs.Pose',
            f'/pnp/attach/{c}@std_msgs/msg/Empty]ignition.msgs.Empty',
            f'/pnp/detach/{c}@std_msgs/msg/Empty]ignition.msgs.Empty',
        ]
    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
                  arguments=bridge_args, parameters=[sim_time])

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

    #Robot state + controllers
    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[cfg.robot_description, sim_time])

    controller_activator = Node(
        package='pnp_task', executable='controller_activator', output='screen')
    start_controllers = RegisterEventHandler(
        OnProcessExit(target_action=spawn, on_exit=[controller_activator]))

    #MoveIt + RViz
    move_group = Node(
        package='moveit_ros_move_group', executable='move_group', output='screen',
        parameters=[cfg.to_dict(), sim_time])

    rviz = Node(
        package='rviz2', executable='rviz2',
        arguments=['-d', os.path.join(moveit_pkg, 'rviz', 'moveit.rviz')],
        parameters=[cfg.robot_description, cfg.robot_description_semantic,
                    cfg.planning_pipelines, sim_time])

    #Perception
    detector = Node(
        package='pnp_perception', executable='object_detector', name='object_detector',
        parameters=[os.path.join(perception_pkg, 'config', 'object_detector.yaml')],
        output='screen')

    #Manipulation
    pick_place_server = Node(
        package='pnp_manipulation', executable='pick_place_server', output='screen',
        parameters=[os.path.join(manipulation_pkg, 'config', 'pick_place_server.yaml'),
                    cfg.robot_description, cfg.robot_description_semantic,
                    cfg.robot_description_kinematics, cfg.joint_limits, sim_time])

    grasp_bridge = Node(
        package='pnp_task', executable='gz_grasp_bridge', output='screen',
        parameters=[{'cubes': CUBES}, sim_time])

    #Task layer
    sorter = Node(
        package='pnp_task', executable='sorter', output='screen',
        parameters=[{
            'bin_colors': ['red', 'green', 'blue'],
            'bin_x': [0.30, 0.45, 0.60],     
            'bin_y': [0.35, 0.35, 0.35],
            'workspace': [0.25, 0.75, -0.40, 0.22],
            'settle_time': 4.0,
        }, sim_time],
        condition=IfCondition(LaunchConfiguration('task')))

    return LaunchDescription([
        headless_arg,
        task_arg,
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
        detector,
        pick_place_server,
        grasp_bridge,
        sorter,
    ])
