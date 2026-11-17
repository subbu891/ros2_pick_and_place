import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    desc_pkg = get_package_share_directory('pnp_description')
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
    return LaunchDescription([
        Node(package='pnp_manipulation', executable='moveit_hello', output='screen',
             parameters=[cfg.robot_description, cfg.robot_description_semantic,
                         cfg.robot_description_kinematics, {'use_sim_time' : True}]),
    ])
