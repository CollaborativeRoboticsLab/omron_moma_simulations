import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def include(package, filename, arguments):
    path = os.path.join(get_package_share_directory(package), 'launch', filename)
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(path), launch_arguments=arguments.items())


def generate_launch_description():
    simulation_share = get_package_share_directory('omron_moma_custom')
    grasping_share = get_package_share_directory('grasping_custom_omron_moma')
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('world', default_value='warehouse'),
        DeclareLaunchArgument('camera', default_value='false'),
        DeclareLaunchArgument('docking', default_value='false'),
        DeclareLaunchArgument('run_task_client', default_value='false'),
        DeclareLaunchArgument('task_mode', default_value='arm_only_ik'),
        DeclareLaunchArgument('motion_config_file', default_value=os.path.join(simulation_share, 'config', 'motion_simulation.yaml')),
        DeclareLaunchArgument('task_config_file', default_value=os.path.join(grasping_share, 'config', 'task_feasibility.yaml')),
        DeclareLaunchArgument('task_client_config_file', default_value=os.path.join(grasping_share, 'config', 'mobile_manipulator_control.yaml')),
        GroupAction(actions=[
            SetParameter(name='use_sim_time', value=True),
            include('omron_moma_custom', 'simulation.launch.py', {
                'mobile_base_control': 'true', 'lidar': 'true', 'gripper': 'soft_two_fingers',
                'gui': LaunchConfiguration('gui'), 'world': LaunchConfiguration('world'),
                'camera': LaunchConfiguration('camera'), 'docking': LaunchConfiguration('docking'),
            }),
            include('omron_amr_simulation', 'nav2.launch.py', {'start_sim': 'false'}),
            include('omron_moma_custom', 'moveit.launch.py', {
                'start_sim': 'false', 'gripper': 'soft_two_fingers',
            }),
            include('grasping_control', 'motion_execution.launch.py', {
                'motion_config_file': LaunchConfiguration('motion_config_file'),
                'startup_activate_default_scene': 'false',
                'launch_feasiblity_service': 'true',
            }),
            include('grasping_custom_omron_moma', 'task_feasibility.launch.py', {
                'config_file': LaunchConfiguration('task_config_file'),
            }),
            Node(package='mobile_manipulator_control', executable='mobile_manipulator_control_node',
                 name='mobile_manipulator_control_node', output='screen',
                 condition=IfCondition(LaunchConfiguration('run_task_client')),
                 parameters=[LaunchConfiguration('task_client_config_file'),
                             {'mode': LaunchConfiguration('task_mode')}]),
        ]),
    ])