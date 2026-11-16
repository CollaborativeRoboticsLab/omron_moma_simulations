import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetParameter


def include(package, filename, arguments):
    path = os.path.join(get_package_share_directory(package), 'launch', filename)
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(path), launch_arguments=arguments.items())


def generate_launch_description():
    simulation_share = get_package_share_directory('omron_moma_onrobot')
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('world', default_value='warehouse'),
        DeclareLaunchArgument('gripper', default_value='rg2', choices=['none', 'rg2', 'rg6', 'vgc10']),
        DeclareLaunchArgument('docking', default_value='false'),
        DeclareLaunchArgument('motion_config_file', default_value=os.path.join(simulation_share, 'config', 'motion_simulation.yaml')),
        GroupAction(actions=[
            SetParameter(name='use_sim_time', value=True),
            include('omron_moma_onrobot', 'simulation.launch.py', {
                'mobile_base_control': 'true', 'lidar': 'true', 'gripper': LaunchConfiguration('gripper'),
                'gui': LaunchConfiguration('gui'), 'world': LaunchConfiguration('world'),
                'docking': LaunchConfiguration('docking'),
            }),
            include('omron_amr_simulation', 'nav2.launch.py', {'start_sim': 'false'}),
            include('omron_moma_onrobot', 'moveit.launch.py', {
                'start_sim': 'false', 'gripper': LaunchConfiguration('gripper'),
            }),
            include('grasping_control', 'motion_execution.launch.py', {
                'motion_config_file': LaunchConfiguration('motion_config_file'),
                'startup_activate_default_scene': 'false',
                'launch_feasiblity_service': 'true',
            }),
        ]),
    ])