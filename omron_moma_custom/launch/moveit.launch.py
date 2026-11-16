import importlib.util
import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, NotSubstitution
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def build_moveit_parameters(gripper):
    packages = {
        'none': 'ld250_tm12x_moveit_config',
        'soft_two_fingers': 'ld250_tm12x_soft_two_fingers_moveit_config',
    }
    if gripper not in packages:
        raise ValueError(f'Unsupported gripper: {gripper}')
    package_share = get_package_share_directory(packages[gripper])
    if gripper == 'soft_two_fingers':
        tm_share = get_package_share_directory('tm12x_moveit_config')
        config = (
            MoveItConfigsBuilder('ld250_tm12x_soft_two_fingers', package_name=packages[gripper])
            .robot_description(file_path=os.path.join(package_share, 'config', 'ld250_tm12x_soft_two_fingers.urdf.xacro'))
            .robot_description_semantic(file_path=os.path.join(package_share, 'config', 'ld250_tm12x_soft_two_fingers.srdf'))
            .robot_description_kinematics(file_path=os.path.join(package_share, 'config', 'kinematics.yaml'))
            .trajectory_execution(file_path=os.path.join(package_share, 'config', 'moveit_controllers.yaml'))
            .joint_limits(file_path=os.path.join(tm_share, 'config', 'joint_limits.yaml'))
            .planning_pipelines(default_planning_pipeline='ompl', pipelines=['ompl'])
            .to_moveit_configs()
        )
    else:
        spec = importlib.util.spec_from_file_location(
            'simulation_moveit_config', os.path.join(package_share, 'launch', '_moveit_config.py'))
        config_module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(config_module)
        config = config_module.load_moveit_config()
    parameters = config.to_dict()
    semantic = ET.fromstring(parameters['robot_description_semantic'])
    if gripper == 'none':
        controllers = parameters['moveit_simple_controller_manager']
        controllers['tmr_arm_with_base_controller'] = controllers.pop('tmr_arm_controller')
        controllers['controller_names'] = ['tmr_arm_with_base_controller']
        for joint in semantic.findall('virtual_joint'):
            semantic.remove(joint)
    else:
        group = semantic.find("group[@name='tmr_arm_with_base']")
        group.clear()
        group.set('name', 'tmr_arm_with_base')
        ET.SubElement(group, 'chain', {'base_link': 'base', 'tip_link': 'camera_mount_link'})
        links = {link.get('name') for link in ET.fromstring(parameters['robot_description']).findall('link')}
        for collision in semantic.findall('disable_collisions'):
            if collision.get('link1') not in links or collision.get('link2') not in links:
                semantic.remove(collision)
    parameters['robot_description_semantic'] = ET.tostring(semantic, encoding='unicode')
    parameters['use_sim_time'] = True
    return parameters


def create_moveit(context):
    parameters = build_moveit_parameters(LaunchConfiguration('gripper').perform(context))
    return [Node(package='moveit_ros_move_group', executable='move_group',
                 output='screen', parameters=[parameters])]


def generate_launch_description():
    share = get_package_share_directory('omron_moma_custom')
    return LaunchDescription([
        DeclareLaunchArgument('start_sim', default_value='true'),
        DeclareLaunchArgument('mobile', default_value='false'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('world', default_value='empty'),
        DeclareLaunchArgument('gripper', default_value='soft_two_fingers'),
        DeclareLaunchArgument('lidar', default_value='false'),
        DeclareLaunchArgument('camera', default_value='false'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'simulation.launch.py')),
            condition=IfCondition(LaunchConfiguration('start_sim')),
            launch_arguments={
                'fixed_base_control': NotSubstitution(LaunchConfiguration('mobile')),
                'mobile_base_control': LaunchConfiguration('mobile'),
                'gui': LaunchConfiguration('gui'), 'world': LaunchConfiguration('world'),
                'gripper': LaunchConfiguration('gripper'), 'lidar': LaunchConfiguration('lidar'),
                'camera': LaunchConfiguration('camera'),
            }.items(),
        ),
        OpaqueFunction(function=create_moveit),
    ])