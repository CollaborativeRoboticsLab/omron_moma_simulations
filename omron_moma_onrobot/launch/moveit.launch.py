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


def build_moveit_parameters(gripper, mappings=None):
    if gripper not in ('none', 'rg2', 'rg6', 'vgc10'):
        raise ValueError(f'Unsupported gripper: {gripper}')

    if gripper == 'none':
        package_share = get_package_share_directory('ld250_tm12x_moveit_config')
        spec = importlib.util.spec_from_file_location(
            'simulation_moveit_config', os.path.join(package_share, 'launch', '_moveit_config.py'))
        config_module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(config_module)
        config = config_module.load_moveit_config()
    else:
        packages = {
            'rg2': 'ld250_tm12x_rg2_moveit_config',
            'rg6': 'ld250_tm12x_rg6_moveit_config',
            'vgc10': 'ld250_tm12x_vgc10_moveit_config',
        }
        package_share = get_package_share_directory(packages[gripper])
        tm_share = get_package_share_directory('tm12x_moveit_config')
        description_share = get_package_share_directory('grasping_description_onrobot_omron_moma')
        model = f'ld250_tm12x_{gripper}'
        mappings = mappings or {'mount_xyz': '0 0 0', 'mount_rpy': '0 0 0',
                    'tcp_xyz': '0 0 0', 'tcp_rpy': '0 0 0'}
        config = (
            MoveItConfigsBuilder(model, package_name='ld250_tm12x_onrobot_moveit_config')
            .robot_description(file_path=os.path.join(description_share, 'xacro', f'{model}.urdf.xacro'),
                               mappings={'gripper': gripper, **mappings})
            .robot_description_semantic(file_path=os.path.join(package_share, 'config', f'{model}.srdf.xacro'),
                                        mappings={'gripper': gripper})
            .robot_description_kinematics(file_path=os.path.join(package_share, 'config', 'kinematics.yaml'))
            .trajectory_execution(file_path=os.path.join(package_share, 'config', 'moveit_controllers.yaml'))
            .joint_limits(file_path=os.path.join(tm_share, 'config', 'joint_limits.yaml'))
            .planning_pipelines(default_planning_pipeline='ompl', pipelines=['ompl'])
            .to_moveit_configs()
        )
    parameters = config.to_dict()
    semantic = ET.fromstring(parameters['robot_description_semantic'])
    if gripper == 'none':
        controllers = parameters['moveit_simple_controller_manager']
        controllers['tmr_arm_with_base_controller'] = controllers.pop('tmr_arm_controller')
        controllers['controller_names'] = ['tmr_arm_with_base_controller']
        for joint in semantic.findall('virtual_joint'):
            semantic.remove(joint)
    else:
        links = {link.get('name') for link in ET.fromstring(parameters['robot_description']).findall('link')}
        for collision in semantic.findall('disable_collisions'):
            if collision.get('link1') not in links or collision.get('link2') not in links:
                semantic.remove(collision)
        if gripper in ('rg2', 'rg6'):
            controllers = parameters['moveit_simple_controller_manager']
            controller_name = f'onrobot_{gripper}'
            controllers['controller_names'].append(controller_name)
            controllers[controller_name] = {
                'type': 'GripperCommand',
                'action_ns': 'gripper_command',
                'default': True,
                'joints': ['finger_joint'],
            }
    parameters['robot_description_semantic'] = ET.tostring(semantic, encoding='unicode')
    parameters['use_sim_time'] = True
    return parameters


def create_moveit(context):
    gripper = LaunchConfiguration('gripper').perform(context)
    mappings = {name: LaunchConfiguration(name).perform(context) for name in
                ('mount_xyz', 'mount_rpy', 'tcp_xyz', 'tcp_rpy')}
    parameters = build_moveit_parameters(gripper, mappings)
    return [Node(package='moveit_ros_move_group', executable='move_group',
                 output='screen', parameters=[parameters])]


def generate_launch_description():
    share = get_package_share_directory('omron_moma_onrobot')
    return LaunchDescription([
        DeclareLaunchArgument('start_sim', default_value='true'),
        DeclareLaunchArgument('mobile', default_value='false'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('world', default_value='empty'),
        DeclareLaunchArgument('gripper', default_value='rg2', choices=['none', 'rg2', 'rg6', 'vgc10']),
        DeclareLaunchArgument('lidar', default_value='false'),
        DeclareLaunchArgument('mount_xyz', default_value='0 0 0'),
        DeclareLaunchArgument('mount_rpy', default_value='0 0 0'),
        DeclareLaunchArgument('tcp_xyz', default_value='0 0 0'),
        DeclareLaunchArgument('tcp_rpy', default_value='0 0 0'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'simulation.launch.py')),
            condition=IfCondition(LaunchConfiguration('start_sim')),
            launch_arguments={
                'fixed_base_control': NotSubstitution(LaunchConfiguration('mobile')),
                'mobile_base_control': LaunchConfiguration('mobile'),
                'gui': LaunchConfiguration('gui'), 'world': LaunchConfiguration('world'),
                'gripper': LaunchConfiguration('gripper'), 'lidar': LaunchConfiguration('lidar'),
                'mount_xyz': LaunchConfiguration('mount_xyz'), 'mount_rpy': LaunchConfiguration('mount_rpy'),
                'tcp_xyz': LaunchConfiguration('tcp_xyz'), 'tcp_rpy': LaunchConfiguration('tcp_rpy'),
            }.items(),
        ),
        OpaqueFunction(function=create_moveit),
    ])