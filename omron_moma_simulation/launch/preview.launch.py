import copy
import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.conditions import evaluate_condition_expression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def create_preview(context):
    preview = os.path.join(
        get_package_share_directory('omron_amr_simulation'), 'launch', 'preview.launch.py')
    gripper = LaunchConfiguration('gripper').perform(context)
    descriptions = {
        'none': ('moma_description', 'xacro/ld250_tm12x.urdf.xacro'),
        'soft_two_fingers': (
            'grasping_description_omron_moma',
            'xacro/ld250_tm12x_soft_two_fingers/ld250-tm12x-soft-two-fingers.urdf.xacro',
        ),
    }
    if gripper not in descriptions:
        raise ValueError(f'Unsupported gripper: {gripper}; choose {", ".join(descriptions)}')
    package, description_file = descriptions[gripper]
    world_name = LaunchConfiguration('world').perform(context)
    gazebo_worlds = {'empty': 'empty', 'grasp_test': 'grasp_test', 'warehouse': 'industrial-warehouse'}
    if world_name not in gazebo_worlds:
        raise ValueError(f'Unsupported world: {world_name}; choose {", ".join(gazebo_worlds)}')
    fixed_base_control = evaluate_condition_expression(context, [LaunchConfiguration('fixed_base_control')])
    description_path = os.path.join(get_package_share_directory(package), description_file)
    root = ET.fromstring(xacro.process_file(description_path).toxml())
    for gazebo in root.findall('gazebo'):
        for plugin in gazebo.findall('plugin'):
            if plugin.get('filename') == 'libgazebo_ros_control.so':
                gazebo.remove(plugin)
    gripper_joints = ('gripper_planar_4', 'gripper_planar_5')
    available_joints = {joint.get('name') for joint in root.findall('joint')}
    has_gripper_joints = all(name in available_joints for name in gripper_joints)
    if fixed_base_control:
        ET.SubElement(root, 'link', {'name': 'world'})
        anchor = ET.SubElement(root, 'joint', {'name': 'simulation_world_anchor', 'type': 'fixed'})
        ET.SubElement(anchor, 'parent', {'link': 'world'})
        ET.SubElement(anchor, 'child', {'link': 'base_link'})
        control = ET.SubElement(root, 'ros2_control', {'name': 'GazeboSimSystem', 'type': 'system'})
        ET.SubElement(ET.SubElement(control, 'hardware'), 'plugin').text = 'gz_ros2_control/GazeboSimSystem'
        joints = [f'joint_{number}' for number in range(1, 7)]
        joints.extend(name for name in gripper_joints if name in available_joints)
        for name in joints:
            joint = ET.SubElement(control, 'joint', {'name': name})
            ET.SubElement(joint, 'command_interface', {'name': 'position'})
            ET.SubElement(joint, 'state_interface', {'name': 'position'})
            ET.SubElement(joint, 'state_interface', {'name': 'velocity'})
        plugin = ET.SubElement(ET.SubElement(root, 'gazebo'), 'plugin', {
            'filename': 'libgz_ros2_control-system.so',
            'name': 'gz_ros2_control::GazeboSimROS2ControlPlugin',
        })
        ET.SubElement(plugin, 'parameters').text = os.path.join(
            get_package_share_directory('omron_moma_simulation'), 'config', 'gazebo_controllers.yaml')
    else:
        ET.SubElement(ET.SubElement(root, 'gazebo'), 'static').text = 'true'
    description = ET.tostring(root, encoding='unicode')
    gazebo_root = copy.deepcopy(root)
    if fixed_base_control and has_gripper_joints:
        for link in gazebo_root.findall('link'):
            name = link.get('name', '')
            if name.startswith('gripper_') and not name.startswith('gripper_soft_gripper_finger'):
                for collision in link.findall('collision'):
                    link.remove(collision)
    for visual in gazebo_root.findall('.//visual'):
        mesh = visual.find('geometry/mesh')
        material = visual.find('material')
        if mesh is not None and mesh.get('filename', '').lower().endswith('.obj') and material is not None:
            visual.remove(material)
    gazebo_description = ET.tostring(gazebo_root, encoding='unicode')

    resource_packages = set()
    for mesh in root.findall('.//mesh'):
        filename = mesh.get('filename', '')
        if filename.startswith('package://'):
            resource_packages.add(filename[len('package://'):].split('/', 1)[0])
    resource_paths = [os.path.dirname(get_package_share_directory(name)) for name in sorted(resource_packages)]
    resource_paths.append(context.environment.get('GZ_SIM_RESOURCE_PATH', ''))
    nodes = [SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.pathsep.join(resource_paths)), IncludeLaunchDescription(
        PythonLaunchDescriptionSource(preview),
        launch_arguments={
            'world_only': 'true',
            'gui': LaunchConfiguration('gui'),
            'world': LaunchConfiguration('world'),
            'randomize_boxes': LaunchConfiguration('randomize_boxes'),
            'box_seed': LaunchConfiguration('box_seed'),
            'randomize_objects': LaunchConfiguration('randomize_objects'),
            'object_seed': LaunchConfiguration('object_seed'),
        }.items(),
    ),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': description, 'use_sim_time': True}], output='screen'),
        Node(package='ros_gz_sim', executable='create',
             arguments=['-world', gazebo_worlds[world_name], '-name', 'ld250_tm12x', '-string', gazebo_description,
                        '-x', LaunchConfiguration('x'), '-y', LaunchConfiguration('y'),
                        '-z', LaunchConfiguration('z'), '-Y', LaunchConfiguration('yaw')], output='screen'),
    ]
    if fixed_base_control:
        for controller in ('joint_state_broadcaster', 'tmr_arm_with_base_controller'):
            nodes.append(Node(package='controller_manager', executable='spawner',
                              arguments=[controller, '--controller-manager-timeout', '120'], output='screen'))
        if has_gripper_joints:
            nodes.append(Node(package='controller_manager', executable='spawner',
                              arguments=['gripper_position_controller', '--controller-manager-timeout', '120'], output='screen'))
    else:
        nodes.append(Node(package='joint_state_publisher', executable='joint_state_publisher',
                          parameters=[{'use_sim_time': True}], output='screen'))
    if gripper == 'soft_two_fingers' and fixed_base_control:
        gripper_config = os.path.join(get_package_share_directory('gripper_ros'), 'config', 'grippers', 'soft_two_finger_dynamixel.yaml')
        nodes.append(Node(package='omron_moma_simulation', executable='sim_gripper_action.py',
                          name='gripper_two_fingers_node', parameters=[gripper_config, {'use_sim_time': True}], output='screen'))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', description='Start the Gazebo graphical client'),
        DeclareLaunchArgument('world', default_value='empty', description='empty, grasp_test, or warehouse'),
        DeclareLaunchArgument('randomize_boxes', default_value='false', description='Randomize warehouse box positions'),
        DeclareLaunchArgument('box_seed', default_value='', description='Optional seed for warehouse box positions'),
        DeclareLaunchArgument('randomize_objects', default_value='false', description='Randomize grasp-test object positions'),
        DeclareLaunchArgument('object_seed', default_value='', description='Optional seed for grasp-test object positions'),
        DeclareLaunchArgument('fixed_base_control', default_value='false', description='Enable fixed-base Gazebo arm control'),
        DeclareLaunchArgument('x', default_value='0.0', description='Spawn X position in meters'),
        DeclareLaunchArgument('y', default_value='0.0', description='Spawn Y position in meters'),
        DeclareLaunchArgument('z', default_value='0.0', description='Spawn Z position in meters'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='Spawn yaw in radians'),
        DeclareLaunchArgument('gripper', default_value='none', description='none or soft_two_fingers'),
        OpaqueFunction(function=create_preview),
    ])