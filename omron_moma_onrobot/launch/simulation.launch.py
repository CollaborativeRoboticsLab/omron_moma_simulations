import copy
import importlib.util
import math
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


def get_vgc10_targets(world_name):
    simulation_share = get_package_share_directory('omron_amr_simulation')
    models = []
    if world_name == 'grasp_test':
        world_path = os.path.join(simulation_share, 'worlds', 'grasp_test.sdf')
        world = ET.parse(world_path).getroot().find('world')
        models = [
            (include.findtext('name'), include.findtext('uri').removeprefix('model://'))
            for include in world.findall('include')
            if include.findtext('uri', '').startswith('model://manipulation_')
        ]
    elif world_name == 'warehouse':
        models = [
            (f'warehouse_box_{color}_{number}', f'warehouse_box_{color}')
            for color in ('blue', 'red', 'green', 'black', 'brown')
            for number in range(1, 5)
        ]

    targets = []
    for model_name, model_resource in models:
        model_path = os.path.join(simulation_share, 'models', model_resource, 'model.sdf')
        model = ET.parse(model_path).getroot().find('model')
        link = model.find('link') if model is not None else None
        if link is None:
            raise ValueError(f'No link found in simulation model: {model_resource}')
        targets.append((model_name, link.get('name')))
    return targets


def add_vgc10_simulation(root, targets):
    contact_topics = []
    cups = ('suction1', 'suction2', 'suction3', 'suction4')
    suction_link = root.find("link[@name='suction']")
    if suction_link is None:
        raise ValueError('VGC10 contact simulation requires the central suction link')
    inertial = ET.SubElement(suction_link, 'inertial')
    ET.SubElement(inertial, 'mass', {'value': '0.001'})
    ET.SubElement(inertial, 'inertia', {
        'ixx': '0.000001', 'ixy': '0', 'ixz': '0',
        'iyy': '0.000001', 'iyz': '0', 'izz': '0.000001',
    })
    for joint_name in ('_suction_joint', *(f'_{cup}_joint' for cup in cups)):
        gazebo = ET.SubElement(root, 'gazebo', {'reference': joint_name})
        ET.SubElement(gazebo, 'preserveFixedJoint').text = 'true'
    for cup in cups:
        link = root.find(f"link[@name='{cup}']")
        collision = link.find('collision') if link is not None else None
        if collision is None:
            raise ValueError(f'VGC10 contact simulation requires collision geometry on {cup}')
        collision.set('name', f'{cup}_collision')
        topic = f'/vgc10/contacts/{cup}'
        contact_topics.append(topic)
        sensor = ET.SubElement(ET.SubElement(root, 'gazebo', {'reference': cup}), 'sensor', {
            'name': f'{cup}_contact', 'type': 'contact',
        })
        ET.SubElement(sensor, 'topic').text = topic
        ET.SubElement(sensor, 'update_rate').text = '50'
        ET.SubElement(sensor, 'always_on').text = 'true'
        contact = ET.SubElement(sensor, 'contact')
        ET.SubElement(contact, 'collision').text = f'{cup}_collision'

    target_keys = []
    attach_topics = []
    detach_topics = []
    state_topics = []
    for model_name, child_link in targets:
        for cup in cups:
            key = f'{cup}::{model_name}'
            target_keys.append(key)
            attach_topics.append(f'/vgc10/attach/{cup}/{model_name}')
            detach_topics.append(f'/vgc10/detach/{cup}/{model_name}')
            state_topics.append(f'/vgc10/state/{cup}/{model_name}')
            plugin = ET.SubElement(ET.SubElement(root, 'gazebo'), 'plugin', {
                'filename': 'gz-sim-detachable-joint-system',
                'name': 'gz::sim::systems::DetachableJoint',
            })
            ET.SubElement(plugin, 'parent_link').text = cup
            ET.SubElement(plugin, 'child_model').text = model_name
            ET.SubElement(plugin, 'child_link').text = child_link
            ET.SubElement(plugin, 'attach_topic').text = attach_topics[-1]
            ET.SubElement(plugin, 'detach_topic').text = detach_topics[-1]
            ET.SubElement(plugin, 'output_topic').text = state_topics[-1]
    return contact_topics, target_keys, attach_topics, detach_topics, state_topics


def create_preview(context):
    preview = os.path.join(
        get_package_share_directory('omron_amr_simulation'), 'launch', 'simulation.launch.py')
    gripper = LaunchConfiguration('gripper').perform(context)
    descriptions = {
        'none': ('moma_description', 'xacro/ld250_tm12x.urdf.xacro'),
        'rg2': ('grasping_description_onrobot_omron_moma', 'xacro/ld250_tm12x_rg2.urdf.xacro'),
        'rg6': ('grasping_description_onrobot_omron_moma', 'xacro/ld250_tm12x_rg6.urdf.xacro'),
        'vgc10': ('grasping_description_onrobot_omron_moma', 'xacro/ld250_tm12x_vgc10.urdf.xacro'),
    }
    if gripper not in descriptions:
        raise ValueError(f'Unsupported gripper: {gripper}; choose {", ".join(descriptions)}')
    package, description_file = descriptions[gripper]
    world_name = LaunchConfiguration('world').perform(context)
    gazebo_worlds = {'empty': 'empty', 'grasp_test': 'grasp_test', 'warehouse': 'industrial-warehouse'}
    if world_name not in gazebo_worlds:
        raise ValueError(f'Unsupported world: {world_name}; choose {", ".join(gazebo_worlds)}')
    fixed_base_control = evaluate_condition_expression(context, [LaunchConfiguration('fixed_base_control')])
    mobile_base_control = evaluate_condition_expression(context, [LaunchConfiguration('mobile_base_control')])
    lidar = evaluate_condition_expression(context, [LaunchConfiguration('lidar')])
    docking = evaluate_condition_expression(context, [LaunchConfiguration('docking')])
    if docking and not mobile_base_control:
        raise ValueError('Docking simulation requires mobile_base_control:=true')
    if docking:
        dock_x, dock_y, dock_yaw = (float(LaunchConfiguration(name).perform(context))
                                     for name in ('dock_x', 'dock_y', 'dock_yaw'))
        if not all(math.isfinite(value) for value in (dock_x, dock_y, dock_yaw)):
            raise ValueError('Dock coordinates must be finite')
    if fixed_base_control and mobile_base_control:
        raise ValueError('fixed_base_control and mobile_base_control cannot both be enabled')
    description_path = os.path.join(get_package_share_directory(package), description_file)
    mappings = {
        name: LaunchConfiguration(name).perform(context)
        for name in ('mount_xyz', 'mount_rpy', 'tcp_xyz', 'tcp_rpy')
    } if gripper in ('rg2', 'rg6', 'vgc10') else None
    root = ET.fromstring(xacro.process_file(description_path, mappings=mappings).toxml())
    vgc10_targets = []
    contact_topics = []
    vgc10_target_keys = []
    attach_topics = []
    detach_topics = []
    state_topics = []
    if gripper == 'vgc10' and (fixed_base_control or mobile_base_control):
        vgc10_targets = get_vgc10_targets(world_name)
        contact_topics, vgc10_target_keys, attach_topics, detach_topics, state_topics = add_vgc10_simulation(
            root, vgc10_targets)
    onrobot_control_joints = []
    if gripper in ('rg2', 'rg6'):
        for gripper_control in root.findall('ros2_control'):
            onrobot_control_joints.extend(gripper_control.findall('joint'))
            root.remove(gripper_control)
    for gazebo in root.findall('gazebo'):
        for plugin in gazebo.findall('plugin'):
            if plugin.get('filename') in (
                    'libgazebo_ros_control.so', 'libgz_ros2_control-system.so'):
                gazebo.remove(plugin)
    has_gripper_joints = bool(onrobot_control_joints)
    if mobile_base_control or lidar:
        spec = importlib.util.spec_from_file_location('base_drive', os.path.join(
            get_package_share_directory('omron_amr_simulation'), 'launch', 'base_drive.py'))
        base_drive = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(base_drive)
    if fixed_base_control or mobile_base_control:
        if fixed_base_control:
            ET.SubElement(root, 'link', {'name': 'world'})
            anchor = ET.SubElement(root, 'joint', {'name': 'simulation_world_anchor', 'type': 'fixed'})
            ET.SubElement(anchor, 'parent', {'link': 'world'})
            ET.SubElement(anchor, 'child', {'link': 'base_link'})
        else:
            base_drive.add_drive_geometry(root, body_name='ld250_main_body')
        control = ET.SubElement(root, 'ros2_control', {'name': 'GazeboSimSystem', 'type': 'system'})
        ET.SubElement(ET.SubElement(control, 'hardware'), 'plugin').text = 'gz_ros2_control/GazeboSimSystem'
        for name in [f'joint_{number}' for number in range(1, 7)]:
            joint = ET.SubElement(control, 'joint', {'name': name})
            ET.SubElement(joint, 'command_interface', {'name': 'position'})
            ET.SubElement(joint, 'state_interface', {'name': 'position'})
            ET.SubElement(joint, 'state_interface', {'name': 'velocity'})
        for joint in onrobot_control_joints:
            control.append(joint)
        plugin = ET.SubElement(ET.SubElement(root, 'gazebo'), 'plugin', {
            'filename': 'libgz_ros2_control-system.so',
            'name': 'gz_ros2_control::GazeboSimROS2ControlPlugin',
        })
        ET.SubElement(plugin, 'parameters').text = os.path.join(
            get_package_share_directory('omron_moma_onrobot'), 'config', 'gazebo_controllers.yaml')
    else:
        ET.SubElement(ET.SubElement(root, 'gazebo'), 'static').text = 'true'
    if lidar:
        base_drive.add_lidars(root, prefixed_frames=True)
    description = ET.tostring(root, encoding='unicode')
    gazebo_root = copy.deepcopy(root)
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
            'physics_engine': 'bullet' if has_gripper_joints and fixed_base_control or contact_topics else 'dart',
            'gui': LaunchConfiguration('gui'),
            'world': LaunchConfiguration('world'),
            'contact_sensors': 'true' if contact_topics else 'false',
            'lidar': LaunchConfiguration('lidar'),
            'docking': LaunchConfiguration('docking'),
            'dock_x': LaunchConfiguration('dock_x'),
            'dock_y': LaunchConfiguration('dock_y'),
            'dock_yaw': LaunchConfiguration('dock_yaw'),
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
    if fixed_base_control or mobile_base_control:
        for controller in ('joint_state_broadcaster', 'tmr_arm_with_base_controller'):
            arguments = [controller, '--controller-manager-timeout', '120', '--switch-timeout', '60']
            nodes.append(Node(package='controller_manager', executable='spawner',
                              arguments=arguments, output='screen'))
        if has_gripper_joints:
            arguments = ['joint_position_controller', '--controller-manager-timeout', '120',
                         '--switch-timeout', '60']
            nodes.append(Node(package='controller_manager', executable='spawner',
                              arguments=arguments, output='screen'))
            nodes.append(Node(package='onrobot_control', executable='onrobot_simulated_rg_action_server',
                              namespace=f'onrobot_{gripper}', parameters=[{
                                  'model': gripper, 'use_sim_time': True,
                              }], remappings=[
                                  ('joint_states', '/joint_states'),
                                  ('joint_position_controller/commands', '/joint_position_controller/commands'),
                              ], output='screen'))
    else:
        nodes.append(Node(package='joint_state_publisher', executable='joint_state_publisher',
                          parameters=[{'use_sim_time': True}], output='screen'))
    if mobile_base_control:
        nodes.extend(base_drive.drive_nodes())
    if lidar:
        nodes.extend(base_drive.lidar_nodes())
    if docking:
        nodes.extend(base_drive.docking_nodes('ld250_tm12x', dock_x, dock_y, dock_yaw))
    if contact_topics:
        bridge_arguments = [
            f'{topic}@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts'
            for topic in contact_topics
        ]
        for topic in attach_topics + detach_topics:
            bridge_arguments.append(f'{topic}@std_msgs/msg/Empty]gz.msgs.Empty')
        for topic in state_topics:
            bridge_arguments.append(f'{topic}@std_msgs/msg/Bool[gz.msgs.Boolean')
        nodes.append(Node(package='ros_gz_bridge', executable='parameter_bridge',
                          arguments=bridge_arguments, output='screen'))
        nodes.append(Node(package='omron_moma_onrobot', executable='vgc10_simulator',
                          parameters=[{
                              'target_models': [name for name, _ in vgc10_targets],
                              'target_keys': vgc10_target_keys,
                              'contact_topics': contact_topics,
                              'attach_topics': attach_topics,
                              'detach_topics': detach_topics,
                              'state_topics': state_topics,
                              'use_sim_time': True,
                          }], output='screen'))
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
        DeclareLaunchArgument('mobile_base_control', default_value='false', description='Enable experimental mobile base and arm control'),
        DeclareLaunchArgument('lidar', default_value='false', description='Enable experimental LD250 laser scans'),
        DeclareLaunchArgument('docking', default_value='false', description='Enable the experimental simulated dock'),
        DeclareLaunchArgument('dock_x', default_value='0.0', description='Docked robot X position in world coordinates'),
        DeclareLaunchArgument('dock_y', default_value='0.0', description='Docked robot Y position in world coordinates'),
        DeclareLaunchArgument('dock_yaw', default_value='0.0', description='Docked robot yaw in radians'),
            DeclareLaunchArgument('mount_xyz', default_value='0 0 0'),
            DeclareLaunchArgument('mount_rpy', default_value='0 0 0'),
            DeclareLaunchArgument('tcp_xyz', default_value='0 0 0'),
            DeclareLaunchArgument('tcp_rpy', default_value='0 0 0'),
        DeclareLaunchArgument('x', default_value='0.0', description='Spawn X position in meters'),
        DeclareLaunchArgument('y', default_value='0.0', description='Spawn Y position in meters'),
        DeclareLaunchArgument('z', default_value='0.0', description='Spawn Z position in meters'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='Spawn yaw in radians'),
        DeclareLaunchArgument('gripper', default_value='none', choices=['none', 'rg2', 'rg6', 'vgc10'],
                      description='none, rg2, rg6, or vgc10'),
        OpaqueFunction(function=create_preview),
    ])