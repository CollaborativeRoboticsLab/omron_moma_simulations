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


def add_wrist_camera(root):
    if root.find("link[@name='camera_link']") is None:
        raise ValueError('Wrist camera simulation requires camera_link')
    ET.SubElement(root, 'link', {'name': 'camera_color_frame'})
    joint = ET.SubElement(root, 'joint', {'name': 'sim_camera_color_joint', 'type': 'fixed'})
    ET.SubElement(joint, 'parent', {'link': 'camera_link'})
    ET.SubElement(joint, 'child', {'link': 'camera_color_frame'})
    ET.SubElement(joint, 'origin', {'xyz': '0 0.015 0'})
    ET.SubElement(root, 'link', {'name': 'camera_color_optical_frame'})
    joint = ET.SubElement(root, 'joint', {'name': 'sim_camera_color_optical_joint', 'type': 'fixed'})
    ET.SubElement(joint, 'parent', {'link': 'camera_color_frame'})
    ET.SubElement(joint, 'child', {'link': 'camera_color_optical_frame'})
    ET.SubElement(joint, 'origin', {'rpy': f'{-math.pi / 2} 0 {-math.pi / 2}'})
    gazebo = ET.SubElement(root, 'gazebo', {'reference': 'camera_link'})
    sensor = ET.SubElement(gazebo, 'sensor', {'name': 'sim_wrist_color', 'type': 'camera'})
    ET.SubElement(sensor, 'pose').text = '0 0.015 0 0 0 0'
    ET.SubElement(sensor, 'topic').text = '/camera/camera/color/image_raw'
    ET.SubElement(sensor, 'gz_frame_id').text = 'camera_color_optical_frame'
    ET.SubElement(sensor, 'update_rate').text = '15'
    camera = ET.SubElement(sensor, 'camera')
    ET.SubElement(camera, 'camera_info_topic').text = '/camera/camera/color/camera_info'
    ET.SubElement(camera, 'horizontal_fov').text = '1.22173'
    image = ET.SubElement(camera, 'image')
    for name, value in (('width', '640'), ('height', '480'), ('format', 'R8G8B8')):
        ET.SubElement(image, name).text = value
    clip = ET.SubElement(camera, 'clip')
    ET.SubElement(clip, 'near').text = '0.1'
    ET.SubElement(clip, 'far').text = '10.0'
    ET.SubElement(root, 'link', {'name': 'camera_depth_frame'})
    joint = ET.SubElement(root, 'joint', {'name': 'sim_camera_depth_joint', 'type': 'fixed'})
    ET.SubElement(joint, 'parent', {'link': 'camera_link'})
    ET.SubElement(joint, 'child', {'link': 'camera_depth_frame'})
    ET.SubElement(root, 'link', {'name': 'camera_depth_optical_frame'})
    joint = ET.SubElement(root, 'joint', {'name': 'sim_camera_depth_optical_joint', 'type': 'fixed'})
    ET.SubElement(joint, 'parent', {'link': 'camera_depth_frame'})
    ET.SubElement(joint, 'child', {'link': 'camera_depth_optical_frame'})
    ET.SubElement(joint, 'origin', {'rpy': f'{-math.pi / 2} 0 {-math.pi / 2}'})
    sensor = ET.SubElement(gazebo, 'sensor', {'name': 'sim_wrist_depth', 'type': 'depth_camera'})
    ET.SubElement(sensor, 'topic').text = '/camera/camera/depth/image_rect_raw'
    ET.SubElement(sensor, 'gz_frame_id').text = 'camera_depth_optical_frame'
    ET.SubElement(sensor, 'update_rate').text = '15'
    camera = ET.SubElement(sensor, 'camera')
    ET.SubElement(camera, 'camera_info_topic').text = '/camera/camera/depth/camera_info'
    ET.SubElement(camera, 'horizontal_fov').text = '1.51844'
    image = ET.SubElement(camera, 'image')
    for name, value in (('width', '640'), ('height', '480'), ('format', 'R8G8B8')):
        ET.SubElement(image, name).text = value
    clip = ET.SubElement(camera, 'clip')
    ET.SubElement(clip, 'near').text = '0.2'
    ET.SubElement(clip, 'far').text = '10.0'


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
    mobile_base_control = evaluate_condition_expression(context, [LaunchConfiguration('mobile_base_control')])
    lidar = evaluate_condition_expression(context, [LaunchConfiguration('lidar')])
    camera = evaluate_condition_expression(context, [LaunchConfiguration('camera')])
    if camera and gripper != 'soft_two_fingers':
        raise ValueError('Wrist camera simulation requires gripper:=soft_two_fingers')
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
    root = ET.fromstring(xacro.process_file(description_path).toxml())
    for gazebo in root.findall('gazebo'):
        for plugin in gazebo.findall('plugin'):
            if plugin.get('filename') == 'libgazebo_ros_control.so':
                gazebo.remove(plugin)
    gripper_joints = ('gripper_planar_4', 'gripper_planar_5')
    available_joints = {joint.get('name') for joint in root.findall('joint')}
    has_gripper_joints = all(name in available_joints for name in gripper_joints)
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
    if lidar:
        base_drive.add_lidars(root, prefixed_frames=True)
    if camera:
        add_wrist_camera(root)
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
            'gui': LaunchConfiguration('gui'),
            'world': LaunchConfiguration('world'),
            'lidar': LaunchConfiguration('lidar'),
            'render_sensors': LaunchConfiguration('camera'),
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
            nodes.append(Node(package='controller_manager', executable='spawner',
                              arguments=[controller, '--controller-manager-timeout', '120'], output='screen'))
        if has_gripper_joints:
            nodes.append(Node(package='controller_manager', executable='spawner',
                              arguments=['gripper_position_controller', '--controller-manager-timeout', '120'], output='screen'))
    else:
        nodes.append(Node(package='joint_state_publisher', executable='joint_state_publisher',
                          parameters=[{'use_sim_time': True}], output='screen'))
    if mobile_base_control:
        nodes.extend(base_drive.drive_nodes())
    if lidar:
        nodes.extend(base_drive.lidar_nodes())
    if camera:
        nodes.append(Node(package='ros_gz_bridge', executable='parameter_bridge', arguments=[
            '/camera/camera/color/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera/color/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/camera/camera/depth/image_rect_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera/depth/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ], output='screen'))
        nodes.append(Node(package='ros_gz_bridge', executable='parameter_bridge', arguments=[
            '/camera/camera/depth/image_rect_raw/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
        ], remappings=[('/camera/camera/depth/image_rect_raw/points', '/camera/camera/depth/points')],
            parameters=[{'override_frame_id': 'camera_depth_frame'}],
            output='screen'))
    if docking:
        nodes.extend(base_drive.docking_nodes('ld250_tm12x', dock_x, dock_y, dock_yaw))
    if gripper == 'soft_two_fingers' and (fixed_base_control or mobile_base_control):
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
        DeclareLaunchArgument('mobile_base_control', default_value='false', description='Enable experimental mobile base and arm control'),
        DeclareLaunchArgument('lidar', default_value='false', description='Enable experimental LD250 laser scans'),
        DeclareLaunchArgument('camera', default_value='false', description='Enable the soft gripper wrist camera'),
        DeclareLaunchArgument('docking', default_value='false', description='Enable the experimental simulated dock'),
        DeclareLaunchArgument('dock_x', default_value='0.0', description='Docked robot X position in world coordinates'),
        DeclareLaunchArgument('dock_y', default_value='0.0', description='Docked robot Y position in world coordinates'),
        DeclareLaunchArgument('dock_yaw', default_value='0.0', description='Docked robot yaw in radians'),
        DeclareLaunchArgument('x', default_value='0.0', description='Spawn X position in meters'),
        DeclareLaunchArgument('y', default_value='0.0', description='Spawn Y position in meters'),
        DeclareLaunchArgument('z', default_value='0.0', description='Spawn Z position in meters'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='Spawn yaw in radians'),
        DeclareLaunchArgument('gripper', default_value='none', description='none or soft_two_fingers'),
        OpaqueFunction(function=create_preview),
    ])