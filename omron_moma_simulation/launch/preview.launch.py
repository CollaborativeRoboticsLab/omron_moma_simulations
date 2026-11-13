import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


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
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(preview),
        launch_arguments={
            'model': 'ld250_tm12x',
            'gui': LaunchConfiguration('gui'),
            'world': LaunchConfiguration('world'),
            'randomize_boxes': LaunchConfiguration('randomize_boxes'),
            'box_seed': LaunchConfiguration('box_seed'),
            'randomize_objects': LaunchConfiguration('randomize_objects'),
            'object_seed': LaunchConfiguration('object_seed'),
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'z': LaunchConfiguration('z'),
            'yaw': LaunchConfiguration('yaw'),
            'description_package': package,
            'description_file': description_file,
        }.items(),
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', description='Start the Gazebo graphical client'),
        DeclareLaunchArgument('world', default_value='empty', description='empty, grasp_test, or warehouse'),
        DeclareLaunchArgument('randomize_boxes', default_value='false', description='Randomize warehouse box positions'),
        DeclareLaunchArgument('box_seed', default_value='', description='Optional seed for warehouse box positions'),
        DeclareLaunchArgument('randomize_objects', default_value='false', description='Randomize grasp-test object positions'),
        DeclareLaunchArgument('object_seed', default_value='', description='Optional seed for grasp-test object positions'),
        DeclareLaunchArgument('x', default_value='0.0', description='Spawn X position in meters'),
        DeclareLaunchArgument('y', default_value='0.0', description='Spawn Y position in meters'),
        DeclareLaunchArgument('z', default_value='0.0', description='Spawn Z position in meters'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='Spawn yaw in radians'),
        DeclareLaunchArgument('gripper', default_value='none', description='none or soft_two_fingers'),
        OpaqueFunction(function=create_preview),
    ])