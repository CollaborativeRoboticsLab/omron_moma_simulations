import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
import pytest
import yaml


PACKAGE = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location('preview', PACKAGE / 'launch' / 'simulation.launch.py')
preview = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(preview)


def test_wrist_camera_uses_existing_mount_and_optical_frame():
    root = ET.fromstring('<robot name="test"><link name="camera_link"/></robot>')
    preview.add_wrist_camera(root)
    sensor = root.find("gazebo[@reference='camera_link']/sensor[@name='sim_wrist_color']")
    assert sensor.findtext('topic') == '/camera/camera/color/image_raw'
    assert sensor.findtext('gz_frame_id') == 'camera_color_optical_frame'
    assert sensor.findtext('camera/camera_info_topic') == '/camera/camera/color/camera_info'
    assert sensor.findtext('camera/image/width') == '640'
    assert root.find("joint[@name='sim_camera_color_optical_joint']/child").get('link') == 'camera_color_optical_frame'
    depth = root.find("gazebo[@reference='camera_link']/sensor[@name='sim_wrist_depth']")
    assert depth.get('type') == 'depth_camera'
    assert depth.findtext('topic') == '/camera/camera/depth/image_rect_raw'
    assert depth.findtext('gz_frame_id') == 'camera_depth_optical_frame'
    assert depth.findtext('camera/camera_info_topic') == '/camera/camera/depth/camera_info'
    assert root.find("joint[@name='sim_camera_depth_optical_joint']/child").get('link') == 'camera_depth_optical_frame'


def test_wrist_camera_requires_camera_link():
    with pytest.raises(ValueError, match='camera_link'):
        preview.add_wrist_camera(ET.fromstring('<robot name="test"/>'))


@pytest.mark.parametrize('fixed', [True, False])
def test_gripper_control_configuration(monkeypatch, fixed):
    monkeypatch.setattr(preview, 'Node', lambda **kwargs: kwargs)
    context = LaunchContext()
    context.launch_configurations.update({
        'gripper': 'soft_two_fingers', 'world': 'empty',
        'fixed_base_control': str(fixed).lower(), 'mobile_base_control': str(not fixed).lower(),
        'lidar': 'false', 'camera': 'false', 'docking': 'false',
    })
    nodes = preview.create_preview(context)
    assert dict(nodes[1].launch_arguments)['physics_engine'] == ('bullet' if fixed else 'dart')
    publisher = next(node for node in nodes if isinstance(node, dict) and node['executable'] == 'robot_state_publisher')
    root = ET.fromstring(publisher['parameters'][0]['robot_description'])
    for name in ('gripper_planar_4', 'gripper_planar_5'):
        limit = root.find(f"joint[@name='{name}']/limit")
        assert float(limit.get('lower')) == pytest.approx(-0.045 if fixed else -0.046)
        assert float(limit.get('upper')) == pytest.approx(0.0 if fixed else 0.001)
    mimic = root.find("joint[@name='gripper_planar_4']/mimic")
    if fixed:
        assert mimic.get('joint') == 'gripper_planar_5'
    else:
        assert mimic is None
    joint = root.find("ros2_control/joint[@name='gripper_planar_4']")
    assert (joint.find('command_interface') is None) == fixed
    assert joint.find("state_interface[@name='position']") is not None
    gripper_dir = Path(get_package_share_directory('gripper_gazebo')) / 'soft-two-finger-gripper'
    with (gripper_dir / ('gripper_coupled.yaml' if fixed else 'gripper_controllers.yaml')).open() as config_file:
        config = yaml.safe_load(config_file)
    assert config['gripper_position_controller']['ros__parameters']['joints'] == (
        ['gripper_planar_5'] if fixed else ['gripper_planar_4', 'gripper_planar_5'])
    adapter = next(node for node in nodes if isinstance(node, dict) and node['executable'] == 'sim_gripper_action.py')
    assert adapter['package'] == 'gripper_gazebo'