import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
from launch import LaunchContext


PACKAGE = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location('onrobot_preview', PACKAGE / 'launch' / 'simulation.launch.py')
preview = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(preview)


def launch_context(gripper, controlled=True, world='empty'):
    context = LaunchContext()
    context.launch_configurations.update({
        'gripper': gripper,
        'world': world,
        'fixed_base_control': str(controlled).lower(),
        'mobile_base_control': 'false',
        'lidar': 'false',
        'docking': 'false',
        'dock_x': '0',
        'dock_y': '0',
        'dock_yaw': '0',
        'mount_xyz': '0.01 0 0',
        'mount_rpy': '0 0 0',
        'tcp_xyz': '0 0 0',
        'tcp_rpy': '0 0 0',
        'x': '0',
        'y': '0',
        'z': '0',
        'yaw': '0',
        'gui': 'false',
        'randomize_boxes': 'false',
        'box_seed': '',
        'randomize_objects': 'false',
        'object_seed': '',
    })
    return context


@pytest.mark.parametrize('gripper, controlled_joint', [
    ('none', None),
    ('rg2', 'finger_joint'),
    ('rg6', 'finger_joint'),
    ('vgc10', None),
])
def test_controlled_description_selects_and_configures_tool(monkeypatch, gripper, controlled_joint):
    monkeypatch.setattr(preview, 'Node', lambda **kwargs: kwargs)
    nodes = preview.create_preview(launch_context(gripper))
    publisher = next(node for node in nodes if isinstance(node, dict) and node['executable'] == 'robot_state_publisher')
    root = ET.fromstring(publisher['parameters'][0]['robot_description'])

    if gripper != 'none':
        assert root.find("joint[@name='onrobot_mount_joint']/origin").get('xyz') == '0.01 0 0'
    assert len(root.findall('ros2_control')) == 1
    control_joints = {joint.get('name') for joint in root.findall('ros2_control/joint')}
    assert all(f'joint_{number}' in control_joints for number in range(1, 7))
    if controlled_joint:
        assert controlled_joint in control_joints
        assert len(root.findall(".//plugin[@filename='libgz_ros2_control-system.so']")) == 1
        action_server = next(node for node in nodes if isinstance(node, dict) and
                             node.get('package') == 'onrobot_control')
        assert action_server['namespace'] == f'onrobot_{gripper}'
    else:
        assert 'finger_joint' not in control_joints


@pytest.mark.parametrize('gripper', ['rg2', 'rg6', 'vgc10'])
def test_static_description_has_no_ros2_control_plugin(monkeypatch, gripper):
    monkeypatch.setattr(preview, 'Node', lambda **kwargs: kwargs)
    nodes = preview.create_preview(launch_context(gripper, controlled=False))
    publisher = next(node for node in nodes if isinstance(node, dict) and node['executable'] == 'robot_state_publisher')
    root = ET.fromstring(publisher['parameters'][0]['robot_description'])
    assert not root.findall('ros2_control')
    assert not root.findall(".//plugin[@filename='libgz_ros2_control-system.so']")


def test_rejects_soft_gripper_from_onrobot_package():
    with pytest.raises(ValueError, match='Unsupported gripper'):
        preview.create_preview(launch_context('soft_two_fingers'))


def test_vgc10_grasp_world_adds_contact_and_detachable_joint_interfaces(monkeypatch):
    monkeypatch.setattr(preview, 'Node', lambda **kwargs: kwargs)
    nodes = preview.create_preview(launch_context('vgc10', world='grasp_test'))
    publisher = next(node for node in nodes if isinstance(node, dict) and node['executable'] == 'robot_state_publisher')
    root = ET.fromstring(publisher['parameters'][0]['robot_description'])
    detachable = root.findall(".//plugin[@filename='gz-sim-detachable-joint-system']")
    assert len(detachable) == 100
    assert {plugin.findtext('child_model') for plugin in detachable} == {
        'sample_box', 'sample_box_1', 'sample_box_2', 'sample_box_4',
        'sample_cylinder', 'sample_cylinder_1', 'sample_cylinder_2', 'sample_cylinder_4',
        'sample_sphere_1', 'sample_sphere_2', 'sample_sphere_3', 'sample_sphere_4', 'sample_sphere_5',
        'sample_block_1', 'sample_block_2', 'sample_block_3', 'sample_block_4', 'sample_block_5',
        'sample_bottle_1', 'sample_bottle_2', 'sample_bottle_3', 'sample_bottle_4', 'sample_bottle_5',
        'sample_box_5', 'sample_cylinder_5',
    }
    for cup in ('suction1', 'suction2', 'suction3', 'suction4'):
        sensor = root.find(f"gazebo[@reference='{cup}']/sensor[@type='contact']")
        assert sensor.findtext('contact/collision') == f'{cup}_collision'
        assert root.find(f"gazebo[@reference='_{cup}_joint']/preserveFixedJoint").text == 'true'
        assert all(plugin.findtext('parent_link') == cup for plugin in detachable
                   if f'/{cup}/' in plugin.findtext('attach_topic'))
    assert root.find("link[@name='suction']/inertial/mass").get('value') == '0.001'

    world_launch = nodes[1]
    assert dict(world_launch.launch_arguments)['contact_sensors'] == 'true'
    simulator = next(node for node in nodes if isinstance(node, dict) and node.get('executable') == 'vgc10_simulator')
    assert len(simulator['parameters'][0]['target_models']) == 25
    assert len(simulator['parameters'][0]['target_keys']) == 100