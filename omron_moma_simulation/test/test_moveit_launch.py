import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest


PACKAGE = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location('sim_moveit', PACKAGE / 'launch' / 'moveit.launch.py')
moveit = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(moveit)


@pytest.mark.parametrize('gripper', ['none', 'soft_two_fingers'])
def test_simulation_moveit_controller_and_robot_model(gripper):
    parameters = moveit.build_moveit_parameters(gripper)
    assert parameters['use_sim_time'] is True
    controllers = parameters['moveit_simple_controller_manager']
    assert 'tmr_arm_with_base_controller' in controllers['controller_names']
    assert controllers['tmr_arm_with_base_controller']['action_ns'] == 'follow_joint_trajectory'
    robot = ET.fromstring(parameters['robot_description'])
    semantic = ET.fromstring(parameters['robot_description_semantic'])
    joints = {joint.get('name') for joint in robot.findall('joint')}
    links = {link.get('name') for link in robot.findall('link')}
    assert all(f'joint_{number}' in joints for number in range(1, 7))
    assert not semantic.findall('virtual_joint')
    for group in semantic.findall('group'):
        for chain in group.findall('chain'):
            assert chain.get('base_link') in links
            assert chain.get('tip_link') in links
    if gripper == 'soft_two_fingers':
        chain = semantic.find("group[@name='tmr_arm_with_base']/chain")
        assert chain.get('tip_link') == 'camera_mount_link'
        assert 'gripper_command' in controllers['controller_names']


def test_simulation_moveit_rejects_unknown_gripper():
    with pytest.raises(ValueError, match='Unsupported gripper'):
        moveit.build_moveit_parameters('unknown')