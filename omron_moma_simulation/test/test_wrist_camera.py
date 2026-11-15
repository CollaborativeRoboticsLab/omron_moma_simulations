import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest


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