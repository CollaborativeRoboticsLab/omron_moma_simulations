import importlib.util
from pathlib import Path
from types import SimpleNamespace

import rclpy
from rclpy.parameter import Parameter

PACKAGE = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location('vgc10_simulator', PACKAGE / 'scripts' / 'vgc10_simulator.py')
simulator = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(simulator)


def test_contact_resolves_supported_model_from_scoped_collision_names():
    contact = SimpleNamespace(
        collision1=SimpleNamespace(name='ld250_tm12x::suction1::suction1_collision'),
        collision2=SimpleNamespace(name='world::sample_box::link::collision'),
    )
    assert simulator.model_from_contact(contact, {'sample_box', 'sample_sphere'}) == 'sample_box'


def test_contact_ignores_objects_without_detachable_joint_targets():
    contact = SimpleNamespace(
        collision1=SimpleNamespace(name='ld250_tm12x::suction1::collision'),
        collision2=SimpleNamespace(name='world::unlisted_object::link::collision'),
    )
    assert simulator.model_from_contact(contact, {'sample_box'}) is None


def test_node_accepts_per_cup_target_parameters():
    target_models = ['sample_box']
    cups = ('suction1', 'suction2', 'suction3', 'suction4')
    target_keys = [f'{cup}::sample_box' for cup in cups]
    overrides = [
        Parameter('target_models', value=target_models),
        Parameter('target_keys', value=target_keys),
        Parameter('contact_topics', value=[f'/vgc10/contacts/{cup}' for cup in cups]),
        Parameter('attach_topics', value=[f'/vgc10/attach/{key.replace("::", "/")}' for key in target_keys]),
        Parameter('detach_topics', value=[f'/vgc10/detach/{key.replace("::", "/")}' for key in target_keys]),
        Parameter('state_topics', value=[f'/vgc10/state/{key.replace("::", "/")}' for key in target_keys]),
    ]
    rclpy.init()
    node = None
    try:
        node = simulator.Vgc10Simulator(parameter_overrides=overrides)
        assert set(node.attach_publishers) == set(target_keys)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()