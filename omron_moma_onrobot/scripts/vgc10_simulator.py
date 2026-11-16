#!/usr/bin/env python3

import asyncio
import time

import rclpy
from onrobot_msgs.action import VGCommand
from rclpy.action import ActionServer, GoalResponse
from rclpy.node import Node
from rclpy.parameter import Parameter
from ros_gz_interfaces.msg import Contacts
from std_msgs.msg import Bool, Empty


def model_from_contact(contact, target_models):
    for collision in (contact.collision1, contact.collision2):
        for component in collision.name.split('::'):
            if component in target_models:
                return component
    return None


class Vgc10Simulator(Node):
    def __init__(self, *, parameter_overrides=None):
        super().__init__('vgc10_simulator', parameter_overrides=parameter_overrides)
        target_models = self.declare_parameter('target_models', Parameter.Type.STRING_ARRAY).value
        self.target_models = set(target_models)
        target_keys = self.declare_parameter('target_keys', Parameter.Type.STRING_ARRAY).value
        contact_topics = self.declare_parameter('contact_topics', Parameter.Type.STRING_ARRAY).value
        attach_topics = self.declare_parameter('attach_topics', Parameter.Type.STRING_ARRAY).value
        detach_topics = self.declare_parameter('detach_topics', Parameter.Type.STRING_ARRAY).value
        state_topics = self.declare_parameter('state_topics', Parameter.Type.STRING_ARRAY).value
        if not (len(attach_topics) == len(detach_topics) == len(state_topics) == len(target_keys)):
            raise ValueError('VGC10 target keys and topic lists must have equal lengths')

        self.attach_publishers = {
            key: self.create_publisher(Empty, topic, 10)
            for key, topic in zip(target_keys, attach_topics)
        }
        self.detach_publishers = {
            key: self.create_publisher(Empty, topic, 10)
            for key, topic in zip(target_keys, detach_topics)
        }
        self.states = {key: False for key in target_keys}
        self.state_subscriptions = [
            self.create_subscription(Bool, topic, self._state_callback(key), 10)
            for key, topic in zip(target_keys, state_topics)
        ]
        self.contacts = {}
        self.contact_subscriptions = [
            self.create_subscription(
                Contacts, topic, lambda message, name=topic: self._contact_callback(name, message), 10)
            for topic in contact_topics
        ]
        self.attached_target = None
        self.action_server = ActionServer(
            self, VGCommand, 'onrobot_vg/command', self._execute, goal_callback=self._goal)

    def _state_callback(self, model):
        def update(message):
            self.states[model] = message.data
            if not message.data and self.attached_target == model:
                self.attached_target = None
        return update

    def _contact_callback(self, topic, message):
        model = next((found for contact in message.contacts
                      if (found := model_from_contact(contact, self.target_models))), None)
        if model:
            cup = topic.rsplit('/', 1)[-1]
            self.contacts[topic] = (f'{cup}::{model}', time.monotonic())
        else:
            self.contacts.pop(topic, None)

    def _contacting_target(self):
        recent = [(target, stamp) for target, stamp in self.contacts.values()
                  if time.monotonic() - stamp <= 0.5]
        return max(recent, key=lambda item: item[1])[0] if recent else None

    @staticmethod
    def _requests_vacuum(goal):
        return ((goal.mode_a == VGCommand.Goal.GRIP and goal.target_vacuum_a_percent > 0) or
                (goal.mode_b == VGCommand.Goal.GRIP and goal.target_vacuum_b_percent > 0))

    def _goal(self, goal):
        if (goal.mode_a > VGCommand.Goal.IDLE or goal.mode_b > VGCommand.Goal.IDLE or
                goal.target_vacuum_a_percent > 80 or goal.target_vacuum_b_percent > 80):
            return GoalResponse.REJECT
        if self._requests_vacuum(goal) and self._contacting_target() is None:
            self.get_logger().warning('Rejecting vacuum goal: no supported object is in cup contact')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    async def _wait_for_state(self, model, expected):
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            if self.states.get(model) is expected:
                return True
            await asyncio.sleep(0.05)
        return False

    async def _execute(self, goal_handle):
        goal = goal_handle.request
        result = VGCommand.Result()
        feedback = VGCommand.Feedback()
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            return result

        if self._requests_vacuum(goal):
            target = self._contacting_target()
            if target is None:
                goal_handle.abort()
                return result
            self.attach_publishers[target].publish(Empty())
            if not await self._wait_for_state(target, True):
                goal_handle.abort()
                return result
            self.attached_target = target
        elif goal.mode_a == VGCommand.Goal.RELEASE or goal.mode_b == VGCommand.Goal.RELEASE:
            target = self.attached_target
            if target is not None:
                self.detach_publishers[target].publish(Empty())
                if not await self._wait_for_state(target, False):
                    goal_handle.abort()
                    return result
                self.attached_target = None

        measured = 800 if self.attached_target else 0
        result.measured_vacuum_a_milli_relative = measured
        result.measured_vacuum_b_milli_relative = measured
        feedback.measured_vacuum_a_milli_relative = measured
        feedback.measured_vacuum_b_milli_relative = measured
        goal_handle.publish_feedback(feedback)
        goal_handle.succeed()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = Vgc10Simulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()