#!/usr/bin/env python3
import math
import threading
import time

from control_msgs.action import GripperCommand
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class SimGripperAction(Node):
    def __init__(self):
        super().__init__('gripper_two_fingers_node')
        self.declare_parameter('gripper_open_width', 0.09)
        self.declare_parameter('gripper_closed_width', 0.0)
        self.declare_parameter('gripper_position_tolerance', 0.0005)
        self.declare_parameter('gripper_joint_name_prefix', '')
        self.declare_parameter('goal_timeout', 10.0)
        self.declare_parameter('state_timeout', 2.0)
        self.fingers = []
        prefix = str(self.get_parameter('gripper_joint_name_prefix').value)
        for side, default_joint in (('left', 'gripper_planar_4'), ('right', 'gripper_planar_5')):
            base = f'gripper_{side}_finger'
            self.declare_parameter(f'{base}.joint_name', default_joint)
            self.declare_parameter(f'{base}.joint_open_position', -0.045)
            self.declare_parameter(f'{base}.joint_close_position', 0.0)
            name = str(self.get_parameter(f'{base}.joint_name').value)
            if prefix and not name.startswith(prefix):
                name = prefix + name
            self.fingers.append((name, float(self.get_parameter(f'{base}.joint_open_position').value),
                                 float(self.get_parameter(f'{base}.joint_close_position').value)))
        self.open_width = float(self.get_parameter('gripper_open_width').value)
        self.closed_width = float(self.get_parameter('gripper_closed_width').value)
        if not 0 <= self.closed_width < self.open_width:
            raise ValueError('gripper widths must satisfy 0 <= closed < open')
        self.lock = threading.Lock()
        self.positions = None
        self.state_time = None
        self.goal_active = False
        callbacks = ReentrantCallbackGroup()
        self.publisher = self.create_publisher(Float64MultiArray, '/gripper_position_controller/commands', 10)
        self.subscription = self.create_subscription(JointState, '/joint_states', self.on_state, 10,
                                                     callback_group=callbacks)
        self.server = ActionServer(self, GripperCommand, 'gripper_command', execute_callback=self.execute,
                                   goal_callback=self.accept_goal, cancel_callback=self.cancel_goal,
                                   callback_group=callbacks)

    def on_state(self, message):
        values = dict(zip(message.name, message.position))
        if all(name in values and math.isfinite(values[name]) for name, _, _ in self.fingers):
            with self.lock:
                self.positions = [values[name] for name, _, _ in self.fingers]
                self.state_time = time.monotonic()

    def accept_goal(self, goal):
        position = float(goal.command.position)
        effort = float(goal.command.max_effort)
        if not math.isfinite(position) or not math.isfinite(effort) or effort != 0.0:
            self.get_logger().warning('Simulation supports finite position goals with max_effort=0 only')
            return GoalResponse.REJECT
        with self.lock:
            if self.goal_active:
                return GoalResponse.REJECT
            self.goal_active = True
        return GoalResponse.ACCEPT

    def cancel_goal(self, goal_handle):
        return CancelResponse.ACCEPT

    def current_state(self):
        with self.lock:
            return (None if self.positions is None else list(self.positions), self.state_time)

    def command(self, positions):
        message = Float64MultiArray()
        message.data = positions
        self.publisher.publish(message)

    def width(self, positions):
        ratios = [(position - closed) / (opened - closed)
                  for position, (_, opened, closed) in zip(positions, self.fingers)]
        return self.closed_width + (self.open_width - self.closed_width) * sum(ratios) / len(ratios)

    def execute(self, goal_handle):
        requested = float(goal_handle.request.command.position)
        target_width = requested if requested >= 0 else abs(requested) * len(self.fingers)
        target_width = max(self.closed_width, min(self.open_width, target_width))
        ratio = (target_width - self.closed_width) / (self.open_width - self.closed_width)
        targets = [closed + ratio * (opened - closed) for _, opened, closed in self.fingers]
        tolerance = float(self.get_parameter('gripper_position_tolerance').value)
        deadline = time.monotonic() + float(self.get_parameter('goal_timeout').value)
        state_timeout = float(self.get_parameter('state_timeout').value)
        result = GripperCommand.Result()
        try:
            positions, stamp = self.current_state()
            if positions is None or stamp is None or time.monotonic() - stamp > state_timeout:
                self.get_logger().error('No recent simulated gripper joint states')
                goal_handle.abort()
                return result
            self.command(targets)
            while rclpy.ok():
                positions, stamp = self.current_state()
                if positions is not None:
                    result.position = self.width(positions)
                if goal_handle.is_cancel_requested:
                    if positions is not None:
                        self.command(positions)
                    goal_handle.canceled()
                    return result
                if stamp is None or time.monotonic() - stamp > state_timeout or time.monotonic() >= deadline:
                    if positions is not None:
                        self.command(positions)
                    self.get_logger().error('Simulated gripper did not reach its target or joint states timed out')
                    goal_handle.abort()
                    return result
                feedback = GripperCommand.Feedback()
                feedback.position = result.position
                feedback.reached_goal = all(abs(actual - target) <= tolerance
                                            for actual, target in zip(positions, targets))
                goal_handle.publish_feedback(feedback)
                if feedback.reached_goal:
                    result.reached_goal = True
                    goal_handle.succeed()
                    return result
                time.sleep(0.05)
            goal_handle.abort()
            return result
        finally:
            with self.lock:
                self.goal_active = False


def main():
    rclpy.init()
    node = SimGripperAction()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()