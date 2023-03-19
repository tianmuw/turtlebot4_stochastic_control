#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
@Author: Tianmu Wang (tiw028@ucsd.edu)
@Description: ROS2 Using the turtlebot4 action NavigateToPosition to navigate the turtlebot4
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from irobot_create_msgs.action import NavigateToPosition


class NavActionClientNode(Node):
    def __init__(self, name):
        super().__init__(name)
        # Create action client (action_type, action_name)
        self._action_client = ActionClient(
            self, NavigateToPosition, 'navigate_to_position')

    def send_goal(self, goal_pose, achieve_goal_heading):
        goal_msg = NavigateToPosition.Goal()
        goal_msg.goal_pose = goal_pose
        goal_msg.achieve_goal_heading = achieve_goal_heading

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected!')
            return

        self.get_logger().info('Goal accepted!')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.pose}')

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Current navigate state: {feedback.navigate_state}, remaining angle travel is {feedback.remaining_angle_travel}, and remaining travel distance is {feedback.remaining_travel_distance}')


def main(args=None):
    rclpy.init(args=args)
    node = NavActionClientNode('move_turtlebot_client')
    pose = PoseStamped()
    pose.pose.position.x = 3.0
    pose.pose.position.y = 3.0
    node.send_goal(pose, True)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

