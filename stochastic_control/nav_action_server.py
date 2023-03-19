#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
@Author: Tianmu Wang (tiw028@ucsd.edu)
@Description: ROS2 Using the turtlebot4 action NavigateToPosition to navigate the turtlebot4
"""

import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from irobot_create_msgs.action import NavigateToPosition


class NavActionServerNode(Node):

    def __init__(self, name):
        super().__init__(name)
        self._action_server = ActionServer(  # create action server（interface_type, action_name, callback_func）
            self,
            NavigateToPosition,
            'navigate_to_position',
            self.execute_callback)

    def execute_callback(self, goal_handle):  # exert function
        self.get_logger().info('Start action ...')
        feedback_msg = NavigateToPosition.Feedback()  # create a feedback message

        while not goal_handle.succeed():  # When the robot has not moved to the goal pose
            if goal_handle.is_cancel_requested:  # If the request is canceled by the client
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return NavigateToPosition.Result().pose
            self.get_logger().info(
                f'Publishing feedback: current navigate state: {feedback_msg.navigate_state}, \n current remaining angle travel: {feedback_msg.remaining_angle_travel}, \n current remaining travel distance: {feedback_msg.remaining_travel_distance}')
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(0.5)

        result = NavigateToPosition.Result()

        return result


def main(args=None):
    rclpy.init(args=args)
    node = NavActionServerNode('move_turtlebot_server')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()