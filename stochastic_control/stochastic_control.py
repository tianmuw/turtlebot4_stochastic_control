#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
@Author: Tianmu Wang (tiw028@ucsd.edu)
@Description: Designed stochastic control law for motion planning in presence of static obstacles.
"""

import rclpy
import yaml
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import PoseStamped
from marvelmind_ros2_msgs.msg import HedgePosition
from rclpy.qos import qos_profile_sensor_data, qos_profile_services_default
from stochastic_control.nav_action_client import NavActionClientNode
from stochastic_control.utils import *


class TBController(Node):

    def __init__(self, name):
        super().__init__(name)

        # The experiment's parameters are stored in the yaml file
        with open('/stochastic_control/yaml/param.yaml', 'r') as f:
            params = yaml.safe_load(f)

        namespace = params[namespace]
        robot_name = params[parameters][robot_name]
        beacon_type = params[parameters][beacon_type]
        x_init = np.array(params[parameters][x_init])
        x_obs = np.array(params[parameters][x_obs])
        d = params[parameters][d]
        theta = params[parameters][theta]

        # Initialize the pose
        self.current_pose = PoseStamped()
        self.current_pose.header.frame_id = 'marvelmind-073'
        self.current_pose.pose.position.x = x_init[0]
        self.current_pose.pose.position.y = x_init[1]

        # Subscriptions:
        self.sub_marvelmind = self.create_subscription(HedgePosition, '/hedgehog_pos', self.marvelmind_pos_callback,
                                                       qos_profile_sensor_data)

        # Action client
        self.navigate_client_node = NavActionClientNode('navigate_client')
        # Move to the initial pose
        self.navigate_client_node.send_goal(self.current_pose, True)

        self.self.timer = self.create_timer(0.1, self.control_callback)

    def control_callback(self):
        while np.linalg.norm(pose2numpy2D(self.pose_current) - x_goal) > thershold:
            pose_next = stochastic_control(pose2numpy2D(self.pose_current), x_goal, x_obs, d, theta)
            self.current_pose.pose.position.x = pose_next[0]
            self.current_pose.pose.position.y = pose_next[1]
            self.navigate_client_node.send_goal(self.current_pose, True)

    def marvelmind_pos_callback(self, pos_msg):
        """
        Receiving the position messages in the marvelmind frame
        :param pos_msg:
        :return:
        """
        x_current = pos_msg.x_m
        y_current = pos_msg.y_m
        z_current = pos_msg.z_m
        angle_current = pos_msg.angle
        self.get_logger().info(f'Current Position: "({x_current}, {y_current}, {z_current}, {angle_current})"')


def main(args=None):
    rclpy.init(args=args)
    controller = TBController('Controller')
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        rclpy.try_shutdown()
        controller.destroy_node()


if __name__ == "__main__":
    main()

