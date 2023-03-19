#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
@Author: Tianmu Wang (tiw028@ucsd.edu)
@Description
"""

import numpy as np


def pose2numpy2D(pose):
    """
    Convert the pose to numpy array
    :param
    Pose: geometry_msgs.msg/PoseStamped
    """
    pose_array = np.array([0.0, 0.0])
    pose_array[0] = pose.position.x
    pose_array[1] = pose.position.y
    return pose_array


def stochastic_control(x_t, x_goal, x_obs, d, theta):
    """
    Stochastic control law for robot motion
    :param x_t: np.array()
        Robot current states
    :param x_goal: np.array()
        The goal states
    :param x_obs: np.ndarray()
        The obstacles and their states
    :param d: float
        Moving distance
    :param theta: float
        Safe margin
    :return: x_next: np.array()
        The next states
    """

    d_x = x_goal - x_t
    c = np.sqrt(d_x[0] ** 2 + d_x[1] ** 2)
    r = d / c
    x_new = x_t + r * d_x
    x_next = np.array([0., 0.])
    safe = False
    while not safe:
        x = np.random.normal(loc=x_new[0], scale=d / np.sqrt(2))
        y = np.random.normal(loc=x_new[1], scale=d / np.sqrt(2))
        x_next[0] = x
        x_next[1] = y
        for x_ob in x_obs[:, ]:
            if np.linalg.norm(x_next - x_ob) < theta:
                safe = False
                break
            else:
                safe = True
    return x_next
