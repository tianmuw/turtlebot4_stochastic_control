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
    Pose: geometry_msgs.msg/PoseStamped
    """
    pose_array = np.array([0.0, 0.0])
    pose_array[0] = pose.position.x
    pose_array[1] = pose.position.y
    return pose_array

def stochastic_control(x_t, x_goal, x_obs, d, theta):
        """
        Params:
        x_t: np.array()
            Robot current state
        x_goal: np.array()
            The goal state
        x_obs: np.ndarray()
            The list of obstacles
        d: float
            Distance
        theta: float
            Safe margin
        
        Return:
        x_next: np.array()
            The next state
        """

        d_x = x_goal - x_t
        c = np.sqrt(d_x[0] ** 2 + d_x[1] ** 2)
        r = d/c
        x_new = x_t + r * d_x
        safe = False
        while not safe:
            x = np.random.normal(loc=x_new[0], scale=d/np.sqrt(2))
            y = np.random.normal(loc=x_new[1], scale=d/np.sqrt(2))
            x_next = np.array([0., 0.])
            x_next[0] = x
            x_next[1] = y
            x_ob = np.array([2,2])
            temp = x_next - x_ob
            if np.linalg.norm(temp) < theta:
                safe = False
            safe = True
            for x_ob in x_obs[:, ]:
                temp = x_next - x_ob
                if np.linalg.norm(temp) < theta:
                    safe = False
                    break
                safe = True
        return x_next