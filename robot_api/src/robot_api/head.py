#!/usr/bin/env python

import math
from typing import Optional

import rospy
import actionlib
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
    PointHeadAction,
    PointHeadGoal,
)
from trajectory_msgs.msg import JointTrajectoryPoint

# Source: http://docs.fetchrobotics.com/api_overview.html#head-interface
LOOK_AT_ACTION_NAME = 'head_controller/point_head'  # Get the name of the look-at action
PAN_TILT_ACTION_NAME = 'head_controller/follow_joint_trajectory'  # Get the name of the pan/tilt action

# Source: http://docs.fetchrobotics.com/robot_hardware.html#joint-limits-and-types
PAN_JOINT = 'head_pan_joint'  # name of the head pan joint
TILT_JOINT = 'head_tilt_joint'  # name of the head tilt joint

PAN_TILT_TIME = 2.5  # How many seconds it should take to move the head.


class Head(object):
    """Head controls the Fetch's head.

    It provides two interfaces:
        head.look_at(frame_id, x, y, z)
        head.pan_tilt(pan, tilt) # In radians

    For example:
        head = robot_api.Head()
        head.look_at('base_link', 1, 0, 0.3)
        head.pan_tilt(0, math.pi/4)
    """
    MIN_PAN = -math.pi / 2  # Minimum pan angle, in radians.
    MAX_PAN = math.pi / 2  # Maximum pan angle, in radians.
    MIN_TILT = -math.pi / 4  # Minimum tilt angle, in radians.
    MAX_TILT = math.pi / 2  # Maximum tilt angle, in radians.
    LOOK_AT_MIN_DURATION = 1.0  # Minimum time to spend in look_at action, in seconds.

    def __init__(self):
        self.look_at_client = actionlib.SimpleActionClient(
            LOOK_AT_ACTION_NAME, PointHeadAction
        )
        self.pan_tilt_client = actionlib.SimpleActionClient(
            PAN_TILT_ACTION_NAME, FollowJointTrajectoryAction
        )

        self.look_at_client.wait_for_server()
        self.pan_tilt_client.wait_for_server()

        self._last_pan = None
        self._last_tilt = None

    def look_at(self, frame_id, x, y, z):
        """Moves the head to look at a point in space.

        Args:
            frame_id: The name of the frame in which x, y, and z are specified.
            x: The x value of the point to look at.
            y: The y value of the point to look at.
            z: The z value of the point to look at.
        """
        goal = PointHeadGoal()
        goal.target.header.frame_id = frame_id
        goal.target.point.x = x
        goal.target.point.y = y
        goal.target.point.z = z
        goal.min_duration = rospy.Duration(self.LOOK_AT_MIN_DURATION)

        self.look_at_client.send_goal_and_wait(goal)
        return self.look_at_client.get_result()

    def pan_tilt(self, pan: Optional[float] = None, tilt: Optional[float] = None):
        """Moves the head by setting pan/tilt angles.

              Args:
            pan: The pan angle, in radians. A positive value is clockwise.
            tilt: The tilt angle, in radians. A positive value is downwards.
        """
        pan = pan if pan is not None else self._last_pan
        tilt = tilt if tilt is not None else self._last_tilt

        self._last_pan = pan
        self._last_tilt = tilt

        if pan is None or tilt is None:
            rospy.logwarn("Pan or tilt is None.")
            return

        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = [PAN_JOINT, TILT_JOINT]
        goal.trajectory.points.append(JointTrajectoryPoint(
            positions=[
                max(self.__class__.MIN_PAN, min(pan, self.__class__.MAX_PAN)),
                max(self.__class__.MIN_TILT, min(tilt, self.__class__.MAX_PAN)),
            ],
            time_from_start=rospy.Duration(PAN_TILT_TIME)
        ))

        self.pan_tilt_client.send_goal_and_wait(goal)
        return self.pan_tilt_client.get_result()
