#!/usr/bin/env python

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

ACTION_NAME = "/torso_controller/follow_joint_trajectory"
JOINT_NAME = "torso_lift_joint"
TIME_FROM_START = 5  # How many seconds it should take to set the torso height.


class Torso(object):
    """Torso controls the robot's torso height.
    """
    MIN_HEIGHT = 0.0
    MAX_HEIGHT = 0.4

    def __init__(self):
        self.client = actionlib.SimpleActionClient(
            ACTION_NAME, FollowJointTrajectoryAction
        )
        self.client.wait_for_server()
        rospy.loginfo("torso client initialized")

    def set_height(self, height):
        """Sets the torso height.

        This will always take ~5 seconds to execute.

        Args:
            height: The height, in meters, to set the torso to. Values range
                from Torso.MIN_HEIGHT (0.0) to Torso.MAX_HEIGHT(0.4).
        """
        height = max(self.__class__.MIN_HEIGHT, min(height, self.__class__.MAX_HEIGHT))

        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names.append(JOINT_NAME)
        goal.trajectory.points.append(
            JointTrajectoryPoint(positions=[height], time_from_start=rospy.Duration(TIME_FROM_START))
        )

        self.client.send_goal_and_wait(goal)
        return self.client.get_result()
