import rospy

from .arm_joints import ArmJoints
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from actionlib import SimpleActionClient
from trajectory_msgs.msg import JointTrajectoryPoint

TOPIC = "/arm_controller/follow_joint_trajectory"
TRAJECTORY_TIME_LIMIT = 5.0

class Arm(object):
    """Arm controls the robot's arm.

    Joint space control:
        joints = ArmJoints()
        # Fill out joint states
        arm = robot_api.Arm()
        arm.move_to_joints(joints)
    """

    def __init__(self):
        self.client = SimpleActionClient(TOPIC, FollowJointTrajectoryAction)
        self.client.wait_for_server()

    def move_to_joints(self, arm_joints: ArmJoints):
        """Moves the robot's arm to the given joints.

        Args:
            arm_joints: An ArmJoints object that specifies the joint values for
                the arm.
        """
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = arm_joints.names()
        goal.trajectory.points.append(JointTrajectoryPoint(
            positions=arm_joints.values(),
            time_from_start=rospy.Duration(TRAJECTORY_TIME_LIMIT)
        ))

        self.client.send_goal_and_wait(goal)
        return self.client.get_result()
