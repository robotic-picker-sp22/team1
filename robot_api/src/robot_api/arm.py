from .utils.moveit import moveit_error_string
import rospy

from .arm_joints import ArmJoints
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from actionlib import SimpleActionClient
from trajectory_msgs.msg import JointTrajectoryPoint
from .moveit_goal_builder import MoveItGoalBuilder    
from moveit_python import MoveGroupInterface            

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
        self.move_group = MoveGroupInterface("arm", "base_link")
        self._move_group_client = self.move_group.get_move_action()

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

    def move_to_pose(self, pose_stamped):
        """Moves the end-effector to a pose, using motion planning.

        Args:
            pose: geometry_msgs/PoseStamped. The goal pose for the gripper.

        Returns:
            string describing the error if an error occurred, else None.
        """
        goal_builder = MoveItGoalBuilder()
        goal_builder.set_pose_goal(pose_stamped)
        goal = goal_builder.build()

        self._move_group_client.send_goal(goal)
        self._move_group_client.wait_for_result(rospy.Duration(10))

        result = self._move_group_client.get_result()
        if result is not None:
            error_string = moveit_error_string(result.error_code.val)
            if error_string == "SUCCESS":
                return None
            return error_string
        else:
            # idk something hardcoded
            return "UNKNOWN_ERROR_CODE"

    def cancel_all_goals(self):
        self.client.cancel_all_goals()  # Your action client from Lab 7
        self._move_group_client.cancel_all_goals()  # From this lab
