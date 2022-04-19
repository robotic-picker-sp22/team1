import rospy
from actionlib import SimpleActionClient
from control_msgs.msg import (FollowJointTrajectoryAction,
                              FollowJointTrajectoryGoal)
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
from moveit_python import MoveGroupInterface
from trajectory_msgs.msg import JointTrajectoryPoint

from .arm_joints import ArmJoints
from .moveit_goal_builder import MoveItGoalBuilder
from .utils.moveit import moveit_error_string

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
        self._compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)

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

    def move_to_pose(self,
                     pose_stamped,
                     allowed_planning_time=10.0,
                     execution_timeout=15.0,
                     group_name='arm',
                     num_planning_attempts=1,
                     plan_only=False,
                     replan=False,
                     replan_attempts=5,
                     tolerance=0.01):
        """Moves the end-effector to a pose, using motion planning.

        Args:
            pose: geometry_msgs/PoseStamped. The goal pose for the gripper.
            allowed_planning_time: float. The maximum duration to wait for a
                planning result, in seconds.
            execution_timeout: float. The maximum duration to wait for
                an arm motion to execute (or for planning to fail completely),
                in seconds.
            group_name: string. Either 'arm' or 'arm_with_torso'.
            num_planning_attempts: int. The number of times to compute the same
                plan. The shortest path is ultimately used. For random
                planners, this can help get shorter, less weird paths.
            plan_only: bool. If True, then this method does not execute the
                plan on the robot. Useful for determining whether this is
                likely to succeed.
            replan: bool. If True, then if an execution fails (while the arm is
                moving), then come up with a new plan and execute it.
            replan_attempts: int. How many times to replan if the execution
                fails.
            tolerance: float. The goal tolerance, in meters.

        Returns:
            string describing the error if an error occurred, else None.
        """
        goal_builder = MoveItGoalBuilder()
        goal_builder.set_pose_goal(pose_stamped)
        goal_builder.allowed_planning_time = allowed_planning_time
        goal_builder.num_planning_attempts = num_planning_attempts
        goal_builder.plan_only = plan_only
        goal_builder.replan = replan
        goal_builder.replan_attempts = replan_attempts
        goal_builder.tolerance = tolerance
        goal = goal_builder.build()

        self._move_group_client.send_goal(goal)
        # Use execution_timeout for wait_for_result()
        self._move_group_client.wait_for_result(
            rospy.Duration(execution_timeout))

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

    def check_pose(self,
                   pose_stamped,
                   allowed_planning_time=10.0,
                   group_name='arm',
                   tolerance=0.01):
        return self.move_to_pose(
            pose_stamped,
            allowed_planning_time=allowed_planning_time,
            group_name=group_name,
            tolerance=tolerance,
            plan_only=True)

    from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest

    def compute_ik(self, pose_stamped, timeout=rospy.Duration(5)):
        """Computes inverse kinematics for the given pose.

        Note: if you are interested in returning the IK solutions, we have
            shown how to access them.

        Args:
            pose_stamped: geometry_msgs/PoseStamped.
            timeout: rospy.Duration. How long to wait before giving up on the
                IK solution.

        Returns: True if the inverse kinematics were found, False otherwise.
        """
        request = GetPositionIKRequest()
        request.ik_request.pose_stamped = pose_stamped
        request.ik_request.group_name = 'arm'
        request.ik_request.timeout = timeout
        response = self._compute_ik(request)
        error_str = moveit_error_string(response.error_code.val)
        success = error_str == 'SUCCESS'
        if not success:
            return False
        joint_state = response.solution.joint_state
        for name, position in zip(joint_state.name, joint_state.position):
            if name in ArmJoints.names():
                rospy.loginfo('{}: {}'.format(name, position))
        return True
