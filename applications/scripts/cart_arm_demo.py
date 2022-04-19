#! /usr/bin/env python

from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
import rospy
import robot_api


def get_gripper_poses():
    pose1 = Pose(Point(0.042, 0.384, 1.826),
                 Quaternion(0.173, -0.693, -0.242, 0.657))
    pose2 = Pose(Point(0.047, 0.545, 1.822),
                 Quaternion(-0.274, -0.701, 0.173, 0.635))
    ps1 = PoseStamped()
    ps1.header.frame_id = 'base_link'
    ps1.pose = pose1
    ps2 = PoseStamped()
    ps2.header.frame_id = 'base_link'
    ps2.pose = pose2
    gripper_poses = [ps1, ps2]
    return gripper_poses


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


def main():
    # ... init ...
    rospy.init_node('cart_arm_demo')
    wait_for_time()
    arm = robot_api.Arm()

    def shutdown():
        arm.cancel_all_goals()
    rospy.on_shutdown(shutdown)

    # Move the arm
    gripper_poses = get_gripper_poses()
    isdone = False
    while not isdone:
        for pose in gripper_poses:
            error = arm.move_to_pose(pose)
            # DEBUG: There is a spam of CONTROL_FAILED messages
            # They don't seem to be doing anything wrong now, but there is probably something off in the arm
            # Maybe further Labs will solve it.
            if error is not None:
                rospy.logerr(error)
                if error != "CONTROL_FAILED":
                    isdone = True


if __name__ == '__main__':
    main()
