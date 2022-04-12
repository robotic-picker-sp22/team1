#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from joint_state_reader import JointStateReader


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


def main():
    rospy.init_node('joint_state_republisher')
    wait_for_time()
    rospy.sleep(0.5)
    reader = JointStateReader()

    # Get the joint names from the server
    while True:
        joints = list(reader.latest_joint_states.keys())
        rospy.loginfo("Waiting for joints")
        if len(joints) > 0:
            break
        rospy.sleep(0.5)


    # Create a publisher for each joint
    pubs = {
        joint: rospy.Publisher(f"joint_state_republisher/{joint}", Float64, queue_size=10)
        for joint in joints
    }
    rospy.sleep(0.5)

    # Publish the joint states @ 10Hz
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        for joint, pub in pubs.items():
            msg = Float64()
            msg.data = reader.get_joint(joint)
            pub.publish(msg)
        rate.sleep()


if __name__ == '__main__':
    main()