#! /usr/bin/env python

from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import PoseStamped
import robot_api
import rospy
import rosbag

HALLUCINATE = True


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


class ArTagReader(object):
    def __init__(self):
        self.markers = []

    def callback(self, msg):
        self.markers = msg.markers


def main():
    rospy.init_node("hallucinations")
    wait_for_time()

    start = PoseStamped()
    start.header.frame_id = 'base_link'
    start.pose.position.x = 0.5
    start.pose.position.y = 0.5
    start.pose.position.z = 0.75
    print(start)
    arm = robot_api.Arm()
    arm.move_to_pose(start)

    reader = ArTagReader()
    sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, callback=reader.callback, queue_size=10)

    if HALLUCINATE:
        fake_topic_name = "/ar_pose_marker_fake"
        fake_pub = rospy.Publisher(fake_topic_name, AlvarMarkers, queue_size=10)
        # rospy.sleep(0.2)
        rospy.Subscriber(fake_topic_name, AlvarMarkers, callback=reader.callback, queue_size=10)
        # rospy.sleep(0.2)
        bag = rosbag.Bag("/home/capstone/data/ar_pose_markers_example.bag")
        for _, msg, _ in bag.read_messages(["/ar_pose_marker"]):
            rospy.loginfo("Pub msg")
            fake_pub.publish(msg)

    while len(reader.markers) == 0:
        rospy.sleep(0.1)

    for marker in reader.markers:
        marker.pose.header.frame_id = 'base_link'
        marker.pose.pose.orientation.x = 0
        marker.pose.pose.orientation.y = 0
        marker.pose.pose.orientation.z = 0
        marker.pose.pose.orientation.w = 0
        print(marker)
        error = arm.move_to_pose(marker.pose)
        if error is None:
            rospy.loginfo('Moved to marker {}'.format(marker.id))
            return
        else:
            rospy.logwarn('Failed to move to marker {} with error {}'.format(marker.id, error))
    rospy.logerr('Failed to move to any markers!')


if __name__ == '__main__':
    main()
