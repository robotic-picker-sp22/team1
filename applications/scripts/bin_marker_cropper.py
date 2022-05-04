from typing import List

import rosbag
import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers, AlvarMarker

# ! /usr/bin/env python


HALLUCINATE = True


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


class ArTagReader(object):
    def __init__(self):
        self.markers: List[AlvarMarker] = []

    def callback(self, msg):
        self.markers = msg.markers


def main():
    rospy.init_node("hallucinations")
    wait_for_time()

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

    marker = next(filter(lambda m: m.id == 0, reader.markers))  # maybe support cropping multiple markers
    marker_position = marker.pose.pose.position
    rospy.set_param("crop_min_x", marker_position.x)
    rospy.set_param("crop_min_y", marker_position.y - 0.26)
    rospy.set_param("crop_min_z", marker_position.z - 0.15)
    rospy.set_param("crop_max_x", marker_position.x + 0.3)
    rospy.set_param("crop_max_y", marker_position.y - 0.12)
    rospy.set_param("crop_max_z", marker_position.z - 0.02)


if __name__ == '__main__':
    main()
