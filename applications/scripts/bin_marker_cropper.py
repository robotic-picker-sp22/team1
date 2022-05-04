#! /usr/bin/env python

import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers

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
        if self.markers:
            marker = next(filter(lambda m: m.id == 0, self.markers))  # maybe support cropping multiple markers
            marker_position = marker.pose.pose.position
            rospy.set_param("crop_min_x", marker_position.x)
            rospy.set_param("crop_min_y", marker_position.y - 0.26)
            rospy.set_param("crop_min_z", marker_position.z - 0.15)
            rospy.set_param("crop_max_x", marker_position.x + 0.3)
            rospy.set_param("crop_max_y", marker_position.y - 0.12)
            rospy.set_param("crop_max_z", marker_position.z - 0.02)


def main():
    rospy.init_node("bin_marker_cropper")
    wait_for_time()

    reader = ArTagReader()
    sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, callback=reader.callback, queue_size=10)


if __name__ == '__main__':
    main()
