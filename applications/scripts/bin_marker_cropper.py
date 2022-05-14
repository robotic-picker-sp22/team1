#! /usr/bin/env python

import rospy
import tf2_geometry_msgs
import tf2_ros
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import PoseStamped

CENTER_SMALL = [-0.02, -0.1, -0.03, +0.5, +0.03, +0.1]
UPPER_SMALL = [-0.02, -0.1, 0.07, +0.5, +0.03, +0.2]
BOTTOM_BIG = [-0.02, 0, -0.2, 0.5, -0.2, -0.1]


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


class ArTagReader(object):
    def __init__(self):
        self.markers = []
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(100))  # tf buffer length
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def callback(self, msg):
        self.markers = msg.markers


def bin_marker_crop(reader, bin_offset, marker_id=0):
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        if len(reader.markers) == 0:
            rate.sleep()
            continue
        marker = next(filter(lambda m: m.id == marker_id, reader.markers))  # maybe support cropping multiple markers
        transform = reader.tf_buffer.lookup_transform("base_link", marker.header.frame_id[1:], marker.header.stamp,
                                                      rospy.Duration(1))
        transform_back = reader.tf_buffer.lookup_transform(marker.header.frame_id[1:], "base_link", marker.header.stamp,
                                                           rospy.Duration(1))

        pose_transformed = tf2_geometry_msgs.do_transform_pose(marker.pose, transform)
        position_transformed = pose_transformed.pose.position
        min_pose = PoseStamped()
        min_pose.pose.position.x = position_transformed.x + bin_offset[0]
        min_pose.pose.position.y = position_transformed.y + bin_offset[1]
        min_pose.pose.position.z = position_transformed.z + bin_offset[2]
        max_pose = PoseStamped()
        max_pose.pose.position.x = position_transformed.x + bin_offset[3]
        max_pose.pose.position.y = position_transformed.y + bin_offset[4]
        max_pose.pose.position.z = position_transformed.z + bin_offset[5]

        pose_min_transformed = tf2_geometry_msgs.do_transform_pose(min_pose, transform_back)
        pose_max_transformed = tf2_geometry_msgs.do_transform_pose(max_pose, transform_back)

        rospy.set_param("/crop_min_x", pose_min_transformed.pose.position.x)
        rospy.set_param("/crop_min_y", pose_min_transformed.pose.position.y)
        rospy.set_param("/crop_min_z", pose_min_transformed.pose.position.z)
        rospy.set_param("/crop_max_x", pose_max_transformed.pose.position.x)
        rospy.set_param("/crop_max_y", pose_max_transformed.pose.position.y)
        rospy.set_param("/crop_max_z", pose_max_transformed.pose.position.z)
        rate.sleep()


def main():
    rospy.init_node("bin_marker_cropper")
    wait_for_time()

    reader = ArTagReader()
    rospy.Subscriber("/ar_pose_marker", AlvarMarkers, callback=reader.callback, queue_size=1)

    bin_marker_crop(reader, UPPER_SMALL)


if __name__ == '__main__':
    main()
