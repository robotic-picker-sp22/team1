#! /usr/bin/env python
"""
This file uses given pointcloud topic and the segmentation pipeline to publish a new topic which is the original PC minus all of detected objects.
"""
import rospy
from perception_msgs.msg import PCLIndices
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import numpy as np
from collections import deque
from multiprocessing import Lock

def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


class PCFilterSegmentation():
    __MAX_QUEUE_SIZE = 5

    def __init__(self, input_pc_topic, input_idx_topic, output_pc_topic) -> None:
        self.__pub = rospy.Publisher(output_pc_topic, PointCloud2)

        self.__pcl_lock = Lock()
        self.__last_pcl_msgs = {}
        self.__last_pcl_queue = deque([], self.__MAX_QUEUE_SIZE)

        self.__idx_lock = Lock()
        self.__any_idx_msg_received = False
        self.__last_idx_msgs = {}
        self.__last_idx_queue = deque([], self.__MAX_QUEUE_SIZE)

        rospy.logwarn(
            f"Starting PC Filter Segmentation with topics:\n\t- {input_idx_topic}\n\t- {input_pc_topic}"
        )

        rospy.Subscriber(input_idx_topic, PCLIndices, self.callback_idx)
        rospy.Subscriber(input_pc_topic, PointCloud2, self.callback_pc)

    def callback_pc(self, msg):
        msg: PointCloud2

        with self.__pcl_lock:
            # Delete old key if applicable
            if self.__last_pcl_queue.maxlen == len(self.__last_pcl_queue):
                old_key = self.__last_pcl_queue.popleft()
                del self.__last_pcl_msgs[old_key]

            # Add to queue
            key = msg.header.stamp
            self.__last_pcl_queue.append(key)
            self.__last_pcl_msgs[key] = msg

        # Attempt to publish a message
        self.__attempt_publish(msg.header)

    def callback_idx(self, msg):
        self.__any_idx_msg_received = True
        msg: PointCloud2

        with self.__idx_lock:
            # Delete old key if applicable
            if self.__last_idx_queue.maxlen == len(self.__last_idx_queue):
                old_key = self.__last_idx_queue.popleft()
                del self.__last_idx_msgs[old_key]

            # Add to queue
            key = msg.header.stamp
            self.__last_idx_queue.append(key)
            self.__last_idx_msgs[key] = msg

        # Attempt to publish a message
        self.__attempt_publish(msg.header)

    def __attempt_publish(self, header):
        key = header.stamp

        with self.__pcl_lock:
            with self.__idx_lock:

                if key not in self.__last_idx_msgs or key not in self.__last_pcl_msgs:
                    if key not in self.__last_idx_msgs:
                        rospy.logwarn_throttle(3, "Missing idx message")
                    if key not in self.__last_pcl_msgs:
                        rospy.logwarn_throttle(3, "Missing pcl message")
                    # Header not found in both messages. Return
                    # But receiving idx msgs
                    if self.__any_idx_msg_received:
                        rospy.logwarn_throttle(3, "Returning")
                        return

                # Get original messages
                pcl_msg = self.__last_pcl_msgs[key]
                if self.__any_idx_msg_received:
                    idx_msg = self.__last_idx_msgs[key]
                else:
                    idx_msg = PCLIndices()

        # Convert to pc2 object
        pc_array = np.array(list(pc2.read_points(pcl_msg)))

        # Get a filter array; true if not detected as object. false otherwise
        bool_arr = np.ones(len(pc_array), dtype=bool)
        bool_arr[idx_msg.indices] = False

        # Filter pc_array
        pc_array = pc_array[bool_arr]

        new_pc_msg = pc2.create_cloud(
            header=header,
            fields=pcl_msg.fields,
            points=pc_array
        )

        self.__pub.publish(new_pc_msg)


def main():
    rospy.init_node('segmentation_filtered_pcl')
    wait_for_time()

    pc_topic = rospy.get_param("~input_pc_topic", "/head_camera/depth_registered/points")
    idx_topic = rospy.get_param("~input_idx_topic", "/segment_indices")
    pc_out_topic = rospy.get_param("~output_pc_topic", "/segment_filter_cloud")

    PCFilterSegmentation(pc_topic, idx_topic, pc_out_topic)

    rospy.spin()

if __name__ == "__main__":
    main()
