#!/usr/bin/env python3

import copy
from multiprocessing import Lock
import rospy
from pathlib import Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from map_annotator.msg import PoseNames, UserAction
from nav_msgs.srv import GetPlan
from robot_api import Base
from typing import Dict, Optional
import json
import numpy as np
from tf.transformations import euler_from_quaternion
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from visualization_msgs.msg import Marker



class MapAnnotatorServer():
    def __init__(self) -> None:
        self.poses_dict: Dict[str, InteractiveMarker] = {}
        self.cur_pose: Optional[PoseStamped] = None
        self.pose_lock = Lock()

        self.__SAVE_DIR = Path("tmp_annotator_data/")

        if not self.__SAVE_DIR.exists():
            self.__SAVE_DIR.mkdir(parents=True)

        self._base = Base()

        self.marker_server = InteractiveMarkerServer("/map_annotator/map_poses")

        # Load saved poses
        for path in self.__SAVE_DIR.glob("annotated_pose_*.json"):
            name = path.stem.split("_")[-1]
            with open(path, "r") as f:
                pose_dict = json.load(f)
                pose = self.__dict_to_pose(pose_dict)
                int_marker = self.__pose_to_marker(pose, name)
                self.poses_dict[name] = int_marker
                self.marker_server.insert(int_marker, self.__process_marker_feedback)

        # Subscribe to the current pose
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.callback_cur_pose)

        # Subscribe to user actions
        rospy.Subscriber("/map_annotator/user_actions", UserAction, self.callback_user_action)

        # Publish all of saved poses
        self.poses_list_pub = rospy.Publisher("/map_annotator/pose_names", PoseNames, queue_size=1, latch=True)

        self.__move_to_publisher = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)

        # Publish for the first time
        rospy.sleep(0.5)
        self.publish_poses_list()
        self.marker_server.applyChanges()

    @staticmethod
    def __pose_to_marker(pose: PoseStamped, name: str) -> InteractiveMarker:
        marker = InteractiveMarker()
        marker.header = pose.header
        marker.pose = pose.pose
        marker.name = name
        marker.description = f"Pose {name}"

        cylinder_marker = Marker()
        cylinder_marker.type = Marker.CYLINDER
        cylinder_marker.pose.position.z = 0.3
        cylinder_marker.scale.x = 0.1
        cylinder_marker.scale.y = 0.1
        cylinder_marker.scale.z = 0.1
        cylinder_marker.color.r = 0.0
        cylinder_marker.color.g = 0.0
        cylinder_marker.color.b = 1.0
        cylinder_marker.color.a = 1.0
        arrow_marker = Marker()
        arrow_marker.type = Marker.ARROW
        arrow_marker.pose.position.z = 0.3
        arrow_marker.scale.x = 0.3
        arrow_marker.scale.y = 0.1
        arrow_marker.scale.z = 0.1
        arrow_marker.color.r = 1.0
        arrow_marker.color.g = 0.0
        arrow_marker.color.b = 0.0
        arrow_marker.color.a = 1.0

        # base_marker.header = pose.header
        # base_marker.pose = pose.pose

        # Create a control which is used to represent the pose
        move_control = InteractiveMarkerControl()
        move_control.orientation.w = 1
        move_control.orientation.x = 0
        move_control.orientation.y = 1
        move_control.orientation.z = 0
        move_control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        move_control.always_visible = True
        move_control.markers.append(cylinder_marker)

        rotate_control = InteractiveMarkerControl()
        rotate_control.orientation.w = 1
        rotate_control.orientation.x = 0
        rotate_control.orientation.y = 1
        rotate_control.orientation.z = 0
        rotate_control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        rotate_control.always_visible = True
        rotate_control.markers.append(arrow_marker)

        marker.controls.append(move_control)
        marker.controls.append(rotate_control)

        return marker

    @staticmethod
    def __marker_to_pose(marker: InteractiveMarker) -> PoseStamped:
        pose = PoseStamped()
        pose.header = marker.header
        pose.pose = marker.pose
        return pose

    @staticmethod
    def __pose_to_dict(pose: PoseStamped) -> Dict[str, Dict[str, float]]:
        result = {}
        for slot in pose.__slots__:
            slot_val = getattr(pose, slot)
            if isinstance(slot_val, rospy.AnyMsg) or {"__slots__", "_slot_types"}.issubset(dir(slot_val)):
                result[slot] = __class__.__pose_to_dict(slot_val)
            else:
                result[slot] = slot_val
        return result

    @staticmethod
    def __dict_to_pose(pose_dict: Dict[str, Dict[str, float]]) -> PoseStamped:
        result = PoseStamped()
        result.header.seq = pose_dict["header"]["seq"]
        result.header.stamp.secs = pose_dict["header"]["stamp"]["secs"]
        result.header.stamp.nsecs = pose_dict["header"]["stamp"]["nsecs"]
        result.header.frame_id = pose_dict["header"]["frame_id"]
        for k, v in pose_dict.get("pose", {}).get("position", {}).items():
            setattr(result.pose.position, k, v)
        for k, v in pose_dict.get("pose", {}).get("orientation", {}).items():
            setattr(result.pose.orientation, k, v)
        return result

    def callback_cur_pose(self, msg: PoseWithCovarianceStamped):
        with self.pose_lock:
            tmp = PoseStamped()
            tmp.pose = msg.pose.pose
            tmp.header = msg.header
            self.cur_pose = tmp

    def __create_pose(self, pose_name: str) -> bool:

        with self.pose_lock:
            marker = self.__pose_to_marker(self.cur_pose, pose_name)
            self.poses_dict[pose_name] = marker
            self.marker_server.insert(marker, self.__process_marker_feedback)
            self.marker_server.applyChanges()

        with open(self.__SAVE_DIR / f"annotated_pose_{pose_name}.json", "w") as f:
            json.dump(self.__pose_to_dict(self.__marker_to_pose(self.poses_dict[pose_name])), f)

        self.publish_poses_list()
        return True

    def __delete_pose(self, pose_name: str) -> bool:
        with self.pose_lock:
            marker = self.poses_dict.pop(pose_name, None)
            if marker is not None:
                self.marker_server.erase(marker.name)
                self.marker_server.applyChanges()
                pose_path = self.__SAVE_DIR / f"annotated_pose_{pose_name}.json"
                if pose_path.exists():
                    pose_path.unlink()
            else:
                return False
        return True

    def __move_to_pose(self, goal_pose: PoseStamped):
        self.__move_to_publisher.publish(goal_pose)

    def callback_user_action(self, msg: UserAction):
        if msg.command == msg.CREATE:
            if self.__create_pose(msg.name):
                self.publish_poses_list()
        elif msg.command == msg.DELETE:
            if self.__delete_pose(msg.name):
                self.publish_poses_list()
        elif msg.command == msg.RENAME:
            with self.pose_lock:
                old_pose = self.poses_dict.pop(msg.name, None)
                if old_pose is not None:
                    self.poses_dict[msg.updated_name] = old_pose
                    self.publish_poses_list()
        elif msg.command == msg.GOTO:
            if msg.name in self.poses_dict:
                self.__move_to_pose(self.__marker_to_pose(self.poses_dict[msg.name]))

    def publish_poses_list(self):
        # Redo the cache if it changed
        new_pose_list = PoseNames()
        new_pose_list.names = list(self.poses_dict.keys())
        self.poses_list_pub.publish(new_pose_list)

    def __process_marker_feedback(self, feedback: InteractiveMarkerFeedback):
        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            pose_name = feedback.marker_name
            if pose_name in self.poses_dict:
                self.poses_dict[pose_name].pose = feedback.pose
                with open(self.__SAVE_DIR / f"annotated_pose_{pose_name}.json", "w") as f:
                    json.dump(self.__pose_to_dict(self.__marker_to_pose(self.poses_dict[pose_name])), f)
                self.publish_poses_list()

if __name__ == '__main__':
    rospy.init_node('map_annotator')
    MapAnnotatorServer()
    rospy.spin()
