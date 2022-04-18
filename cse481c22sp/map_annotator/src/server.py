from multiprocessing import Lock
import rospy
from pathlib import Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from map_annotator.msg import PoseStampedNamed, PosesStampedList, UserAction
from nav_msgs.srv import GetPlan
from typing import Dict, Optional
import json

class MapAnnotatorServer():
    def __init__(self) -> None:
        self.poses_dict: Dict[str, PoseStamped] = {}
        self.cur_pose: Optional[PoseStamped] = None
        self.pose_lock = Lock()

        self.__SAVE_DIR = Path("tmp_annotator_data/")

        if not self.__SAVE_DIR.exists():
            self.__SAVE_DIR.mkdir(parents=True)

        # Load saved poses
        for path in self.__SAVE_DIR.glob("annotated_pose_*.json"):
            name = path.stem.split("_")[-1]
            with open(path, "r") as f:
                pose_dict = json.load(f)
                pose = self.__dict_to_pose(pose_dict)
                self.poses_dict[name] = pose

        # Subscribe to the current pose
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.callback_cur_pose)

        # Subscribe to user actions
        rospy.Subscriber("/map_annotator/user_actions", UserAction, self.callback_user_action)

        # Publish all of saved poses
        self.poses_list_pub = rospy.Publisher("/map_annotator/poses", PosesStampedList, queue_size=1, latch=True)

        rospy.wait_for_service("/move_base/NavfnROS/make_plan")
        self.__send_goal_request = rospy.ServiceProxy("/move_base/NavfnROS/make_plan", GetPlan)

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
        for k, v in pose_dict.get("header", {}).items():
            setattr(result.header, k, v)
        for k, v in pose_dict.get("position", {}).items():
            setattr(result.pose.position, k, v)
        for k, v in pose_dict.get("orientation", {}).items():
            setattr(result.pose.orientation, k, v)
        return result

    def callback_cur_pose(self, msg: PoseWithCovarianceStamped):
        with self.pose_lock:
            tmp = PoseStamped
            tmp.pose = msg.pose.pose
            tmp.header = msg.header
            self.cur_pose = tmp

    def callback_user_action(self, msg: UserAction):
        if msg.command == msg.CREATE:
            with self.pose_lock:
                self.poses_dict[msg.name] = self.cur_pose
                self.publish_poses_list()
        elif msg.command == msg.DELETE:
            with self.pose_lock:
                old_pose = self.poses_dict.pop(msg.name, None)
                if old_pose is not None:
                    self.publish_poses_list()
        elif msg.command == msg.RENAME:
            with self.pose_lock:
                old_pose = self.poses_dict.pop(msg.name, None)
                if old_pose is not None:
                    self.poses_dict[msg.updated_name] = old_pose
                    self.publish_poses_list()
        elif msg.command == msg.GOTO:
            pass
            # TODO: Figure this out

    def publish_poses_list(self):
        # Redo the cache if it changed
        new_pose_list = PosesStampedList()
        for k, v in self.poses_dict.items():
            new_pose_list.poses.append(PoseStampedNamed(name=k, pose=v))

        self.poses_list_pub.publish(new_pose_list)


if __name__ == '__main__':
    rospy.init_node('map_annotator')
    MapAnnotatorServer()
    rospy.spin()
