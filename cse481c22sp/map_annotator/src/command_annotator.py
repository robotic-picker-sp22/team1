import copy
from pathlib import Path
from typing import Dict, Optional
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav_msgs.srv import GetPlan, GetPlanRequest
from multiprocessing import Lock
from robot_api import Base
import numpy as np
from tf.transformations import euler_from_quaternion
import json
import textwrap


class CommandAnnotator():
    def __init__(self) -> None:
        self.command_dict = {
            "list": self.callback_list,
            "save": self.callback_save,
            "delete": self.callback_delete,
            "help": self.callback_help,
            "goto": self.callback_goto,
        }
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
                pose = copy.deepcopy(self.__dict_to_pose(pose_dict))
                self.poses_dict[name] = pose

        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.rospy_callback_odom)

        self._base = Base()

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
        for k, v in pose_dict.get("pose", {}).get("position", {}).items():
            setattr(result.pose.position, k, v)
        for k, v in pose_dict.get("pose", {}).get("orientation", {}).items():
            setattr(result.pose.orientation, k, v)
        return result

    def rospy_callback_odom(self, msg: PoseWithCovarianceStamped):
        with self.pose_lock:
            tmp = PoseStamped()
            tmp.pose = msg.pose.pose
            tmp.header = msg.header
            self.cur_pose = tmp

    def callback_list(self, verbose: bool = False):
        if len(self.poses_dict) == 0:
            print("No poses saved.")
            return
        print("Saved poses:")
        for key, val in self.poses_dict.items():
            print("\t{}".format(key))
            if bool(verbose):
                print("\t\t[{x:.4g}, {y:.4g}, {theta:.4g}]".format(
                    x=val.pose.position.x,
                    y=val.pose.position.y,
                    theta=euler_from_quaternion(
                        [getattr(val.pose.orientation, x) for x in ["x", "y", "z", "w"]]
                    )[-1],
                ))
        print("Current Pose")
        if verbose:
            print("\t[{x:.4g}, {y:.4g}, {theta:.4g}]".format(
                x=self.cur_pose.pose.position.x,
                y=self.cur_pose.pose.position.y,
                theta=euler_from_quaternion(
                    [getattr(self.cur_pose.pose.orientation, x) for x in ["x", "y", "z", "w"]]
                )[-1],
            ))

    def callback_save(self, name: str):
        if self.cur_pose is not None:
            with self.pose_lock:
                self.poses_dict[name] = copy.deepcopy(self.cur_pose)
                # Save pose
                cur_pose_dict = self.__pose_to_dict(self.cur_pose)
                with open(Path(self.__SAVE_DIR) / f"annotated_pose_{name}.json", "w") as f:
                    json.dump(cur_pose_dict, f)
        else:
            print("No pose to save. Is the robot running?")

    def callback_delete(self, name: str):
        if self.poses_dict.pop(name, None) is None:
            print("No pose named {} to delete.".format(name))
        else:
            pose_path = Path(self.__SAVE_DIR) / f"annotated_pose_{name}.json"
            if pose_path.exists():
                pose_path.unlink()

    def callback_help(self):
        print(textwrap.dedent(
            """
            Commands:
             - list: List saved poses.
             - save <name>: Save the robot's current pose as <name>. Overwrites if <name> already exists.
             - delete <name>: Delete the pose given by <name>.
             - goto <name>: Sends the robot to the pose given by <name>.
             - help: Show this list of commands
             - quit: Exit the program"""
        ))

    def callback_goto(self, name: str):
        if name in self.poses_dict:
            goal_pose = self.poses_dict[name]
            cur_pose = self.cur_pose

            # Convert to euler angles
            goal_yaw = euler_from_quaternion([getattr(goal_pose.pose.orientation, x) for x in ["x", "y", "z", "w"]])[-1]
            cur_yaw = euler_from_quaternion([getattr(cur_pose.pose.orientation, x) for x in ["x", "y", "z", "w"]])[-1]

            # First calculate the distance. If we are close enough, we can just stay in place.
            distance = np.sqrt((goal_pose.pose.position.x - cur_pose.pose.position.x) ** 2 + (goal_pose.pose.position.y - cur_pose.pose.position.y) ** 2)
            if distance < 0.1 and abs(goal_yaw - cur_yaw) < 0.05:
                print("Already at goal pose.")
                return

            # goal_yaw = euler_from_quaternion([getattr(goal_pose.pose.orientation, x) for x in ["x", "y", "z", "w"]])[-1]
            # cur_yaw = R([getattr(cur_pose.pose.orientation, x) for x in ["x", "y", "z", "w"]]).as_euler("zyx")[0]

            # Figure out the angle between current and the goal pose
            go_to_goal_angle = np.arctan2(goal_pose.pose.position.y - cur_pose.pose.position.y, goal_pose.pose.position.x - cur_pose.pose.position.x)

            # Rotate robot the the go_to_goal_angle
            self._base.turn(go_to_goal_angle - cur_yaw)

            # TODO: There is a MASSIVE drift. It might be best to move in small chunks, and every 0.5m adjust the angle.
            # Go forward the distance between the current and the goal pose
            self._base.go_forward(distance)

            # Rotate robot to the goal pose
            after_move_yaw = euler_from_quaternion([getattr(self.cur_pose.pose.orientation, x) for x in ["x", "y", "z", "w"]])[-1]
            self._base.turn(goal_yaw - after_move_yaw)
        else:
            print("No pose named {} to goto.".format(name))

    def spin(self):
        while True:
            args = input("> ")
            command = args.split(" ")[0]

            if command == "quit":
                break

            if command in self.command_dict:
                # try:
                if len(args.split(" ")) > 1:
                    self.command_dict[command](args.split(" ", 1)[1])
                else:
                    self.command_dict[command]()

    def run(self) -> None:
        print(
            """Welcome to the map annotator!"""
        )
        self.callback_help()
        self.spin()

if __name__ == "__main__":
    rospy.init_node('keyboard_map_annotator', anonymous=True)
    CommandAnnotator().run()
