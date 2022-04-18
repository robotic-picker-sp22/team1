from pathlib import Path
from typing import Dict, Optional
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav_msgs.srv import GetPlan, GetPlanRequest
from multiprocessing import Lock
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
                pose = self.__dict_to_pose(pose_dict)
                self.poses_dict[name] = pose

        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.rospy_callback_odom)

        rospy.wait_for_service("/move_base/NavfnROS/make_plan")
        self.__send_goal_request = rospy.ServiceProxy("/move_base/NavfnROS/make_plan", GetPlan)
        self.simple_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)

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

    def rospy_callback_odom(self, msg: PoseWithCovarianceStamped):
        with self.pose_lock:
            tmp = PoseStamped
            tmp.pose = msg.pose.pose
            tmp.header = msg.header
            self.cur_pose = tmp

    def callback_list(self):
        if len(self.poses_dict) == 0:
            print("No poses saved.")
            return
        print("Saved poses:")
        for key in self.poses_dict:
            print("\t{}".format(key))

    def callback_save(self, name: str):
        if self.cur_pose is not None:
            with self.pose_lock:
                self.poses_dict[name] = self.cur_pose
                # Save pose
                cur_pose_dict = self.__pose_to_dict(self.cur_pose)
                with open(Path(self.__SAVE_DIR) / f"annotated_pose_{name}.json", "w") as f:
                    json.dump(cur_pose_dict, f)
        else:
            print("No pose to save. Is the robot running?")

    def callback_delete(self, name: str):
        if self.poses_dict.pop(name, None) is None:
            print("No pose named {} to delete.".format(name))

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
            print("Sending robot to {}".format(name))
            req = GetPlanRequest(start=self.cur_pose, goal=self.poses_dict[name])
            self.__send_goal_request(req)
            # self.simple_pub.publish(self.poses_dict[name])
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
