from pathlib import Path
from typing import Dict, Optional
import rospy
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from multiprocessing import Lock
import json


class CommandAnnotator():
    def __init__(self) -> None:
        self.command_dict = {
            "list": self.callback_list,
            "save": self.callback_save,
            "delete": self.callback_delete,
            "help": self.callback_help,
            "goto": self.callback_goto,
        }
        self.poses_dict: Dict[str, Pose] = {}
        self.cur_pose: Optional[Pose] = None
        self.pose_lock = Lock()

        self.__SAVE_DIR = Path("tmp_annotator_data/")

        # Load saved poses
        for path in self.__SAVE_DIR.glob("annotated_pose_*.json"):
            name = path.stem.split("_")[-1]
            with open(path, "r") as f:
                pose_dict = json.load(f)
                pose = Pose(**pose_dict)
                self.poses_dict[name] = pose

        rospy.Subscriber("/odom", Odometry, self.rospy_callback_odom)

    def rospy_callback_odom(self, msg: Odometry):
        with self.pose_lock:
            self.cur_pose = msg.pose.pose

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
                cur_pose_dict = {
                    slot: getattr(self.cur_pose, slot)
                    for slot in self.cur_pose.__slots__
                }
                with open(Path(self.__SAVE_DIR) / f"annotated_pose_{name}.json", "w") as f:
                    json.dump(cur_pose_dict, f)
                    f.write(str(self.cur_pose))
        else:
            print("No pose to save. Is the robot running?")

    def callback_delete(self, name: str):
        if self.poses_dict.pop(name, None) is None:
            print("No pose named {} to delete.".format(name))

    def callback_help(self):
        print(
            """Commands:
                list: List saved poses.
                save <name>: Save the robot's current pose as <name>. Overwrites if <name> already exists.
                delete <name>: Delete the pose given by <name>.
                goto <name>: Sends the robot to the pose given by <name>.
                help: Show this list of commands
                quit: Exit the program"""
        )

    def callback_goto(self, name: str):
        if name in self.poses_dict:
            print("Sending robot to {}".format(name))
            self.poses_dict[name]
        else:
            print("No pose named {} to goto.".format(name))

    def spin(self):
        while True:
            args = input("> ")
            command = args.split(" ")[0]

            if command == "quit":
                break

            if command in self.command_dict:
                if len(args.split(" ")) > 1:
                    self.command_dict[command](args.split(" ", 1)[1])
                else:
                    self.command_dict[command]()

    def run(self) -> None:
        print(
            """
            Welcome to the map annotator!
            """
        )
        self.callback_help()
        self.spin()

if __name__ == "__main__":
    CommandAnnotator().run()
