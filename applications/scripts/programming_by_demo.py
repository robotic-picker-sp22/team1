#! /usr/bin/env python

import math
import rospy
import tf
from ar_track_alvar_msgs.msg import AlvarMarkers
from multiprocessing import Lock
from geometry_msgs.msg import PoseStamped, Pose
from robot_api import Gripper
import pickle
from robot_controllers_msgs.msg import ControllerState, QueryControllerStatesGoal


"""
NOTE: This code assume that robot is stationary!
"""

def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


class ActionSaver():
    def __init__(self):
        self.__tf_listener = tf.TransformListener()

        rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.__alvar_markers_callback)
        self.__cur_markers = {}
        self.__cur_markers_lock = Lock()

        self.__cur_program = []

        self.__gripper = Gripper()

    def __alvar_markers_callback(self, markers):
        with self.__cur_markers_lock:
            for marker in markers.markers:
                self.__cur_markers[marker.id] = marker

    def __reset_program(self):
        _sure = input("Are you sure? [y/N]")
        if _sure.lower() == "y":
            self.__cur_program = []

    def __save_pose(self):
        t = rospy.Time.now()
        self.__tf_listener.waitForTransform("wrist_flex_link", "base_link", t, rospy.Duration(10))
        position, orientation = self.__tf_listener.lookupTransform("wrist_flex_link", "base_link", t)

        # Create wrist pose
        wrist_pose = PoseStamped()
        wrist_pose.header.stamp = t
        wrist_pose.header.frame_id = "base_link"
        wrist_pose.pose.position.x = position[0]
        wrist_pose.pose.position.y = position[1]
        wrist_pose.pose.position.z = position[2]
        wrist_pose.pose.orientation.x = orientation[0]
        wrist_pose.pose.orientation.y = orientation[1]
        wrist_pose.pose.orientation.z = orientation[2]
        wrist_pose.pose.orientation.w = orientation[3]

        # Calculate distance to every marker
        print("Frames:")
        with self.__cur_markers_lock:
            for id, marker in self.__cur_markers:
                print(f"\t- marker {id} (distance from gripper: {self.pose_dist(marker.pose.pose, wrist_pose.pose):.4g}")
        print(f"\t- base (distance from gripper: {self.pose_dist(wrist_pose.pose, Pose()):.4g}")
        print("")

        print("Which marker you want to save pose with respect to?")
        print("Please use ints: 1, 2, 3, 4 etc. or type base (default)")
        try:
            marker_id = int(input(">"))
            self.__tf_listener.waitForTransform(f"ar_marker_{marker_id}", "base_link", wrist_pose.header.stamp, rospy.Duration(10))
            wrist_pose = self.__tf_listener.transformPose(f"ar_marker_{marker_id}", wrist_pose)

        except:
            # base - do nothing
            pass

        self.__cur_program.append(("goto", wrist_pose))

    def __open_gripper(self):
        self.__gripper.open()
        self.__cur_program.append(("opengripper",))

    def __close_gripper(self):
        self.__gripper.close()
        self.__cur_program.append(("closegripper",))

    def __save_program(self):
        print("What's the programs name? (.prog.pkl will be appended to the end)")
        program_name = input(">")

        with open(f"{program_name}.prog.pkl", mode="wb") as f:
            pickle.dump(self.__cur_program, f)

    def __relax(self):
        self.__change_state(ControllerState.STOPPED)

    def __flex(self):
        self.__change_state(ControllerState.RUNNING)

    def __change_state(self, state):
        if rospy.get_param("use_sim_time"):
            pass

        goal = QueryControllerStatesGoal()
        state = ControllerState()
        state.name = 'arm_controller/follow_joint_trajectory'
        state.state = ControllerState.STOPPED
        goal.updates.append(state)
        self._controller_client.send_goal(goal)
        self._controller_client.wait_for_result()

    def __help(self):
        print("Usage: (Every command is case agnostic)")
        print("\trx OR relax - Relax the arm to start recording on the real robot (Ignored in sim)")
        print("\tfx OR flex - Relax the arm to end recording on the real robot (Ignored in sim)")
        print("\tr OR reset - Reset the currently recorded program (asks for confirmation)")
        print("\tsp OR savepose - Save current pose of gripper into the program")
        print("\top OR opengripper - Open gripper and save opening it into the program")
        print("\tcg OR closegripper - Close gripper and save closing it into the program")
        print("\ts OR save - Save current program into a file (Asks for a file name).")
        print("\th or help - Print this help.")

    @staticmethod
    def pose_dist(pose1: Pose, pose2: Pose):
        return math.sqrt(
            (pose1.position.x - pose2.position.x) ** 2
            + (pose1.position.y - pose2.position.y) ** 2
            + (pose1.position.z - pose2.position.z) ** 2
        )


    def __process_command(self, command):
        if command in {"rx", "relax"}:
            self.__relax()
        elif command in {"fx", "flex"}:
            self.__flex()
        elif command in {"r", "reset"}:
            self.__reset_program()
        elif command in {"sp", "savepose"}:
            self.__save_pose()
        elif command in {"og", "opengripper"}:
            self.__open_gripper()
        elif command in {"cg", "closegripper"}:
            self.__close_gripper()
        elif command in {"s", "save"}:
            self.__save_program
        elif command in {"h", "help"}:
            self.__help()
        else:
            print("Command unknown. Try pressing h/help")


    def run(self):
        self.__help()
        cur_command = ""
        while cur_command not in {"q", "quit"}:
            cur_command = input(">").lower()
            if cur_command not in {"q", "quit"}:
                self.__process_command(cur_command)



def main():
    rospy.init_node("pregramming_by_demo")
    wait_for_time()

    a = ActionSaver()
    a.run()


if __name__ == "__main__":
    main()
