import textwrap
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import InteractiveMarkerFeedback


class CommandPerceptionInterface:
    def __init__(self):
        self.command_dict = {
            "list": self.callback_list,
            "goto": self.callback_goto,
            "help": self.callback_help,
        }
        # rospy.Subscriber("/segment_marker", Marker, self.rospy_callback_marker)
        rospy.Subscriber("/auto_pick_im_server/feedback", InteractiveMarkerFeedback, self.rospy_callback_interactive_marker_feedback)

        self.__object_poses = {}
        self.__move_to_publisher = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
        self.__grasp_object_publisher = rospy.Publisher("/gripper_ik/grasp", PoseStamped, queue_size=1)

    def rospy_callback_marker(self, marker: Marker):
        pose_stamped = PoseStamped()
        pose_stamped.header = marker.header
        pose_stamped.pose = marker.pose
        name = marker.text.split(" (", maxsplit=1)[0]
        self.__object_poses[name] = pose_stamped

    def rospy_callback_interactive_marker_feedback(self, feedback):
        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            pose_stamped = PoseStamped()
            pose_stamped.header = feedback.header
            pose_stamped.pose = feedback.pose
            name = "spoof"
            self.__object_poses[name] = pose_stamped

    def callback_list(self):
        if not self.__object_poses:
            return

        print("Saved poses:\n\t- " + "\n\t- ".join(self.__object_poses.keys()))

    def callback_goto(self, name: str):
        if name not in self.__object_poses:
            print("No pose saved with name '{}'".format(name))
            return

        self.__grasp_object_publisher.publish(self.__object_poses[name])
        # self.__move_to_publisher.publish(self.__object_poses[name])

    def callback_help(self):
        print(textwrap.dedent(
            """
            Commands:
             - list: List saved poses.
             - goto <name>: Sends the robot pick up object with <name> and place it in the bin.
             - help: Show this list of commands
             - quit: Exit the program"""
        ))

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


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


def main():
    rospy.init_node('perception_interface')
    wait_for_time()
    CommandPerceptionInterface().run()


if __name__ == '__main__':
    main()
