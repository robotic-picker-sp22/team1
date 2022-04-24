#! /usr/bin/env python
import enum
import rospy
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from visualization_msgs.msg import Marker, MenuEntry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import ColorRGBA

from robot_api import Arm, Gripper



GRIPPER_MESH = 'package://fetch_description/meshes/gripper_link.dae'
L_FINGER_MESH = 'package://fetch_description/meshes/l_gripper_finger_link.STL'
# L_FINGER_MESH = 'package://fetch_description/meshes/l_gripper_finger_link.dae'
R_FINGER_MESH = 'package://fetch_description/meshes/r_gripper_finger_link.STL'
# R_FINGER_MESH = 'package://fetch_description/meshes/r_gripper_finger_link.dae'


class MenuIDs(enum.Enum):
    GOTO: int = 1


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


class GripperTeleop(object):
    def __init__(self, arm, gripper, im_server):
        self._arm = arm
        self._gripper = gripper

        self._im_server = im_server
        self._im_marker = None

        self._last_pose = None
        self._cur_color = ColorRGBA(0.0, 1.0, 0.0, 1.0)

    def __get_base_markers(self):

        OFFSET_X = 0.17
        OFFSET_X = 0.16645

        gripper_marker = Marker()
        gripper_marker.type = Marker.MESH_RESOURCE
        gripper_marker.mesh_resource = GRIPPER_MESH
        gripper_marker.pose.position.x = OFFSET_X
        gripper_marker.scale.x = 1.0
        gripper_marker.scale.y = 1.0
        gripper_marker.scale.z = 1.0
        gripper_marker.color = self._cur_color
        # gripper_marker.pose.orientation.w = 1

        l_finger_marker = Marker()
        l_finger_marker.type = Marker.MESH_RESOURCE
        l_finger_marker.mesh_resource = L_FINGER_MESH
        l_finger_marker.pose.position.x = OFFSET_X
        l_finger_marker.pose.position.y = -0.055
        l_finger_marker.scale.x = 1.0
        l_finger_marker.scale.y = 1.0
        l_finger_marker.scale.z = 1.0
        l_finger_marker.color = self._cur_color

        # l_finger_marker.pose.orientation.w = 1

        r_finger_marker = Marker()
        r_finger_marker.type = Marker.MESH_RESOURCE
        r_finger_marker.mesh_resource = R_FINGER_MESH
        r_finger_marker.pose.position.x = OFFSET_X
        r_finger_marker.pose.position.y = 0.055
        r_finger_marker.scale.x = 1.0
        r_finger_marker.scale.y = 1.0
        r_finger_marker.scale.z = 1.0
        r_finger_marker.color = self._cur_color
        # r_finger_marker.pose.orientation.w = 1

        return gripper_marker, l_finger_marker, r_finger_marker

    def __make_6dof_control(self, int_marker, base_markers):
        control_gripper = InteractiveMarkerControl()
        control_gripper.always_visible = True
        control_gripper.markers.append(base_markers[0])

        control_gripper.name = "move"
        control_gripper.interaction_mode = InteractiveMarkerControl.MOVE_3D
        int_marker.controls.append(control_gripper)


        control_fingers = InteractiveMarkerControl()
        control_fingers.always_visible = True
        control_fingers.markers.extend(base_markers[1:])

        control_fingers.name = "rotate"
        control_fingers.interaction_mode = InteractiveMarkerControl.ROTATE_3D
        int_marker.controls.append(control_fingers)

    def __make_menu_control(self, im_marker):
        goto_entry = MenuEntry()
        goto_entry.id = MenuIDs.GOTO.value
        goto_entry.parent_id = 0
        goto_entry.title = "Go to"
        goto_entry.command_type = MenuEntry.FEEDBACK

        im_marker.menu_entries.append(goto_entry)


    def __update_colors(self):
        if self._im_marker is None:
            return
        for control in self._im_marker.controls:
            for marker in control.markers:
                marker.color = self._cur_color
        self._im_server.erase(self._im_marker.name)
        self._im_server.insert(self._im_marker, feedback_cb=self.handle_feedback)
        self._im_server.applyChanges()

    def start(self):
        gripper_im = InteractiveMarker()
        gripper_im.name = "Gripper IK"
        gripper_im.header.stamp = rospy.Time.now()
        gripper_im.description = "Where gripper should go"
        gripper_im.header.frame_id = "gripper_link"

        # Create markers for the gripper
        base_markers = self.__get_base_markers()

        # Add 3d location + orientation controls
        self.__make_6dof_control(gripper_im, base_markers)
        # Add menu control
        self.__make_menu_control(gripper_im)

        self._im_marker = gripper_im
        self._im_server.insert(self._im_marker, feedback_cb=self.handle_feedback)
        self._im_server.applyChanges()


    def handle_feedback(self, feedback: InteractiveMarkerFeedback):
        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            self.handle_pose_update(feedback)
        elif feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
            self.handle_menu_select(feedback)
        pass

    def handle_pose_update(self, feedback: InteractiveMarkerFeedback):
        pose_stamped = PoseStamped()
        pose_stamped.header = feedback.header
        pose_stamped.pose = feedback.pose
        self._last_pose = pose_stamped

        self._im_marker.header = feedback.header
        self._im_marker.pose = feedback.pose
        if self._arm.compute_ik(pose_stamped) is not None:
            self._cur_color.r = 0.0
            self._cur_color.g = 1.0
        else:
            self._cur_color.r = 1.0
            self._cur_color.g = 0.0
        self.__update_colors()

    def handle_menu_select(self, feedback: InteractiveMarkerFeedback):
        if feedback.menu_entry_id == MenuIDs.GOTO.value:
            if self._last_pose is None:
                return

            arm_joints = self._arm.compute_ik(self._last_pose)
            if arm_joints is None:
                return

            self._arm.move_to_joints(arm_joints)


class AutoPickTeleop(object):
    def __init__(self, arm, gripper, im_server):
        self._arm = arm
        self._gripper = gripper
        self._im_server = im_server

    def start(self):
        obj_im = InteractiveMarker()
        self._im_server.insert(obj_im, feedback_cb=self.handle_feedback)
        self._im_server.applyChanges()


    def handle_feedback(self, feedback):
        pass


def main():
    rospy.init_node("interactive_gripper_teleop")
    wait_for_time()

    arm = Arm()
    gripper = Gripper()

    im_server = InteractiveMarkerServer('gripper_im_server')
    auto_pick_im_server = InteractiveMarkerServer('auto_pick_im_server')
    teleop = GripperTeleop(arm, gripper, im_server)
    auto_pick = AutoPickTeleop(arm, gripper, auto_pick_im_server)
    teleop.start()
    auto_pick.start()
    rospy.spin()


if __name__ == "__main__":
    main()
