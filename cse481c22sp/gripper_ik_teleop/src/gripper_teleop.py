#! /usr/bin/env python
import copy
import enum
from typing import Optional
import rospy
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from visualization_msgs.msg import Marker, MenuEntry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import ColorRGBA
import tf

from robot_api import Arm, Gripper



GRIPPER_MESH = 'package://fetch_description/meshes/gripper_link.dae'
L_FINGER_MESH = 'package://fetch_description/meshes/l_gripper_finger_link.STL'
# L_FINGER_MESH = 'package://fetch_description/meshes/l_gripper_finger_link.dae'
R_FINGER_MESH = 'package://fetch_description/meshes/r_gripper_finger_link.STL'
# R_FINGER_MESH = 'package://fetch_description/meshes/r_gripper_finger_link.dae'


class MenuIDs(enum.Enum):
    Go_To: int = 1
    Grasp_Close: int = 2
    Grasp_Open: int = 3


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


class GripperTeleop(object):
    OFFSET_X = 0.166
    # Techincally the TF between fingers is 0.065, but value below seems to better reflect open state
    # TODO: We can just subscribe to current TF and use that? It will change if gripper is activated
    OFFSET_FINGERS_Y = 0.0535

    def __init__(self, arm, gripper, im_server):
        self._arm = arm
        self._gripper = gripper

        self._im_server = im_server
        self._im_marker = None

        self._last_pose = None
        self._cur_color = ColorRGBA(0.0, 1.0, 0.0, 1.0)

        self.__tf_listener = tf.TransformListener()

    def __get_base_markers(self):

        gripper_marker = Marker()
        gripper_marker.type = Marker.MESH_RESOURCE
        gripper_marker.mesh_resource = GRIPPER_MESH
        gripper_marker.pose.position.x = self.OFFSET_X
        gripper_marker.scale.x = 1.0
        gripper_marker.scale.y = 1.0
        gripper_marker.scale.z = 1.0
        gripper_marker.color = self._cur_color

        l_finger_marker = Marker()
        l_finger_marker.type = Marker.MESH_RESOURCE
        l_finger_marker.mesh_resource = L_FINGER_MESH
        l_finger_marker.pose.position.x = self.OFFSET_X
        l_finger_marker.pose.position.y = -self.OFFSET_FINGERS_Y
        l_finger_marker.scale.x = 1.0
        l_finger_marker.scale.y = 1.0
        l_finger_marker.scale.z = 1.0
        l_finger_marker.color = self._cur_color


        r_finger_marker = Marker()
        r_finger_marker.type = Marker.MESH_RESOURCE
        r_finger_marker.mesh_resource = R_FINGER_MESH
        r_finger_marker.pose.position.x = self.OFFSET_X
        r_finger_marker.pose.position.y = self.OFFSET_FINGERS_Y
        r_finger_marker.scale.x = 1.0
        r_finger_marker.scale.y = 1.0
        r_finger_marker.scale.z = 1.0
        r_finger_marker.color = self._cur_color

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
        for menu_key, menu_value in MenuIDs.__members__.items():

            goto_entry = MenuEntry()
            goto_entry.id = menu_value.value
            goto_entry.parent_id = 0
            goto_entry.title = menu_key.replace("_", " ")
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

    def __handle_goto(self, pose: Optional[PoseStamped] = None) -> bool:
        pose = pose if pose is not None else self._last_pose

        if pose is None:
            return False

        arm_joints = self._arm.compute_ik(pose)
        if arm_joints is None:
            return False

        self._arm.move_to_joints(arm_joints)
        return True

    def handle_menu_select(self, feedback: InteractiveMarkerFeedback):
        if feedback.menu_entry_id == MenuIDs.Go_To.value:
            self.__handle_goto()
        elif feedback.menu_entry_id == MenuIDs.Grasp_Close.value:
            self._gripper.close()
        elif feedback.menu_entry_id == MenuIDs.Grasp_Open.value:
            self._gripper.open()

    def grasp_object(self, object_pose: PoseStamped):
        # 1. Open gripper
        self._gripper.open()

        transform = self.__tf_listener.lookupTransform(
            "gripper_link",
            object_pose.header.frame_id,
        )

        # 2. Calculate pre-grasp pose
        # TODO: This is incorrect. Should create a new frame for object, and then define pre-grasp pose in terms of that frame
        object_pose = copy.deepcopy(object_pose)
        object_pose.pose.position.x -= 0.1
        object_pose.pose.position.y += 0.1

        self.__handle_goto(object_pose)
        self._gripper.close()
        self.__handle_goto()


class AutoPickTeleop(object):
    def __init__(self, arm, gripper, im_server, grasp_callback):
        self._arm = arm
        self._gripper = gripper
        self._im_server = im_server

        self.__grasp_callback = grasp_callback

    def __make_6dof_control(self, int_marker):
        for axis in ["x", "y", "z"]:
            for dof in ["move", "rotate"]:
                control = InteractiveMarkerControl()
                control.always_visible = True
                control.name = f"{dof}_{axis}"
                control.orientation.w = 1
                setattr(control.orientation, axis, 1)
                control.interaction_mode = getattr(InteractiveMarkerControl, f"{dof.upper()}_AXIS")
                int_marker.controls.append(control)


    def start(self):
        obj_im = InteractiveMarker()
        obj_im.name = "Object"
        obj_im.header.stamp = rospy.Time.now()
        obj_im.description = "Object"
        obj_im.header.frame_id = "base_link"

        obj_im.pose.position.x = 0.5
        obj_im.pose.position.y = 0.5
        obj_im.pose.position.z = 0.5

        # Create markers for the obj
        base_marker = Marker()
        base_marker.type = Marker.CUBE
        base_marker.color = ColorRGBA(255.0 / 255, 240.0 / 255, 0 / 255, 1.0)
        base_marker.scale.x = 0.05
        base_marker.scale.y = 0.05
        base_marker.scale.z = 0.05

        # Add base marker to the interactive marker
        fake_control = InteractiveMarkerControl()
        fake_control.always_visible = True
        fake_control.markers.append(base_marker)
        obj_im.controls.append(fake_control)

        # Add 3d location + orientation controls
        self.__make_6dof_control(obj_im)
        # Add menu control
        grasp_entry = MenuEntry()
        grasp_entry.id = 1
        grasp_entry.parent_id = 0
        grasp_entry.title = "Pick up"
        grasp_entry.command_type = MenuEntry.FEEDBACK
        obj_im.menu_entries.append(grasp_entry)

        self._im_server.insert(obj_im, feedback_cb=self.handle_feedback)
        self._im_server.applyChanges()

    def handle_feedback(self, feedback):
        if feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
            if feedback.menu_entry_id == 1:
                pose = PoseStamped()
                pose.header = feedback.header
                pose.pose = feedback.pose
                self.__grasp_callback(pose)

def main():
    rospy.init_node("interactive_gripper_teleop")
    wait_for_time()

    arm = Arm()
    gripper = Gripper()

    im_server = InteractiveMarkerServer('gripper_im_server')
    auto_pick_im_server = InteractiveMarkerServer('auto_pick_im_server')
    teleop = GripperTeleop(arm, gripper, im_server)
    auto_pick = AutoPickTeleop(arm, gripper, auto_pick_im_server, teleop.grasp_object)
    teleop.start()
    auto_pick.start()
    rospy.spin()


if __name__ == "__main__":
    main()
