#! /usr/bin/env python
import copy
import enum
from multiprocessing import Lock
from typing import Optional
import rospy
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from visualization_msgs.msg import Marker, MenuEntry
from geometry_msgs.msg import Pose, PoseStamped
from tf2_geometry_msgs import PoseStamped as PoseStampedTF2
from std_msgs.msg import ColorRGBA
from sensor_msgs.msg import JointState
import tf
from moveit_python import PlanningSceneInterface


from robot_api import Arm, Gripper, ArmJoints



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


class ArmJointsServer:
    def __init__(self) -> None:
        self.__arm_joints = ArmJoints()
        self.__lock = Lock()

        rospy.Subscriber('/joint_states', JointState, self.__callback_joint_states)

    def __callback_joint_states(self, msg: JointState):
        with self.__lock:
            for name, pos in zip(msg.name, msg.position):
                if name in self.__arm_joints.names():
                    # Example: shoulder_pan_joint -> set_shoulder_pan
                    getattr(self.__arm_joints, f"set_{name[:-6]}")(pos)

    def get_current_arm_joints(self) -> ArmJoints:
        with self.__lock:
            return copy.deepcopy(self.__arm_joints)


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
        self.__arm_joints_server = ArmJointsServer()

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
        control_gripper.markers.extend(base_markers)

        control_gripper.name = "move"
        control_gripper.interaction_mode = InteractiveMarkerControl.MOVE_3D
        int_marker.controls.append(control_gripper)

        for axis in ["x", "y", "z"]:
            for dof in ["rotate"]:
                control = InteractiveMarkerControl()
                control.always_visible = True
                control.name = f"gripper_{dof}_{axis}"
                control.orientation.w = 1
                setattr(control.orientation, axis, 1)
                control.interaction_mode = getattr(InteractiveMarkerControl, f"{dof.upper()}_AXIS")
                int_marker.controls.append(control)

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

    def __handle_goto(self, pose: Optional[PoseStamped] = None, use_raw_joints: bool = False) -> bool:
        pose = pose if pose is not None else self._last_pose

        if pose is None:
            return False

        arm_joints = self._arm.compute_ik(pose)
        if arm_joints is None:
            print("arm_joints not found")
            return False

        if use_raw_joints:
            print("arm_joints move_to_joints")
            self._arm.move_to_joints(arm_joints)
        else:
            pose_tf2 = PoseStampedTF2()
            pose_tf2.header.frame_id = pose.header.frame_id
            pose_tf2.header.stamp = pose.header.stamp
            pose_tf2.pose = pose.pose
            
            self.__tf_listener.waitForTransform("map", pose_tf2.header.frame_id, pose.header.stamp, rospy.Duration(10))
            pose_tf2 = self.__tf_listener.transformPose("map", pose_tf2)
            self.__tf_listener.waitForTransform("base_link", pose_tf2.header.frame_id, pose.header.stamp, rospy.Duration(10))
            pose_tf2 = self.__tf_listener.transformPose("base_link", pose_tf2)
            
            self._arm.move_to_pose(
                pose_tf2,
                allowed_planning_time=15,
                execution_timeout=10,
                num_planning_attempts=5,
                replan=False,
            )

        return True

    def handle_menu_select(self, feedback: InteractiveMarkerFeedback):
        if feedback.menu_entry_id == MenuIDs.Go_To.value:
            self.__handle_goto()
        elif feedback.menu_entry_id == MenuIDs.Grasp_Close.value:
            self._gripper.close()
        elif feedback.menu_entry_id == MenuIDs.Grasp_Open.value:
            self._gripper.open()

    def __get_best_grasp_pose(self, pose: PoseStamped) -> Optional[PoseStamped]:
        cur_arm_joints = self.__arm_joints_server.get_current_arm_joints()

        result = None
        result_score = 100000
        result_pos_axis = None

        PREGRASP_OFFSET = 0.17
        for (position_axis, orientation_axis) in [
            # ("x", "x"),
            ("x", "w"),
            # ("y", "z"),
            # ("z", "y"),
        ]:
            for orientation_val in [1.0]:
                pregrasp_pose = copy.deepcopy(pose)
                # Offset so that object lies in the gripper
                setattr(pregrasp_pose.pose.position, position_axis, getattr(pregrasp_pose.pose.position, position_axis) - PREGRASP_OFFSET)
                setattr(pregrasp_pose.pose.orientation, orientation_axis, orientation_val)

                arm_joints = self._arm.compute_ik(pregrasp_pose)
                if arm_joints is not None:
                    cur_score = cur_arm_joints.distance_from(arm_joints)
                    if cur_score < result_score:
                        print(f"orientation_axis: {position_axis}")
                        print(f"orientation_axis: {orientation_axis}")
                        result = pregrasp_pose
                        result_score = cur_score
                        result_pos_axis = position_axis

        return result, result_pos_axis

    def grasp_object(self, object_pose: PoseStamped):
        # 0. Calculate pre-grasp pose
        object_pose = copy.deepcopy(object_pose)

        
        grasp_pose, pos_axis = self.__get_best_grasp_pose(object_pose)
        if grasp_pose is None:
            return


        # 1. Open gripper
        self._gripper.open()

        pregrasp_pose = copy.deepcopy(grasp_pose)
        if pos_axis == "x":
            pregrasp_pose.pose.position.x -= 0.15
        elif pos_axis == "z":
            pregrasp_pose.pose.position.z += 0.

        self.__handle_goto(pregrasp_pose)
        self.__handle_goto(grasp_pose, use_raw_joints=True)
        self._gripper.close()
        self.__handle_goto()


class AutoPickTeleop(object):
    def __init__(self, arm, gripper, im_server, grasp_callback, object_name="object"):
        self._arm = arm
        self._gripper = gripper
        self._im_server = im_server

        self.__grasp_callback = grasp_callback
        self.__object_name = object_name

        self.__planning_scene = PlanningSceneInterface('map')

        self.__br = tf.TransformBroadcaster()
        rospy.sleep(0.1)

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
        obj_im.name = self.__object_name
        obj_im.header.stamp = rospy.Time.now()
        obj_im.description = "Object"
        obj_im.header.frame_id = "base_link"

        obj_im.pose.position.x = 0.5
        obj_im.pose.position.y = 0.5
        obj_im.pose.position.z = 0.5
        obj_im.pose.orientation.w = 1

        # Update tf for the first time
        self.__update_tf(obj_im.pose, obj_im.header.frame_id, obj_im.header.stamp)


        # Create markers for the obj
        base_marker = Marker()
        base_marker.type = Marker.CUBE
        base_marker.color = ColorRGBA(255.0 / 255, 240.0 / 255, 0 / 255, 1.0)
        base_marker.scale.x = 0.065
        base_marker.scale.y = 0.065
        base_marker.scale.z = 0.065

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

    def __update_tf(self, pose: Pose, parent_frame: str, stamp: rospy.Time):
        self.__br.sendTransform(
            translation=(pose.position.x, pose.position.y, pose.position.z),
            rotation=(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
            time=stamp,
            child=self.__object_name,
            parent=parent_frame
        )

    def handle_feedback(self, feedback):
        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            # Update tf
            if feedback.header.frame_id == "map":
                self.__planning_scene.removeCollisionObject(self.__object_name)
                self.__planning_scene.addBox(self.__object_name, 0.065, 0.065, 0.065, feedback.pose.position.x, feedback.pose.position.y, feedback.pose.position.z)

            self.__update_tf(feedback.pose, feedback.header.frame_id, rospy.Time.now())

        if feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
            if feedback.menu_entry_id == 1:
                pose = PoseStamped()
                pose.header = feedback.header
                pose.header.frame_id = self.__object_name

                # Since we changed the frame of the pose, the position should be 0
                pose.pose = Pose()
                pose.pose.orientation.w = 1.0
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
