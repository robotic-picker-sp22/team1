from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from visualization_msgs.msg import Marker
# ... Other imports ...
import rospy
import math
import robot_api

BASE = robot_api.Base()


def wait_for_time():
    """Wait for simulated time to begin.                          
    """
    while rospy.Time().now().to_sec() == 0:
        pass


def main():
    rospy.init_node('interactive_drive_marker')
    wait_for_time()

    server_f = InteractiveMarkerServer("forward_marker")
    server_f.insert(create_marker(0.5, 0), handle_forward)
    server_f.applyChanges()

    server_l = InteractiveMarkerServer("turn_left")
    server_l.insert(create_marker(0, -0.5), handle_turn_left)
    server_l.applyChanges()

    server_r = InteractiveMarkerServer("turn_right")
    server_r.insert(create_marker(0, 0.5), handle_turn_right)
    server_r.applyChanges()


    rospy.spin()


def create_marker(x, y):
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "base_link"
    int_marker.name = "m"
    int_marker.pose.position.x = x
    int_marker.pose.position.y = y
    int_marker.pose.orientation.w = 1

    box_marker = Marker()
    box_marker.type = Marker.SPHERE
    box_marker.pose.orientation.w = 1
    box_marker.scale.x = 0.15
    box_marker.scale.y = 0.15
    box_marker.scale.z = 0.15
    box_marker.color.r = 0.0
    box_marker.color.g = 0.5
    box_marker.color.b = 0.5
    box_marker.color.a = 1.0

    button_control = InteractiveMarkerControl()
    button_control.interaction_mode = InteractiveMarkerControl.BUTTON
    button_control.always_visible = True
    button_control.markers.append(box_marker)
    int_marker.controls.append(button_control)

    return int_marker


def handle_forward(input):
    if (input.event_type == InteractiveMarkerFeedback.BUTTON_CLICK):
        BASE.go_forward(0.5)
    else:
        rospy.loginfo(
            f'Cannot handle this InteractiveMarker event {input.event_type}')


def handle_turn_right(input):
    if (input.event_type == InteractiveMarkerFeedback.BUTTON_CLICK):
        BASE.turn(math.pi / 6.0)
    else:
        rospy.loginfo(
            f'Cannot handle this InteractiveMarker event {input.event_type}')


def handle_turn_left(input):
    if (input.event_type == InteractiveMarkerFeedback.BUTTON_CLICK):
        BASE.turn(math.pi / -6.0)
    else:
        rospy.loginfo(
            f'Cannot handle this InteractiveMarker event {input.event_type}')


if __name__ == '__main__':
    main()
