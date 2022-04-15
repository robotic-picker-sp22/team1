# from functools import partial
# from interactive_markers.interactive_marker_server import InteractiveMarkerServer
# from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
# from visualization_msgs.msg import Marker
# # ... Other imports ...
# import rospy
# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import Twist
# import tf.transformations as tft
# import math
# import robot_api

# BASE = robot_api.Base()


# def wait_for_time():
#     """Wait for simulated time to begin.                          
#     """
#     while rospy.Time().now().to_sec() == 0:
#         pass


# def main():
#     rospy.init_node('interactive_drive_marker')
#     wait_for_time()

#     server = InteractiveMarkerServer("simple_marker")

#     rospy.Subscriber(
#         'odom', Odometry, callback=partial(odom_callback, server=server))

#     #publisher = rospy.Publisher("cmd_vel", Twist, queue_size=10)

#     rospy.spin()


# def odom_callback(msg: Odometry, server: InteractiveMarkerServer):
#     latest_odom = msg
#     create_marker(server, latest_odom)


# def create_marker(server, odom):
#     int_marker = InteractiveMarker()
#     int_marker.header.frame_id = "base_link"
#     int_marker.name = "my_marker"
#     # int_marker.description = "Simple Click Control"

#     # compute marker's position
#     # position = odom.pose.pose.position
#     # quaternion = odom.pose.pose.orientation
#     # matrix = tft.quaternion_matrix(
#     #     [quaternion.x, quaternion.y, quaternion.z, quaternion.w])
#     # x = matrix[0, 1]
#     # y = matrix[1, 1]
#     # # print("x", x)
#     # # print("y", y)
#     int_marker.pose.position.x = 0.5
#     # int_marker.pose.position.y = 0
#     int_marker.pose.orientation.w = 1

#     box_marker = Marker()
#     box_marker.type = Marker.SPHERE
#     box_marker.pose.orientation.w = 1
#     box_marker.scale.x = 0.25
#     box_marker.scale.y = 0.25
#     box_marker.scale.z = 0.25
#     box_marker.color.r = 0.0
#     box_marker.color.g = 0.5
#     box_marker.color.b = 0.5
#     box_marker.color.a = 1.0

#     button_control = InteractiveMarkerControl()
#     button_control.interaction_mode = InteractiveMarkerControl.BUTTON
#     button_control.always_visible = True
#     button_control.markers.append(box_marker)
#     int_marker.controls.append(button_control)

#     server.insert(int_marker, handle_viz_input)
#     server.applyChanges()


# def handle_viz_input(input):
#     if (input.event_type == InteractiveMarkerFeedback.BUTTON_CLICK):
#         BASE.go_forward(0.5)
#     else:
#         rospy.loginfo(
#             f'Cannot handle this InteractiveMarker event {input.event_type}')


# if __name__ == '__main__':
#     main()
