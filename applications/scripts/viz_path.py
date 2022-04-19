from matplotlib.pyplot import show
import rospy
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA


class NavPath(object):
    def __init__(self, marker_publisher):
        self._path = []
        self._marker_publisher = marker_publisher

    def callback(self, msg: Odometry):
        if not self._path or msg.pose.pose.position != self._path[-1]:
            self._path.append(msg.pose.pose.position)
            self.show_path_in_rviz()

    def show_path_in_rviz(self):
        marker = Marker(
            type=Marker.LINE_STRIP,
            id=0,
            lifetime=rospy.Duration.from_sec(2**20),
            scale=Vector3(0.01, 0.10, 0.10),
            pose=Pose(Point(0, 0, 0), Quaternion(0, 0, 0, 1)),
            header=Header(frame_id='odom'),
            points=self._path,
            color=ColorRGBA(1.0, 0.0, 0.0, 0.8))
        self._marker_publisher.publish(marker)


def wait_for_time():
    """Wait for simulated time to begin.                          
    """
    while rospy.Time().now().to_sec() == 0:
        pass


def main():
    rospy.init_node('marker_demo')
    wait_for_time()

    marker_publisher = rospy.Publisher(
        'visualization_marker', Marker, queue_size=5)
    rospy.sleep(0.5)
    nav_path = NavPath(marker_publisher)
    rospy.Subscriber('odom', Odometry, nav_path.callback)
    rospy.spin()


if __name__ == '__main__':
    main()
