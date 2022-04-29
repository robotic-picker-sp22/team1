#! /usr/bin/env python

from moveit_python import PlanningSceneInterface
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
import robot_api
import rospy


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


def print_usage():
    print('Usage: rosrun applications lab26_obstacles.py')
    print('Drive the robot until the PlanningScene lines up with the point cloud.')


def main():
    rospy.init_node('lab26_obstacles')
    wait_for_time()

    # planning_scene = PlanningSceneInterface('base_link')
    planning_scene = PlanningSceneInterface('map')
    planning_scene.clear()

    planning_scene.removeCollisionObject('table')
    planning_scene.removeCollisionObject('floor')

    # # base link
    # planning_scene.addBox('table', 0.5, 1, 0.83, 1, 0, 0.72 / 2)
    # map
    planning_scene.addBox('table', 1, 1, 1.52, 4, 3, 0.0)


    # Map frame
    # Front left corner: 3.6, 3.3, 0.76
    # Back right corner: 4.5, 2.75, 0.76


    # planning_scene.addBox('floor', 2, 2, 0.01, 0, 0, 0.01 / 2)
    # floor = CollisionObject()
    # floor.header.frame_id = 'base_link'
    # floor.operation = CollisionObject.ADD
    # floor.id = 'floor'
    # floor.primitives.append(SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[2, 2, 0.01]))
    # floor_pose = Pose()
    # floor_pose.position.x = 0.0
    # floor_pose.position.y = 0.0
    # floor_pose.position.z = 0.01 / 2.0
    # floor.primitive_poses.append(floor_pose)
    # planning_scene.addCollisionObject(floor)

    rospy.sleep(2)


if __name__ == '__main__':
    main()
