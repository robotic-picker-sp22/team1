#! /usr/bin/env python

import copy
from dis import dis
import math
from turtle import distance
from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry
import rospy
import tf.transformations as tft


def get_angle(odometry: Odometry):
    quaternion = odometry.pose.pose.orientation
    matrix = tft.quaternion_matrix(
        [quaternion.x, quaternion.y, quaternion.z, quaternion.w])
    x = matrix[0, 0]
    y = matrix[1, 0]
    return math.atan2(-x, y) % (2*math.pi)


class Base(object):
    """Base controls the mobile base portion of the Fetch robot.

    Sample usage:
        base = robot_api.Base()
        while CONDITION:
            base.move(0.2, 0)
        base.stop()
    """

    def __init__(self):
        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self._odom_sub = rospy.Subscriber(
            'odom', Odometry, callback=self._odom_callback)
        self._latest_odom = None

    def _odom_callback(self, msg: Odometry):
        self._latest_odom = msg

    def go_forward(self, distance, speed=0.1):
        """Moves the robot a certain distance.

        It's recommended that the robot move slowly. If the robot moves too
        quickly, it may overshoot the target. Note also that this method does
        not know if the robot's path is perturbed (e.g., by teleop). It stops
        once the distance traveled is equal to the given distance or more.

        Args:
            distance: The distance, in meters, to move. A positive value
                means forward, negative means backward.
            speed: The speed to travel, in meters/second.
        """
        while not self._latest_odom:
            pass
        start = copy.deepcopy(self._latest_odom)
        rate = rospy.Rate(10)
        direction = -1 if distance < 0 else 1
        while (remaining_distance := direction * (distance - (self._latest_odom.pose.pose.position.x - start.pose.pose.position.x))) > 0:
            linear_speed = max(0.05, min(speed, remaining_distance))
            self.move(direction * linear_speed, 0)
            rate.sleep()

    def turn(self, angular_distance, speed=0.5):
        """Rotates the robot a certain angle.

        Args:
            angular_distance: The angle, in radians, to rotate. A positive
                value rotates counter-clockwise.
            speed: The angular speed to rotate, in radians/second.
        """
        direction = -1 if angular_distance < 0 else 1
        angular_distance = abs(angular_distance) % (2 * math.pi)
        while not self._latest_odom:
            pass
        start = copy.deepcopy(self._latest_odom)
        max_angle_travelled = 0

        def angle_travelled():
            current_angle = get_angle(self._latest_odom)
            start_angle = get_angle(start)
            # print(f"start angle: {start_angle}")
            # print(f"current angle: {current_angle}")
            if direction > 0:
                if start_angle <= current_angle:
                    return current_angle - start_angle
                else:
                    return 2 * math.pi - start_angle + current_angle
            else:
                if start_angle >= current_angle:
                    return start_angle - current_angle
                else:
                    return 2 * math.pi - current_angle + start_angle

        rate = rospy.Rate(10)
        while (remaining_distance := angular_distance - angle_travelled()) > 0 and max_angle_travelled <= angle_travelled():
            max_angle_travelled = angle_travelled()
            # print(remaining_distance)
            angular_speed = max(0.25, min(speed, remaining_distance))
            # print(f"angular speed: {angular_speed}")
            # print(f"direction: {direction}")
            # self.move(0, direction * angular_speed)
            rate.sleep()

    def move(self, linear_speed, angular_speed):
        """Moves the base instantaneously at given linear and angular speeds.

        "Instantaneously" means that this method must be called continuously in
        a loop for the robot to move.

        Args:
            linear_speed: The forward/backward speed, in meters/second. A
                positive value means the robot should move forward.
            angular_speed: The rotation speed, in radians/second. A positive
                value means the robot should rotate clockwise.
        """
        vel = Twist()
        vel.linear.x = linear_speed
        vel.angular.z = angular_speed
        self.pub.publish(vel)

    def stop(self):
        """Stops the mobile base from moving.
        """
        msg = Twist()
        rospy.loginfo("stop")
        self.pub.publish(msg)
