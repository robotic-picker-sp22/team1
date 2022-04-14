#!/usr/bin/env python

import re
import robot_api
import rospy
from web_teleop.srv import SetJointValue, SetJointValueResponse


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


class ActuatorServer(object):
    def __init__(self):
        self._torso = robot_api.Torso()
        self._arm = robot_api.Arm()
        self._head = robot_api.Head()
        self._gripper = robot_api.Gripper()
        self._arm_joints = robot_api.ArmJoints()

    def handle_set_joint_value(self, request):
        if request.body_part == "torso":
            self._torso.set_height(request.value)
        elif request.body_part == "head":
            try:
                self._head.pan_tilt(
                    **{request.joint: request.value}
                )
            except:
                rospy.logwarn("Head didn't work")
                pass
        elif request.body_part == "arm":
            try:
                getattr(self._arm_joints, f"set_{request.joint}")(request.value)
                self._arm.move_to_joints(self._arm_joints)
            except:
                rospy.logwarn("Stuff happened")
                pass
        elif request.body_part == "gripper":
            if request.value > 0:
                self._gripper.close(request.value)
            else:
                self._gripper.open()
        else:
            rospy.logwarn(f"Unknown body part {request.body_part}")
        return SetJointValueResponse()


def main():
    rospy.init_node('web_teleop_actuators')
    wait_for_time()
    server = ActuatorServer()
    torso_service = rospy.Service('web_teleop/set_joint_value', SetJointValue,
                                  server.handle_set_joint_value)
    rospy.spin()


if __name__ == '__main__':
    main()