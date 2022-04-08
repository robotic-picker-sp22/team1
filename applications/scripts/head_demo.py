#! /usr/bin/env python

import rospy
import robot_api


def print_usage():
    # NOTE: We don't expect you to implement look_at for Kuri
    # But if you do, show us because that would be impressive ;)
    # `eyes`, naturally, is Kuri only.
    print('Usage:')
    print('    rosrun applications head_demo.py look_at FRAME_ID X Y Z')
    print('    rosrun applications head_demo.py pan_tilt PAN_ANG TILT_ANG')
    print('    rosrun applications head_demo.py eyes ANG')
    print('Examples:')
    print('    rosrun applications head_demo.py look_at base_link 1 0 0.3')
    print('    rosrun applications head_demo.py pan_tilt 0 0.707')
    print('    rosrun applications head_demo.py eyes .50')


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


def main():
    rospy.init_node('head_demo')
    wait_for_time()
    argv = rospy.myargv()
    if len(argv) < 2:
        print_usage()
        return
    command = argv[1]

    if command == 'look_at':
        if len(argv) < 6:
            print_usage()
            return
        args = argv[2], float(argv[3]), float(argv[4]), float(
            argv[5])
    elif command == 'pan_tilt':
        if len(argv) < 4:
            print_usage()
            return
        args = float(argv[2]), float(argv[3])
    else:
        print_usage()
        exit(0)

    head = robot_api.Head()
    getattr(head, command)(*args)


if __name__ == '__main__':
    main()
