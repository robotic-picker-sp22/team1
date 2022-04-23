#! /usr/bin/env python
import rospy
import tf

def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass

def main():
    rospy.init_node('ee_pose_demo')
    wait_for_time()

    listener = tf.TransformListener()
    rospy.sleep(0.2)

    while not rospy.is_shutdown():
        print(f"Time: {rospy.Time.now().to_sec()}")
        try:
            translation, quartenion = listener.lookupTransform("gripper_link", "base_link", rospy.Time(0))
            print("Translation: ", translation)
            print("Quartenion: ", quartenion)
        except tf.ConnectivityException:
            print("Exception: ConnectivityException")
        except tf.LookupException:
            print("Exception: LookupException")
        except tf.ExtrapolationException:
            print("Exception: ExtrapolationException")
        print("\n")


        # Sleep for 1s
        rospy.sleep(rospy.Duration(1.0))




if __name__ == '__main__':
    main()
