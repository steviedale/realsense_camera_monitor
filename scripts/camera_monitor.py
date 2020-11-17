#!/usr/bin/env python3
import rospy
import roslaunch
import rospkg
from subprocess import Popen, PIPE
import os

DEBUG = True


def is_camera_node_alive():
    cmd = 'rosnode info /camera/realsense2_camera'
    process = Popen(cmd, stdin=PIPE, stderr=PIPE, stdout=PIPE, shell=True)
    output, error = process.communicate()
    output, error = output.decode('utf-8'), error.decode('utf-8')
    if DEBUG:
        print("is_camera_node_alive()")
        print("called: {}".format(cmd))
        print("returned output: {}".format(output))
        print("returned error: {}".format(error))
    return 'cannot contact' not in error


if __name__ == '__main__':
    rospy.init_node('realsense_camera_monitor')
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    rospack = rospkg.RosPack()
    launch_file = os.path.join(rospack.get_path('realsense2_camera'), 'launch', 'rs_camera.launch')
    launch = roslaunch.parent.ROSLaunchParent(uuid, [launch_file])

    while True:
        if not is_camera_node_alive():
            print("camera node is dead, re-launching")
            launch.shutdown()
            launch.start()
        else:
            if DEBUG:
                print("camera node is alive and well")
        rospy.sleep(1)

    print("realsense_camera_monitor node finished.")