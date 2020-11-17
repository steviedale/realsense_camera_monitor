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

    alive = 'cannot contact' not in error

    if DEBUG:
        print("is_camera_node_alive() -> {}".format(alive))
        if not alive:
            print("called: {}".format(cmd))
            print("returned output: {}".format(output))
            print("returned error: {}".format(error))

    return alive


# WIP
def toggle_ethernet_port():
    cmd = 'sudo echo "on" > /sys/class/net/"$(ls /sys/class/net/ | grep -E \'^e\')"/power/control'
    process = Popen(cmd, stdin=PIPE, stderr=PIPE, stdout=PIPE, shell=True)
    output, error = process.communicate()
    output, error = output.decode('utf-8'), error.decode('utf-8')


if __name__ == '__main__':
    # create node
    rospy.init_node('realsense_camera_monitor')

    # get the path to the launch file we want to run
    rospack = rospkg.RosPack()
    # launch_file = os.path.join(rospack.get_path('realsense2_camera'), 'launch', 'rs_camera.launch')
    launch_file = os.path.join(rospack.get_path('rars_application'), 'launch', 'gantry', 'bringup_realsense.launch')

    # init launch object
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(uuid, [launch_file])

    # check if node is alive, loop once per second
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