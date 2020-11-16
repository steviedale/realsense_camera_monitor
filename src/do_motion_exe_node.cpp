#include <rars_motion_execution/do_motion_exe_server.h>
#include <ros/ros.h>

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "motion_execution_node");
  ros::NodeHandle nh;

  rars_motion_execution::MotionExecutionServer server(nh);
  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::waitForShutdown();
}
