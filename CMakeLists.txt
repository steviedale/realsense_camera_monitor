cmake_minimum_required(VERSION 2.8.3)
project(realsense_camera_monitor)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  realsense2_camera
)

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS
    roscpp
    rospy
    realsense2_camera
)

catkin_install_python(
  PROGRAMS scripts/camera_monitor.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

foreach(dir generated_program)
   install(DIRECTORY ${dir}/
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach()
