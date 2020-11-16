cmake_minimum_required(VERSION 2.8.3)
project(camera_monitor)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
)

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS 
    roscpp
    rospy
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#add_executable(motion_execution_node
#  src/do_motion_exe_node.cpp
#  src/do_motion_exe_server.cpp
#)

#target_link_libraries(motion_execution_node
#  ${catkin_LIBRARIES}
#)

catkin_install_python(
  PROGRAMS scripts/camera_monitor.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#install(TARGETS motion_execution_node
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

foreach(dir generated_program)
   install(DIRECTORY ${dir}/
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach()
