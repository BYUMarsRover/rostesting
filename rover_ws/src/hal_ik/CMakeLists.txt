cmake_minimum_required(VERSION 2.8.3)
project(hal_ik)

find_package(catkin REQUIRED COMPONENTS 
	trac_ik_lib 
	eigen_conversions
	std_msgs
	rover_msgs
	)

catkin_package(
  LIBRARIES
  CATKIN_DEPENDS
    trac_ik_lib
    rover_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${eigen_INCLUDE_DIRS}
)

add_executable(hal_ik src/hal_ik.cpp)
target_link_libraries(hal_ik ${catkin_LIBRARIES})

#add_executable(hal_visualizer src/hal_visualizer.py)
#target_link_libraries(hal_visualizer ${catkin_LIBRARIES})

#add_executable(teleop_controller src/teleop_controller.py)
#target_link_libraries(teleop_controller ${catkin_LIBRARIES})

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)
