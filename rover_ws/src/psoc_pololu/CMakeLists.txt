cmake_minimum_required(VERSION 2.8.12)

project(pololu_driver)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  roscpp
  rospy
  rover_msgs
)


find_package(Boost REQUIRED COMPONENTS system)
find_package(Threads)

# add_message_files(FILES Drive.msg)

# generate_messages(DEPENDENCIES std_msgs)

# add package modules path, not needed in dependend packages
list(INSERT CMAKE_MODULE_PATH 0 "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules")
include(EnableCXX11)

include_directories(include)

include_directories(
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)



catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES velocity_planner
 CATKIN_DEPENDS roscpp rospy std_msgs # message_runtime
#  DEPENDS system_lib
)


add_executable(pololu_driver_node
                      src/pololu_node.cpp
                      src/pololu.cpp
#                      include/psoc.h
                      src/conn_interface.cpp
#                      include/conn_interface.h
                      src/conn_serial.cpp
#                      include/conn_serial.h
#                      include/conn_msgbuffer.h
                      )

add_dependencies(pololu_driver_node rover_msgs_generate_messages_cpp)

target_link_libraries(pololu_driver_node ${Boost_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} ${catkin_LIBRARIES})
