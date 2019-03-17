################################################################################
# CMake
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(zetabank_example)

################################################################################
# Packages
################################################################################
find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  geometry_msgs
  sensor_msgs
  actionlib
  actionlib_msgs
  message_generation
  visualization_msgs
  interactive_markers
)

find_package(Boost REQUIRED COMPONENTS
    system
    thread
)

################################################################################
# Declare ROS messages, services and actions
################################################################################
add_action_files(FILES 
    zetabank.action
)

generate_messages(DEPENDENCIES
    std_msgs
    actionlib_msgs
    geometry_msgs  
)

################################################################################
# Catkin specific configuration
################################################################################
catkin_package(
  LIBRARIES zetabank_patrol
  CATKIN_DEPENDS
  std_msgs
  message_runtime
  rospy 
  actionlib
  actionlib_msgs
  geometry_msgs
  DEPENDS Boost
)
################################################################################
# Build
################################################################################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}${Boost_INCLUDE_DIRS}
)

add_executable(zetabank_marker_server src/zetabank_marker_server.cpp)
target_link_libraries(zetabank_marker_server ${catkin_LIBRARIES})

################################################################################
# Install
################################################################################
install(PROGRAMS 
   scripts/zetabank_pointop_key.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS
  scripts/zetabank_client.py
  scripts/zetabank_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
  scripts/zetabank_client.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS zetabank_marker_server 
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch 
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
################################################################################
# Test
################################################################################
