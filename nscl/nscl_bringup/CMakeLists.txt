################################################################################
# CMake
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(nscl_bringup)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

################################################################################
# Packages
################################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  diagnostic_msgs
  sensor_msgs
  nscl_msgs
)


################################################################################
# Catkin specific configuration
################################################################################
catkin_package(
  LIBRARIES nscl_bringup
  CATKIN_DEPENDS roscpp std_msgs diagnostic_msgs sensor_msgs nscl_msgs
)

################################################################################
# Build
################################################################################
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(nscl_diagnostics src/nscl_diagnostics.cpp)
target_link_libraries(nscl_diagnostics ${catkin_LIBRARIES})
add_dependencies(nscl_diagnostics ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

################################################################################
# Install
################################################################################
install(TARGETS nscl_diagnostics
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS scripts/create_udev_rules
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES 99-nscl-cdc.rules
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
################################################################################
# Test
################################################################################
