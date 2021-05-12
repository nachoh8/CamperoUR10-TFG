cmake_minimum_required(VERSION 3.0.2)
project(campero_ur10_server)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


find_package(Eigen3 REQUIRED)

# Eigen 3.2 (Wily) only provides EIGEN3_INCLUDE_DIR, not EIGEN3_INCLUDE_DIRS
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

find_package(catkin REQUIRED
  COMPONENTS
    interactive_markers
    moveit_core
    moveit_visual_tools
    moveit_ros_planning
    moveit_ros_planning_interface
    pluginlib
    geometric_shapes
    pcl_ros
    pcl_conversions
    rosbag
    tf2_ros
    tf2_eigen
    tf2_geometry_msgs
    campero_ur10_msgs
)

find_package(Boost REQUIRED system filesystem date_time thread)

catkin_package(
  LIBRARIES
    c_ur10_def
  CATKIN_DEPENDS
    moveit_core
    moveit_visual_tools
    moveit_ros_planning_interface
    interactive_markers
  DEPENDS
    EIGEN3
    campero_ur10_msgs
)

include_directories(SYSTEM include ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_library(c_ur10_def src/c_ur10_utils.cpp src/campero_ur10.cpp)
target_link_libraries(c_ur10_def ${catkin_LIBRARIES})
add_dependencies(c_ur10_def campero_ur10_msgs_generate_messages_cpp)

#link_directories(${catkin_LIBRARY_DIRS})

add_executable(campero_ur10_server src/campero_ur10_server.cpp src/c_ur10_utils.cpp src/campero_ur10.cpp)
target_link_libraries(campero_ur10_server c_ur10_def ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS campero_ur10_server DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})



