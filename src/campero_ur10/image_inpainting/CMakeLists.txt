cmake_minimum_required(VERSION 3.0.2)
project(image_inpainting)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


find_package(Eigen3 REQUIRED)
# find_package(PCL 1.3 REQUIRED COMPONENTS common io)

# Eigen 3.2 (Wily) only provides EIGEN3_INCLUDE_DIR, not EIGEN3_INCLUDE_DIRS
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  campero_ur10_msgs
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  tf
)

find_package(OpenCV REQUIRED)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES image_inpainting
   CATKIN_DEPENDS campero_ur10_msgs
                  cv_bridge
                  image_transport
                  roscpp
                  sensor_msgs
                  std_msgs
                  tf
    DEPENDS
                  EIGEN3
#  DEPENDS system_lib
)

include_directories(SYSTEM include ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_executable(image_inpainting src/image_inpainting.cpp)
add_dependencies(image_inpainting campero_ur10_msgs_generate_messages_cpp)
target_link_libraries(image_inpainting ${OpenCV_LIBS} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS image_inpainting DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

