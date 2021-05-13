cmake_minimum_required(VERSION 3.0.2)
project(image_inpainting)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

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
#  DEPENDS system_lib
)

include_directories(SYSTEM include ${Boost_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_executable(image_inpainting src/image_inpainting.cpp)
target_link_libraries(image_inpainting ${OpenCV_LIBS} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS image_inpainting DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

