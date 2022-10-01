cmake_minimum_required(VERSION 3.0.2)
project(vision)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
cv_bridge
image_transport
message_generation
message_runtime
roscpp
rospy
sensor_msgs
std_msgs
std_srvs
master
)
find_package(OpenCV REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES vision
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(vision_capture src/vision_capture.cpp)
add_dependencies(vision_capture ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(vision_capture ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} master_lib)

add_executable(vision_main src/vision_main.cpp)
add_dependencies(vision_main ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(vision_main ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} master_lib)