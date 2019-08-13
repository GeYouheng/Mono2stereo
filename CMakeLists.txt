cmake_minimum_required(VERSION 2.8.3)
project(mono2stereo)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg image_transport cv_bridge)
find_package(OpenCV)

## Declare ROS messages and services
#add_message_files(FILES Num.msg)
#add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services
#generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(
/opt/ros/melodic/include
${OpenCV_INCLUDE_DIRS}
)

set(LIBS 
${OpenCV_LIBS} 
)

add_executable(converter src/converter.cpp)
target_link_libraries(converter ${catkin_LIBRARIES})
target_link_libraries(converter ${LIBS})
add_dependencies(converter mono2stereo_generate_messages_cpp)




