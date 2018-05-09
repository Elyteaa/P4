cmake_minimum_required(VERSION 2.8.3)
project(beginner_tutorials)

## Find catkin and any catkin packagescc
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs sensor_msgs message_generation cv_bridge image_transport)

## Declare ROS messages and services
add_message_files(FILES Num.msg)
add_message_files(FILES thermalRect.msg)
add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs sensor_msgs)

## Declare a catkin package
catkin_package(
CATKIN_DEPENDS message_runtime
)

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

#add_executable(talker src/talker.cpp)
#target_link_libraries(talker ${catkin_LIBRARIES})
#add_dependencies(talker beginner_tutorials_generate_messages_cpp)

#add_executable(listener src/listener.cpp)
#target_link_libraries(listener ${catkin_LIBRARIES})
#add_dependencies(listener beginner_lstutorials_generate_messages_cpp)

add_executable(turtlebot_move src/hello_turtle.cpp)
target_link_libraries(turtlebot_move ${catkin_LIBRARIES})

add_executable(asus_node src/asus.cpp)
target_link_libraries(asus_node ${catkin_LIBRARIES})
add_dependencies(asus_node beginner_tutorials_generate_messages_cpp)

add_executable(thermal_node src/thermal.cpp)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(thermal_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(thermal_node beginner_tutorials_generate_message_cpp)

add_executable(rgb_cam_node src/RGB.cpp)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(rgb_cam_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
