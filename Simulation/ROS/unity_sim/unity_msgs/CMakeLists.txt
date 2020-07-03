cmake_minimum_required(VERSION 2.8.3)
project(unity_msgs)
find_package(catkin REQUIRED COMPONENTS message_generation std_msgs roscpp)
add_message_files(
   FILES
   speed_control.msg
)
generate_messages(
   DEPENDENCIES
   std_msgs
)
catkin_package(
   CATKIN_DEPENDS message_runtime std_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

