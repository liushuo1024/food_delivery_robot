cmake_minimum_required(VERSION 3.0.2)
project(robot_bringup)

find_package(catkin REQUIRED COMPONENTS
geometry_msgs
roscpp
rospy
sensor_msgs
tf
)
catkin_package(
  INCLUDE_DIRS include
)
include_directories(
include
 ${catkin_INCLUDE_DIRS}
)
add_executable(robot_bringup
              src/robot_bringup/robot_bringup.cpp
              src/robot_bringup/robot.cpp
              src/robot_bringup/mbot_linux_serial.cpp)
             
target_link_libraries(robot_bringup ${catkin_LIBRARIES})

add_executable(disable_bringup
              src/disable_bringup/robot_bringup.cpp
              src/disable_bringup/robot.cpp
              src/disable_bringup/mbot_linux_serial.cpp)
             
target_link_libraries(disable_bringup ${catkin_LIBRARIES})

add_executable(without_imu_bringup
              src/without_imu_bringup/robot_bringup.cpp
              src/without_imu_bringup/robot.cpp
              src/without_imu_bringup/mbot_linux_serial.cpp)
             
target_link_libraries(without_imu_bringup ${catkin_LIBRARIES})
