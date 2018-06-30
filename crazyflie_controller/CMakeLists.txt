cmake_minimum_required(VERSION 2.8.3)
project(crazyflie_controller)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  tf
)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES crazyflie_controller
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_definitions(-DVICON_OFF)
add_definitions(-DERROR_INPUT)
add_executable(crazyflie_controller
  src/controller.cpp)

target_link_libraries(crazyflie_controller
  ${catkin_LIBRARIES}
)

