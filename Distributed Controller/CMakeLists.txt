cmake_minimum_required(VERSION 2.8.3)
project(crazyflie_coop)

find_package(catkin REQUIRED COMPONENTS 
	std_msgs
	tf
	geometry_msgs
)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES crazyflie_coop
#  CATKIN_DEPENDS
#  DEPENDS
#  INCLUDE_DIRS include
#  LIBRARIES crazyflie_coop
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)


include_directories(include)

add_definitions(-DVICON_OFF)
add_definitions(-DTEST_NO_CF)
#add_definitions(-DTEST_COMM)
add_definitions(-DIMPLEMENT_TRUST)
add_definitions(-DCALC_TRUST)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(crazyflieCoopLib
  src/CommSocket.cpp
  src/coop.cpp)

add_executable(crazyflie_coop
  src/crazyflie_coop.cpp)

target_link_libraries(crazyflie_coop
  crazyflieCoopLib
  ${catkin_LIBRARIES}
)
