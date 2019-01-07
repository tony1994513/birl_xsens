cmake_minimum_required(VERSION 2.8.3)
project(birl_xsens)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  tf
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES birl_xsens
 CATKIN_DEPENDS roscpp tf geometry_msgs
#  DEPENDS system_lib
)

include_directories(
# include
${catkin_INCLUDE_DIRS}
)

add_executable(xsens_mvn_driver src/xsens_mvn_driver.cpp ${catkin_INCLUDE_DIRS})

target_link_libraries(xsens_mvn_driver ${catkin_LIBRARIES})