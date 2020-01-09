cmake_minimum_required(VERSION 2.8.3)
project(ray_casting)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
  DEPENDS
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
  ${ReactPhysics3D_INCLUDE_DIRS}
  /home/khaledalali/reactphysics3d/src/
)

add_executable(${PROJECT_NAME}_node src/mytest.cpp)

add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} libreactphysics3d.a)
