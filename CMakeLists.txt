cmake_minimum_required(VERSION 2.8.3)
project(uav_command)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  quadrotor_msgs
  tf
  vision_server
  mavros_msgs
  #object_all
)

find_package(Eigen3 REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES uav_command
   CATKIN_DEPENDS geometry_msgs roscpp std_msgs quadrotor_msgs tf  mavros_msgs#object_all
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS} 
  ${PROJECT_SOURCE_DIR}/include
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/learning_topic.cpp
# )

# )

add_executable(uav_command 
                src/uav_command.cpp
                src/get_data.cpp
                src/uav_task.cpp)

target_link_libraries(uav_command ${catkin_LIBRARIES})
add_dependencies(uav_command vision_server_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

