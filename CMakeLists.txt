cmake_minimum_required(VERSION 2.8.3)
project(emotional_manager)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  message_generation
  tf
)

find_package(OpenCV REQUIRED)
include(/home/alexis/src/dlib-18.15/dlib/cmake)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate services in the 'srv' folder
#add_service_files(
#   FILES
#)

## Generate messages in the 'msg' folder
#add_message_files(
#   FILES
#)

## Generate added messages and services with any dependencies listed here
#generate_messages(
#   DEPENDENCIES
#   std_msgs
#   geometry_msgs
#)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 CATKIN_DEPENDS message_runtime 
)

# using c++11 :
#--------------
add_definitions(-std=c++11)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
 install(PROGRAMS
   nodes/action_manager.py
   nodes/emotional_manager.py
   nodes/valence_arousal_map.py
   nodes/space_tf_manager.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

install(DIRECTORY include/
   DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
 )
## Mark other files for installation (e.g. launch and bag files, etc.) 
 install(FILES
launch/nao_emotional.launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
 )

add_executable(vision src/vision.cpp)
target_link_libraries(vision dlib ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
install(TARGETS
   vision
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

