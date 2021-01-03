cmake_minimum_required(VERSION 3.0.2)
set(CMAKE_BUILD_TYPE Debug)
set (CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(ros_action_server)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  message_generation
  roscpp
  sensor_msgs
  std_msgs
)

find_package(orocos_kdl REQUIRED)
IF (orocos_kdl_FOUND)
    message(STATUS "[AS] orocos_kdl found. Version: ${orocos_kdl_VERSION}")
    include_directories(BEFORE SYSTEM ${orocos_kdl_INCLUDE_DIRS})
    add_definitions(${orocos_kdl_CFLAGS})
    link_directories(${orocos_kdl_LIBRARY_DIRS})
ENDIF()

find_package(Boost REQUIRED COMPONENTS system)

find_package(Eigen3 REQUIRED)
IF (Eigen3_FOUND)
    message(STATUS "[AS] Eigen3 found. Version: ${Eigen3_VERSION}")
    include_directories(BEFORE SYSTEM ${Eigen3_INCLUDE_DIRS})
    add_definitions(${Eigen3_CFLAGS})
    link_directories(${Eigen3_LIBRARY_DIRS})
ENDIF()
link_directories(/usr/local/include)

## Generate actions in the 'action' folder
add_action_files(
   FILES
   MyMsg.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   actionlib_msgs sensor_msgs std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ros_action_server
   LIBRARIES  ${orocos_kdl_LIBRARIES}
#  CATKIN_DEPENDS actionlib actionlib_msgs message_generation roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/action_server.cpp)
##add_executable(${PROJECT_NAME}_node src/ros_action_server_node.cpp)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
   ${orocos_kdl_LIBRARIES}
   Eigen3::Eigen)
