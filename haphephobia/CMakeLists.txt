cmake_minimum_required(VERSION 3.0.2)
project(haphephobia)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    std_msgs
    geometry_msgs
    sensor_msgs
    visualization_msgs
    message_generation
    pcl_ros
    pcl_conversions
    moveit_core
    moveit_ros_perception
    moveit_ros_planning_interface
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
add_service_files(
    FILES
    create_vfs.srv
    create_compliant_poses.srv
    generate_cartesian_plan.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    std_msgs geometry_msgs sensor_msgs visualization_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################


###################################
## catkin specific configuration ##
###################################
catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS message_runtime
)

###########
## Build ##
###########

include_directories(
    include 
    ${catkin_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
)

find_package(PCL 1.2 REQUIRED)

add_executable(vf_generator src/vf_generator.cpp)
add_dependencies(
    vf_generator
    ${catkin_EXPORTED_TARGETS}
    ${pcl_tools_EXPORTED_TARGETS}
)
target_link_libraries(
    vf_generator
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)

add_executable(pose_generator src/pose_generator.cpp)
add_dependencies(
    pose_generator
    ${catkin_EXPORTED_TARGETS}
    ${pcl_tools_EXPORTED_TARGETS}
)
target_link_libraries(
    pose_generator
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)

#############
## Install ##
#############

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
    PATTERN ".svn" EXCLUDE
)

#############
## Testing ##
#############
