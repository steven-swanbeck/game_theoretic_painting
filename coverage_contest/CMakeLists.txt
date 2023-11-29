cmake_minimum_required(VERSION 3.0.2)
project(coverage_contest)

find_package(catkin REQUIRED COMPONENTS
    std_msgs
    geometry_msgs
    sensor_msgs
    message_generation
    pcl_ros
    pcl_conversions
)

catkin_package(
    INCLUDE_DIRS include
)

add_library(game_manager src/game_manager.cpp)
add_library(mcts src/mcts.cpp)
add_library(game_visualizer src/game_visualizer.cpp)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

find_package(PCL 1.2 REQUIRED)

add_executable(play_game src/play_game.cpp)
add_dependencies(
    play_game
    ${catkin_EXPORTED_TARGETS}
    ${pcl_tools_EXPORTED_TARGETS}
)
target_link_libraries(
    play_game
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    game_manager
    mcts
    game_visualizer
)

