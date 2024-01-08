cmake_minimum_required(VERSION 3.8)
project(coverage_contest)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
# find_package(message_generation REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(visualization_msgs REQUIRED)

# TODO ros interfaces
# rosidl_generate_interfaces(${PROJECT_NAME}
#   DEPENDENCIES
# )

# rosidl_get_typesupport_target(cpp_typesupport_target
#   ${PROJECT_NAME} rosidl_typesupport_cpp)

include_directories(include)

add_library(game_visualizer src/game_visualizer.cpp)
ament_target_dependencies(game_visualizer rclcpp sensor_msgs visualization_msgs pcl_ros)
# ament_export_targets(game_visualizer HAS_LIBRARY_TARGET)

# install(
#   DIRECTORY
#     include/game_visualizer
#   DESTINATION include
# )

# install(
#   TARGETS
#     game_visualizer
#   EXPORT
#     game_visualizer
#   LIBRARY DESTINATION lib
#   ARCHIVE DESTINATION lib
#   RUNTIME DESTINATION bin
#   INCLUDES DESTINATION include
# )

add_executable(play_game src/play_game.cpp)
ament_target_dependencies(play_game rclcpp std_srvs)
target_compile_options(play_game PRIVATE -g)
target_link_libraries(play_game game_visualizer)

install(
  TARGETS
    play_game
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    config
    launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
