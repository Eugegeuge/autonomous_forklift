cmake_minimum_required(VERSION 3.8)
project(autonomous_forklift)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mvsim REQUIRED)
find_package(geometry_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(simple_controller src/simple_controller.cpp)
ament_target_dependencies(simple_controller rclcpp geometry_msgs)

install(TARGETS
  simple_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  mvsim
  mvsim_models
  urdf
  config
  models
  worlds
  rviz
  maps
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/graph_visualizer.py
  scripts/waypoint_follower.py
  scripts/interface_node.py
  scripts/graph_editor.py
  scripts/lift_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  scripts/templates
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
