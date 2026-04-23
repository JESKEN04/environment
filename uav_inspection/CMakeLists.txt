cmake_minimum_required(VERSION 3.8)
project(uav_inspection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# Install directories
install(DIRECTORY
  worlds
  launch
  config
  rbf_training
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/build_voxel_map.py
  scripts/abc_path_planner.py
  scripts/ros2_grid_map_publisher.py
  scripts/ros2_path_publisher.py
  scripts/generate_thesis_figures.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
