cmake_minimum_required(VERSION 3.8)
project(uav_inspection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/VoxelGrid3D.msg"
  "msg/Path3D.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY
  worlds
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  planning/grid_map_builder.py
  planning/abc_path_planner.py
  planning/map_and_path_publishers.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
