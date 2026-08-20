cmake_minimum_required(VERSION 3.8)
project(gps_denied_nav)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  gps_denied_nav/vo/vo_node.py
  gps_denied_nav/fusion/ekf_fusion_node.py
  gps_denied_nav/bridge/tf_broadcaster.py
  gps_denied_nav/bridge/mavros_vision_bridge.py
  gps_denied_nav/mapping/occupancy_grid_node.py
  gps_denied_nav/eval/diagnostics_node.py
  gps_denied_nav/eval/synthetic_data_generator.py
  gps_denied_nav/eval/trajectory_evaluator.py
  scripts/run_synthetic_demo.py
  scripts/calibrate_camera.py
  scripts/evaluate_trajectory.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch, config, and docs directories
install(DIRECTORY
  launch
  config
  docs
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
