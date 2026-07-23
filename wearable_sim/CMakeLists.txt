cmake_minimum_required(VERSION 3.8)
project(wearable_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Simulation / launch dependencies
find_package(ros_gz_sim REQUIRED)
find_package(ros_gz_bridge REQUIRED)
find_package(robot_state_publisher REQUIRED)

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

# Install launch files so ros2 launch can find them
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install URDF files so they are found via get_package_share_directory()
install(
  DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

# Install world SDF files (wearable_world.sdf contains the Sensors system plugin)
install(
  DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)

# Install configuration files (SLAM params, etc.)
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install RViz configuration
install(
  DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

# ── Python scripts (vision nodes) ────────────────────────────────────────────
# Makes each script in scripts/ executable via `ros2 run wearable_sim <name>`
install(PROGRAMS
  scripts/arrow_teleop.py
  scripts/kobuki_driver.py
  scripts/download_model.py
  scripts/vision_perception.py
  scripts/find_object.py
  scripts/scene_describer.py
  scripts/phone_camera.py
  DESTINATION lib/${PROJECT_NAME}
)

# ── YOLOv5m ONNX model ───────────────────────────────────────────────────────
# Install alongside the script so __file__-relative lookup works at runtime.
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/scripts/yolov5m.onnx")
  install(
    FILES
      scripts/yolov5m.onnx
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

ament_package()
