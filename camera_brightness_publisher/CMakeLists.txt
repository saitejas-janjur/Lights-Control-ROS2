cmake_minimum_required(VERSION 3.8)
project(camera_brightness_publisher)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Mark the camera_brightness_publisher/ directory as a Python package
ament_python_install_package(${PROJECT_NAME})

# Install your script(s)
install(
  PROGRAMS
    scripts/camera_brightness_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
