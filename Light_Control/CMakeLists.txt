cmake_minimum_required(VERSION 3.8)
project(bluerov2_control)

# Default compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# If needed by your Python script
find_package(rclpy REQUIRED)
find_package(std_srvs REQUIRED)
find_package(mavros_msgs REQUIRED)

#
# 1) Generate the .srv interfaces
#
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetLight.srv"  # relative path from 'srv' folder
  DEPENDENCIES builtin_interfaces
)

#
# 2) Export the interface runtime
#
ament_export_dependencies(rosidl_default_runtime)

#
# 3) Install scripts (Python nodes, etc.)
#
install(PROGRAMS
  scripts/light_service_node.py
  scripts/fade_lights.py
  scripts/auto_light_adjust.py
  DESTINATION lib/${PROJECT_NAME}
)

#
# 4) Install any launch files
#
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
