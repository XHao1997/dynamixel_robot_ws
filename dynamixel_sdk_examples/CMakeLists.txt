cmake_minimum_required(VERSION 3.5)
project(dynamixel_sdk_examples)

# C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(dynamixel_sdk_custom_interfaces REQUIRED)
find_package(generate_parameter_library REQUIRED)

# Generate parameter library (namespace = package name for nice include path)
generate_parameter_library(
  motors_parameters
  config/motors.yaml
  NAMESPACE ${PROJECT_NAME}
)

# Executables
add_executable(read_write_node src/read_write_node.cpp)
add_executable(test_parameter_load src/test_param.cpp)
target_include_directories(test_parameter_load PRIVATE include)
target_include_directories(read_write_node PRIVATE include)
# ROS package deps go here:
ament_target_dependencies(read_write_node
  rclcpp
  sensor_msgs
  dynamixel_sdk
  dynamixel_sdk_custom_interfaces
)
ament_target_dependencies(test_parameter_load
  rclcpp
)

# Link your generated param library here:
target_link_libraries(read_write_node motors_parameters)
target_link_libraries(test_parameter_load motors_parameters)

# (Optional) Some distros export dynamixel_sdk::dynamixel_sdk — enable if needed:
# target_link_libraries(read_write_node PRIVATE dynamixel_sdk::dynamixel_sdk)

# # Install executables
install(TARGETS
  read_write_node
  test_parameter_load
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# # Install the generated parameter library (static/shared)
install(TARGETS motors_parameters
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin  # for Windows
)

install(TARGETS read_write_node test_parameter_load motors_parameters
  EXPORT ${PROJECT_NAME}Targets)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)

ament_package()
