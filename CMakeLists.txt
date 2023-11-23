cmake_minimum_required(VERSION 3.5)
project(example_package)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find Dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclpy REQUIRED)

# Find Messages Dependencies
find_package(std_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# Find Messages Generation Dependencies
find_package(rosidl_default_generators REQUIRED)

# Eigen3 Dependencies
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

# MoveIt! Dependencies
# find_package(moveit_core REQUIRED)
# find_package(moveit_ros_planning_interface REQUIRED)

# Set Cusom Messages
set(msg_files
  "msg/ExampleMsg.msg"
  "msg/ComposedMsg.msg"
)

# Set Custom Services
set(srv_files
  "srv/ExampleSrv.srv"
)

# Set Custom Actions
set(action_files
  "action/ExampleAction.action"
)

# Generate Messages, Services and Actions
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES std_msgs control_msgs trajectory_msgs std_srvs
)

# Include C++ Directories
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${rclcpp_action_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${${PROJECT_NAME}_INCLUDE_DIRS}
  )

# Create C++ Executables
add_executable(global_parameter_server src/global_parameter_server.cpp)
ament_target_dependencies(global_parameter_server rclcpp)

add_executable(example_cpp_node src/example_node.cpp src/example.cpp)
ament_target_dependencies(example_cpp_node rclcpp rclcpp_action)
rosidl_target_interfaces(example_cpp_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Install C++ Executables
install(TARGETS
  global_parameter_server
  example_cpp_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python Modules
ament_python_install_package(${PROJECT_NAME})

# Install Python Scripts
install(PROGRAMS
  scripts/example.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install Launch Files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install Config Files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  #set(ament_cmake_copyright_FOUND TRUE)
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
