cmake_minimum_required(VERSION 3.5)
project(example_package)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find Dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

# Find Messages Dependencies
find_package(std_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# Find Messages Generation Dependencies
find_package(rosidl_default_generators REQUIRED)

# Set Cusom Messages
set(msg_files
  "msg/ExampleMsg.msg"
  "msg/ComposedMsg.msg"
)

# Set Custom Services
set(srv_files
  "srv/ExampleSrv.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES std_msgs control_msgs trajectory_msgs std_srvs
)

# Include C++ Directories
include_directories(include)

# Create C++ Executables
# add_executable(cpp_executable src/cpp_executable.cpp)
# ament_target_dependencies(cpp_executable rclcpp std_msgs)

# Install C++ Executables
install(TARGETS
  # cpp_executable
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python Modules
ament_python_install_package(${PROJECT_NAME})

# Install Python Scripts
install(PROGRAMS
  scripts/example.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  #set(ament_cmake_copyright_FOUND TRUE)
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
