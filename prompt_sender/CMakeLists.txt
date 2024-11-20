cmake_minimum_required(VERSION 3.5)
project(prompt_sender)

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

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

# Install directories
install(
  DIRECTORY
    scripts
    launch
  DESTINATION
    share/${PROJECT_NAME}/
)

# Install individual scripts and launch files as executables
install(
  PROGRAMS
    scripts/prompt_sender.py
    launch/prompt_sender_sim.launch.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
