cmake_minimum_required(VERSION 3.5)

project(lqr_optimizer)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_library(kinematics_trajectory src/kinematics_trajectory.cc src/kinematics_trajectory.h)

add_executable(kinematics_trajectory_demo src/kinematics_trajectory_demo.cc)
ament_target_dependencies(kinematics_trajectory_demo rclcpp std_msgs)

install(TARGETS
  kinematics_trajectory
  kinematics_trajectory_demo
  DESTINATION lib/${PROJECT_NAME})