cmake_minimum_required(VERSION 3.5)

project(mobile_planner)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

#####################################################################################

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

# Add protos.
ADD_SUBDIRECTORY(src/proto)
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR})

add_library(kinematics_trajectory_lib src/kinematics_trajectory.cc src/kinematics_trajectory.h)
target_link_libraries(kinematics_trajectory_lib proto ${PROTOBUF_LIBRARY})

add_executable(kinematics_trajectory_demo src/kinematics_trajectory_demo.cc)
target_link_libraries(kinematics_trajectory_demo kinematics_trajectory_lib ${OpenCV_LIBS} )

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})
add_library(trajectory_solver_lib src/trajectory_solver.cc src/trajectory_solver.h)
target_link_libraries(trajectory_solver_lib kinematics_trajectory_lib)

add_executable(trajectory_solver_demo src/trajectory_solver_demo.cc)
target_link_libraries(trajectory_solver_demo trajectory_solver_lib proto ${PROTOBUF_LIBRARY})