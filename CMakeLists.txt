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
target_link_libraries(kinematics_trajectory_lib ${PROTO_SRCS} ${PROTO_HDRS})

add_executable(kinematics_trajectory_demo src/kinematics_trajectory_demo.cc)
target_link_libraries(kinematics_trajectory_demo kinematics_trajectory_lib ${OpenCV_LIBS})
