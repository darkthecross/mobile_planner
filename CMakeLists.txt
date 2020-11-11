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

add_library(util_lib src/util.cc src/util.h)
target_link_libraries(util_lib proto ${PROTOBUF_LIBRARY} ${OpenCV_LIBS})

add_library(kinematics_lib src/kinematics.cc src/kinematics.h)
target_link_libraries(kinematics_lib proto ${PROTOBUF_LIBRARY} util_lib)

add_executable(kinematics_trajectory_demo src/kinematics_trajectory_demo.cc)
target_link_libraries(kinematics_trajectory_demo kinematics_lib ${OpenCV_LIBS} )

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})
add_library(trajectory_follower_lib src/trajectory_follower.cc src/trajectory_follower.h)
target_link_libraries(trajectory_follower_lib kinematics_lib)

add_executable(trajectory_follower_demo src/trajectory_follower_demo.cc)
target_link_libraries(trajectory_follower_demo trajectory_follower_lib proto ${PROTOBUF_LIBRARY})

add_library(trajectory_potentials_lib src/trajectory_potentials.cc src/trajectory_potentials.h)
target_link_libraries(trajectory_potentials_lib kinematics_lib)

add_executable(object_repeller_demo src/object_repeller_demo.cc)
target_link_libraries(object_repeller_demo trajectory_potentials_lib)

add_executable(trajectory_potentials_demo src/trajectory_potentials_demo.cc)
target_link_libraries(trajectory_potentials_demo trajectory_potentials_lib ${OpenCV_LIBS})
