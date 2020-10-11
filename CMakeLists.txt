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

# Config OGRE path
set(CMAKE_MODULE_PATH "/usr/local/lib/OGRE/cmake/;${CMAKE_MODULE_PATH}")

# find dependencies
# specify which version and components you need
find_package(OGRE 1.12 REQUIRED COMPONENTS Bites RTShaderSystem)

include_directories(${OGRE_INCLUDE_DIRS})
link_directories(${OGRE_LIBRARY_DIRS})
 
# copy resource.cfg next to our binaries where OGRE looks for it
file(COPY ${OGRE_CONFIG_DIR}/resources.cfg DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
 
# add_library(kinematics_trajectory src/kinematics_trajectory.cc src/kinematics_trajectory.h)
add_library(visualizer src/visualizer.cc src/visualizer.h)
target_link_libraries(visualizer OgreBites OgreRTShaderSystem)

add_executable(kinematics_trajectory_demo src/kinematics_trajectory_demo.cc)
target_link_libraries(kinematics_trajectory_demo visualizer OgreBites OgreRTShaderSystem)
