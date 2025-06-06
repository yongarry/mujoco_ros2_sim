cmake_minimum_required(VERSION 3.5)
project(mujoco_ros)

add_compile_options(-Ofast -std=c++17)

# Find dependencies (ROS2 uses ament instead of catkin)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(mujoco_ros_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# If tocabi_lib is available, include it
find_package(tocabi_lib QUIET)
if(tocabi_lib_FOUND)
  message(" -- shm mode enabled")
  add_definitions(-DCOMPILE_SHAREDMEMORY)
endif()

# Set the MuJoCo path
set(MUJOCO_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../mujoco210)
message("MuJoCo path: ${MUJOCO_PATH}")

# Include directories
include_directories(
  include
  ${MUJOCO_PATH}/include
  # ${MUJOCO_PATH}/simulate
  ${EIGEN3_INCLUDE_DIRS}
)

# Link directories
link_directories(
  ${MUJOCO_PATH}/bin
)

# If tocabi_lib is found, include its directories
if(tocabi_lib_FOUND)
  include_directories(${tocabi_lib_INCLUDE_DIRS})
endif()

# Find the MuJoCo library
file(GLOB LIB_MUJOCO ${MUJOCO_PATH}/bin/libmujoco210.so)

# Find UI source file
# file(GLOB UI_SRC "${MUJOCO_PATH}/simulate/simulate.cc")
file(GLOB UI_SRC "${MUJOCO_PATH}/include/uitools.c")

# Define the library target
add_library(mujoco_ros_lib
  ${UI_SRC}
  src/mjros.cpp
)

# Define the executable target
add_executable(mujoco_ros src/main.cpp)

# Link libraries to the executable
target_link_libraries(mujoco_ros
  mujoco_ros_lib
  ${rclcpp_LIBRARIES}
  ${LIB_MUJOCO}
  GL
     
  ${MUJOCO_PATH}/bin/libglew.so
  ${MUJOCO_PATH}/bin/libglfw.so.3
)

# Dependencies between targets
ament_target_dependencies(mujoco_ros_lib
  rclcpp
  std_msgs
  mujoco_ros_msgs
  tf2
  tf2_geometry_msgs
)

ament_target_dependencies(mujoco_ros
  rclcpp
  std_msgs
  mujoco_ros_msgs
  tf2
  tf2_geometry_msgs
)

# Install targets (useful for ROS2 packages)
install(TARGETS
  mujoco_ros
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# Define your target properties for the executable:
set_target_properties(mujoco_ros PROPERTIES
  BUILD_RPATH "${MUJOCO_PATH}/bin"
  INSTALL_RPATH "${MUJOCO_PATH}/bin"
  SKIP_BUILD_RPATH FALSE
  BUILD_WITH_INSTALL_RPATH TRUE
)


# Install the package
ament_package()

