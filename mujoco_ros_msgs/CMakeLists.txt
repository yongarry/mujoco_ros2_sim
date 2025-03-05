cmake_minimum_required(VERSION 3.5)
project(mujoco_ros_msgs)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Add message files
set(MSG_FILES
  "msg/ApplyForce.msg"
  "msg/JointInit.msg"
  "msg/JointSet.msg"
  "msg/JointState.msg"
  "msg/SensorState.msg"
  "msg/SensorBase.msg"
  "msg/SimstatusM2C.msg"
  "msg/SimStatus.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs
  ADD_LINTER_TESTS
)

# Export dependencies
ament_export_dependencies(rosidl_default_runtime)

# Package settings
ament_package()
