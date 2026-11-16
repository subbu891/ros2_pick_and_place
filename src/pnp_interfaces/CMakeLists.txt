cmake_minimum_required(VERSION 3.8)
project(pnp_interfaces)

# Tools to build a ROS 2 package
find_package(ament_cmake REQUIRED)

# The code generator that turns .msg/.action files into C++ and Python code
find_package(rosidl_default_generators REQUIRED)

# Message packages our interfaces use (PoseStamped and Header)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Generate code for our interface files
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DetectedObject.msg"
  "msg/DetectedObjectArray.msg"
  "action/PickPlace.action"
  DEPENDENCIES geometry_msgs std_msgs
)

# Packages that use ours will also need the runtime part
ament_export_dependencies(rosidl_default_runtime)

ament_package()
