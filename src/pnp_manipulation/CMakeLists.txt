cmake_minimum_required(VERSION 3.8)
project(pnp_manipulation LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pnp_interfaces REQUIRED)

# ---- ROS-free grasp geometry library ----
add_library(grasp_planner src/grasp_planner.cpp)
set_target_properties(grasp_planner PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(grasp_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(grasp_planner PUBLIC Eigen3::Eigen)

# ---- First MoveIt node (Step 11) ----
add_executable(moveit_hello src/moveit_hello.cpp)
ament_target_dependencies(moveit_hello rclcpp moveit_ros_planning_interface geometry_msgs)

# ---- PickPlace action server ----
add_executable(pick_place_server src/pick_place_server.cpp)
target_link_libraries(pick_place_server grasp_planner)
ament_target_dependencies(pick_place_server
  rclcpp rclcpp_action moveit_ros_planning_interface geometry_msgs std_msgs
  shape_msgs moveit_msgs tf2_eigen tf2_geometry_msgs pnp_interfaces)

install(TARGETS grasp_planner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS moveit_hello pick_place_server
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

# ---- Unit tests ----
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_grasp_planner test/test_grasp_planner.cpp)
  target_link_libraries(test_grasp_planner grasp_planner)
endif()

ament_package()
