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
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)

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

install(TARGETS grasp_planner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS moveit_hello
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# ---- Unit tests ----
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_grasp_planner test/test_grasp_planner.cpp)
  target_link_libraries(test_grasp_planner grasp_planner)
endif()

ament_package()
