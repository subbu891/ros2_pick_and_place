cmake_minimum_required(VERSION 3.8)
project(pnp_perception LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pnp_interfaces REQUIRED)

# ---- ROS-free core library ----
add_library(color_detector src/color_detector.cpp)
set_target_properties(color_detector PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(color_detector PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(color_detector PUBLIC ${OpenCV_LIBS})

# ---- ROS 2 node ----
add_executable(object_detector src/object_detector_node.cpp)
target_link_libraries(object_detector color_detector)
ament_target_dependencies(object_detector
  rclcpp sensor_msgs geometry_msgs visualization_msgs cv_bridge
  message_filters tf2 tf2_ros tf2_geometry_msgs pnp_interfaces)

install(TARGETS color_detector
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS object_detector
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

# ---- Unit tests ----
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_color_detector test/test_color_detector.cpp)
  target_link_libraries(test_color_detector color_detector)
endif()

ament_package()
