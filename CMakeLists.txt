cmake_minimum_required(VERSION 3.5)
project(vins_fusion_ros2)

set(CMAKE_CXX_STANDARD 14)
add_compile_options(-Wextra -Wpedantic)

set(CMAKE_BUILD_TYPE Release)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcpputils REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(message_filters REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)


include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${OpenCV_INCLUDE_DIRS}
)

add_subdirectory(vins)
add_executable(vins_node ros2_src/vins_estimator.cpp ros2_src/vins_estimator_node.cpp ros2_src/visualization.cpp)
ament_target_dependencies(vins_node rclcpp rcpputils message_filters std_msgs sensor_msgs visualization_msgs geometry_msgs nav_msgs tf2 tf2_ros)
target_link_libraries(vins_node vins_lib ${OpenCV_LIBS})

add_executable(stereo_image_splitter ros2_src/stereo_image_splitter.cpp)
ament_target_dependencies(stereo_image_splitter rclcpp std_msgs sensor_msgs)
target_link_libraries(stereo_image_splitter ${OpenCV_LIBS})

# Install launch files
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# Install nodes
install(
  TARGETS vins_node stereo_image_splitter
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
