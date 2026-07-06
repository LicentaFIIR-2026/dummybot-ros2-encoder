cmake_minimum_required(VERSION 3.8)
project(yolo26_cpp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS_RELEASE "-O3 -march=native -mtune=native -ffast-math -DNDEBUG")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# NCNN
find_package(ncnn REQUIRED)
if(ncnn_FOUND)
  message(STATUS "NCNN found: ${ncnn_DIR}")
else()
  message(FATAL_ERROR "NCNN not found!")
endif()

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# Detector library with NCNN
add_library(yolo26_detector SHARED
  src/yolo26_detector_ncnn.cpp
)

target_link_libraries(yolo26_detector
  ncnn
  ${OpenCV_LIBS}
)

ament_target_dependencies(yolo26_detector
  rclcpp
  OpenCV
)

# Lifecycle Node
add_executable(yolo26_node
  src/yolo26_node.cpp
)

target_link_libraries(yolo26_node
  yolo26_detector
  ncnn
  ${OpenCV_LIBS}
)

ament_target_dependencies(yolo26_node
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  vision_msgs
  cv_bridge
  image_transport
  lifecycle_msgs
  OpenCV
)

# Component
add_library(yolo26_component SHARED
  src/yolo26_component.cpp
)

target_link_libraries(yolo26_component
  yolo26_detector
  ncnn
  ${OpenCV_LIBS}
)

ament_target_dependencies(yolo26_component
  rclcpp
  rclcpp_components
  sensor_msgs
  vision_msgs
  cv_bridge
  image_transport
  OpenCV
)

rclcpp_components_register_node(yolo26_component
  PLUGIN "yolo26_cpp::Yolo26Component"
  EXECUTABLE yolo26_component_node
)

install(TARGETS
  yolo26_detector
  yolo26_node
  yolo26_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
