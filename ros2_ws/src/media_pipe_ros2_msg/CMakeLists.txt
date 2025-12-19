cmake_minimum_required(VERSION 3.8)
project(media_pipe_ros2_msg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/HandPoint.msg"
  "msg/MediaPipeHumanHand.msg"
  "msg/MediaPipeHumanHandList.msg"
  "msg/PosePoint.msg"
  "msg/MediaPipeHumanPose.msg"
  "msg/MediaPipeHumanPoseList.msg"
  DEPENDENCIES std_msgs
)

ament_package()
