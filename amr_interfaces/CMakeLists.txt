cmake_minimum_required(VERSION 3.8)
project(amr_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(vision_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LlcStatus.msg"
  "msg/ArmStatus.msg"
  "msg/MissionState.msg"
  "srv/ArmCommand.srv"
  "srv/SetGripper.srv"
  "srv/ResetOdometry.srv"
  "srv/DetectObject.srv"
  "action/MoveArm.action"
  "action/PickObject.action"
  "action/PlaceObject.action"
  "action/ExecuteMission.action"
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs vision_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
