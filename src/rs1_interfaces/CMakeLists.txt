cmake_minimum_required(VERSION 3.8)
project(rs1_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FireObservation.msg"
  "msg/MissionOwnerStatus.msg"
  "srv/StartMission.srv"
  "srv/StopMission.srv"
  "srv/EnableManualNav.srv"
  "srv/DisableManualNav.srv"
  "srv/EnableTeleop.srv"
  "srv/DisableTeleop.srv"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
