cmake_minimum_required(VERSION 3.8)
project(lpl2_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/EvidenceItem.msg"
  "msg/EvidenceArray.msg"
  "msg/PermissionState.msg"
  "msg/AuthorityState.msg"
  "msg/OptionStatus.msg"
  "msg/DecisionSpace.msg"
  "msg/ExecutionState.msg"
  "msg/Escalation.msg"
  "msg/GateStatus.msg"
  "srv/GrantPermission.srv"
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
