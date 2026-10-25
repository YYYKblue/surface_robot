cmake_minimum_required(VERSION 3.8)
project(f1_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ActuatorCommand.msg"
  "msg/ControlState.msg"
  "msg/WallObservation.msg"
  "msg/EffortPreview.msg"
  "msg/CleaningStatus.msg"
  "msg/EdgeTarget.msg"
  "srv/SetInitialPose.srv"
  "srv/AcquireControl.srv"
  "srv/ReleaseControl.srv"
  "srv/SetEdge.srv"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs)
ament_export_dependencies(rosidl_default_runtime)
install(FILES README.md DESTINATION share/${PROJECT_NAME})
ament_package()
