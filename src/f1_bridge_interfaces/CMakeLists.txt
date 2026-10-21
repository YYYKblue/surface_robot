cmake_minimum_required(VERSION 3.8)
project(f1_bridge_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ActuatorCommand.msg"
  "msg/ControlState.msg"
  "srv/AcquireControl.srv"
  "srv/ReleaseControl.srv"
  DEPENDENCIES builtin_interfaces)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
