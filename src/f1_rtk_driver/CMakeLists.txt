cmake_minimum_required(VERSION 3.8)
project(f1_rtk_driver)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Heading.msg" "msg/RtkStatus.msg" "msg/RawSentence.msg"
  DEPENDENCIES std_msgs)
ament_export_dependencies(rosidl_default_runtime)
# Keep driver code separate from the generated f1_rtk_driver.msg module.
ament_python_install_package(wit_rtk)
install(PROGRAMS scripts/rtk_driver DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(nmea test/test_nmea.py)
  ament_add_pytest_test(alignment test/test_alignment.py)
  ament_add_pytest_test(discovery test/test_discovery.py)
  ament_add_pytest_test(serial_launch test/test_serial_launch.py TIMEOUT 60)
endif()
ament_package()
