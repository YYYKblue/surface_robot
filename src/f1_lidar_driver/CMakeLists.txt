cmake_minimum_required(VERSION 3.8)
project(f1_lidar_driver)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Threads REQUIRED)

# Internal, source-built SDK. No downloaded libraries or ros2_ws build products.
add_library(pavo2_sdk STATIC
  vendor/pavo2_sdk/src/pavo2_net.cpp
  vendor/pavo2_sdk/src/pavo2_parser.cpp
  vendor/pavo2_sdk/src/pavo2_driver.cpp
  vendor/pavo2_sdk/src/pavo2_filter.cpp
  vendor/pavo2_sdk/src/LdrAlgorithm.cpp)
target_include_directories(pavo2_sdk PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/vendor/pavo2_sdk/include)
target_link_libraries(pavo2_sdk PUBLIC Threads::Threads)

add_executable(pavo2_scan_node src/pavo2_scan_node.cpp src/pavo2_utils.cpp)
target_include_directories(pavo2_scan_node PRIVATE include/f1_lidar_driver)
target_link_libraries(pavo2_scan_node pavo2_sdk)
ament_target_dependencies(pavo2_scan_node rclcpp sensor_msgs)

add_executable(pavo2_fov_marker_node src/pavo2_fov_marker_node.cpp)
ament_target_dependencies(pavo2_fov_marker_node rclcpp geometry_msgs visualization_msgs)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(pavo2_scan_node PRIVATE -Wall -Wextra -Wpedantic)
  target_compile_options(pavo2_fov_marker_node PRIVATE -Wall -Wextra -Wpedantic)
endif()

install(TARGETS pavo2_scan_node pavo2_fov_marker_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config rviz deploy DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE PATTERN "*.pyc" EXCLUDE)
install(FILES README.md DESTINATION share/${PROJECT_NAME})
install(FILES vendor/Readme.vendor vendor/PROVENANCE.md vendor/LICENSE.angles
  vendor/package.xml.upstream vendor/source_manifest.json
  DESTINATION share/${PROJECT_NAME}/vendor)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  set(lidar_test_env
    "ROS_LOCALHOST_ONLY=1" "ROS_DOMAIN_ID=178"
    "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
    "CYCLONEDDS_URI=file://${CMAKE_CURRENT_SOURCE_DIR}/test/cyclonedds_localhost.xml")
  ament_add_gtest(scan_conversion test/test_scan_conversion.cpp src/pavo2_utils.cpp
    TIMEOUT 15 ENV ${lidar_test_env})
  target_include_directories(scan_conversion PRIVATE include/f1_lidar_driver)
  target_link_libraries(scan_conversion pavo2_sdk)
  ament_target_dependencies(scan_conversion rclcpp sensor_msgs)
  ament_add_pytest_test(launch_config test/test_launch.py TIMEOUT 30 ENV ${lidar_test_env})
  ament_add_pytest_test(startup_probe test/test_startup_probe.py TIMEOUT 10)
endif()

ament_package()
