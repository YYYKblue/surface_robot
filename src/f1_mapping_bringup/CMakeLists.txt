cmake_minimum_required(VERSION 3.8)
project(f1_mapping_bringup)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})
install(DIRECTORY launch config rviz DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/scan_check DESTINATION lib/${PROJECT_NAME})
install(FILES README.md README_LIDAR.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(configuration test/test_configuration.py)
  ament_add_pytest_test(lidar test/test_lidar.py)
endif()

ament_package()
