cmake_minimum_required(VERSION 3.8)
project(f1_operator_gateway)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(operator_gateway)
install(PROGRAMS scripts/f1_operator_gateway
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(protocol test/test_protocol.py)
endif()

ament_package()
