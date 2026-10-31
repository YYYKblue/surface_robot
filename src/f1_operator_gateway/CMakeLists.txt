cmake_minimum_required(VERSION 3.8)
project(f1_operator_gateway)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(operator_gateway)
install(PROGRAMS scripts/f1_operator_gateway
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch deploy DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(protocol test/test_protocol.py)
  ament_add_pytest_test(map_store test/test_map_store.py)
  ament_add_pytest_test(runtime test/test_runtime.py)
  ament_add_pytest_test(runtime_ros test/test_runtime_ros.py TIMEOUT 180)
  ament_add_pytest_test(node_intent test/test_node_intent.py)
  ament_add_pytest_test(task_control test/test_task_control.py)
  ament_add_pytest_test(task_ros test/test_task_ros.py TIMEOUT 30)
  ament_add_pytest_test(bringup test/test_bringup.py TIMEOUT 60)
  ament_add_pytest_test(clock_gate test/test_clock_gate.py TIMEOUT 15)
endif()

ament_package()
