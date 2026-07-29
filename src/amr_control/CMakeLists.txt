cmake_minimum_required(VERSION 3.8)
project(amr_control)

set(CMAKE_CXX_STANDARD 20)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_diff_drive_controller_yaml
    test/test_diff_drive_controller_yaml.py
  )
endif()

ament_package()
