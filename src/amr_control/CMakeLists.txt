cmake_minimum_required(VERSION 3.8)
project(amr_control)

set(CMAKE_CXX_STANDARD 20)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

endif()

ament_package()
