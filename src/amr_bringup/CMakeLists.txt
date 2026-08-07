cmake_minimum_required(VERSION 3.8)
project(amr_bringup)

set(CMAKE_CXX_STANDARD 20)

find_package(ament_cmake REQUIRED)


install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_system_launch.py
    TIMEOUT 30
    PYTHON_EXECUTABLE python3
  )

endif()

ament_package()
