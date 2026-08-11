cmake_minimum_required(VERSION 3.16)
project(amr_navigation)

set(CMAKE_CXX_STANDARD 20)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch maps
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/twist_to_stamped.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_navigation_launch.py
    TIMEOUT 30
    PYTHON_EXECUTABLE python3
  )

endif()

ament_package()
