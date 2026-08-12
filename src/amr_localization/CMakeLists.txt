cmake_minimum_required(VERSION 3.16)
project(amr_localization)

set(CMAKE_CXX_STANDARD 20)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/tf_restamp.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_localization_launch.py
    TIMEOUT 30
    PYTHON_EXECUTABLE python3
  )

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_tf_restamp test/test_tf_restamp.py)

endif()

ament_package()
