cmake_minimum_required(VERSION 3.16)
project(amr_simulation)

set(CMAKE_CXX_STANDARD 20)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch worlds gazebo tools
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_gazebo_sim_launch.py
    TIMEOUT 30
    PYTHON_EXECUTABLE python3
  )
  add_launch_test(test/test_rsp_gazebo_integration.py
    TIMEOUT 120
    PYTHON_EXECUTABLE python3
  )

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    test_office_world test/test_office_world.py
    TIMEOUT 60
  )

endif()

ament_package()
