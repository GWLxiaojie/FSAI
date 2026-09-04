cmake_minimum_required(VERSION 3.8)
project(fsai_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch vehicles tracks scenarios
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_upstream_compatibility.launch.py TIMEOUT 30)
endif()

ament_package()
