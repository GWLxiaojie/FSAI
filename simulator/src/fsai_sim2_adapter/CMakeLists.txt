cmake_minimum_required(VERSION 3.8)
project(fsai_sim2_adapter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(eufs_sim2 REQUIRED)
find_package(fsai_interfaces REQUIRED)
find_package(fsai_sim_core REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(vehicle_models REQUIRED)
find_package(yaml-cpp REQUIRED)
if(NOT TARGET jsoncpp_lib)
  find_package(jsoncpp REQUIRED)
endif()
set(fsai_ros_dependencies
  ackermann_msgs eufs_msgs geometry_msgs nav_msgs sensor_msgs std_msgs
  std_srvs tf2_ros visualization_msgs
)
foreach(dependency IN LISTS fsai_ros_dependencies)
  find_package(${dependency} REQUIRED)
endforeach()

add_library(${PROJECT_NAME}
  src/core_factory.cpp
  src/fsai_core_adapter.cpp
  src/plugin_registry.cpp
  src/physical_runtime.cpp
  src/reference_driver.cpp
  src/reference_runtime.cpp
  src/runtime_config.cpp
  src/simulation_context.cpp
  src/simulation_node.cpp
  src/simulation_runner.cpp
  src/vehicle_profile_loader.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME} PUBLIC
  eufs_sim2
  fsai_interfaces
  fsai_sim_core
  rclcpp
  rosgraph_msgs
  vehicle_models
  ${fsai_ros_dependencies}
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  fsai_sim_core::fsai_sim_core
  yaml-cpp
  jsoncpp_lib
)
target_compile_definitions(${PROJECT_NAME} PUBLIC EIGEN_INITIALIZE_MATRICES_BY_ZERO)

add_executable(fsai_simulation_node
  src/main.cpp
)
target_link_libraries(fsai_simulation_node
  ${PROJECT_NAME}
)
ament_target_dependencies(fsai_simulation_node
  rclcpp
)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS ${PROJECT_NAME} fsai_simulation_node
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(eufs_sim2 fsai_interfaces fsai_sim_core rclcpp rosgraph_msgs vehicle_models yaml-cpp jsoncpp ${fsai_ros_dependencies})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(eufs_core_composition_test
    test/eufs_core_composition_test.cpp
  )
  ament_target_dependencies(eufs_core_composition_test
    eufs_sim2
    vehicle_models
  )

  ament_add_gtest(core_factory_test
    test/core_factory_test.cpp
  )
  target_link_libraries(core_factory_test
    ${PROJECT_NAME}
  )

  ament_add_gtest(plugin_registry_test
    test/plugin_registry_test.cpp
  )
  target_link_libraries(plugin_registry_test
    ${PROJECT_NAME}
  )

  ament_add_gtest(simulation_runner_test
    test/simulation_runner_test.cpp
  )
  target_link_libraries(simulation_runner_test
    ${PROJECT_NAME}
  )
  foreach(test_name fsai_core_adapter_test runtime_config_test simulation_node_test vehicle_profile_loader_test reference_driver_test)
    ament_add_gtest(${test_name} test/${test_name}.cpp)
    target_link_libraries(${test_name} ${PROJECT_NAME})
  endforeach()
endif()

ament_package()
