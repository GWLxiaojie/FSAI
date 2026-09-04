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
find_package(fsai_sim_core REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(vehicle_models REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(${PROJECT_NAME}
  src/core_factory.cpp
  src/fsai_core_adapter.cpp
  src/plugin_registry.cpp
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
  fsai_sim_core
  rclcpp
  rosgraph_msgs
  vehicle_models
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  fsai_sim_core
  yaml-cpp
)

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
ament_export_dependencies(eufs_sim2 fsai_sim_core rclcpp rosgraph_msgs vehicle_models yaml-cpp)

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
endif()

ament_package()
