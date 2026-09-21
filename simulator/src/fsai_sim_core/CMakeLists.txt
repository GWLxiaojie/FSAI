cmake_minimum_required(VERSION 3.16)
project(fsai_sim_core LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(FSAI_SIM_CORE_STANDALONE "Build the ROS-free library and tests without ament" OFF)
find_package(Eigen3 REQUIRED)
find_package(OpenSSL REQUIRED)
if(NOT FSAI_SIM_CORE_STANDALONE)
  find_package(ament_cmake QUIET)
endif()

add_library(fsai_sim_core
  src/parameters.cpp
  src/safety_chain.cpp
  src/actuator_model.cpp
  src/bicycle_backend.cpp
  src/hybrid_integrator.cpp
  src/ground_truth_builder.cpp
  src/plant.cpp
)
target_include_directories(fsai_sim_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(fsai_sim_core PUBLIC Eigen3::Eigen OpenSSL::Crypto)
target_compile_definitions(fsai_sim_core PUBLIC EIGEN_INITIALIZE_MATRICES_BY_ZERO)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS fsai_sim_core
  EXPORT fsai_sim_coreTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

if(ament_cmake_FOUND AND NOT FSAI_SIM_CORE_STANDALONE)
  find_package(ament_cmake REQUIRED)
  ament_export_targets(fsai_sim_coreTargets HAS_LIBRARY_TARGET)
  ament_export_dependencies(Eigen3 OpenSSL)
  if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(types_test test/types_test.cpp)
    target_link_libraries(types_test fsai_sim_core)
    ament_add_gtest(parameters_test test/parameters_test.cpp)
    target_link_libraries(parameters_test fsai_sim_core)
    ament_add_gtest(safety_chain_test test/safety_chain_test.cpp)
    target_link_libraries(safety_chain_test fsai_sim_core)
    ament_add_gtest(actuator_model_test test/actuator_model_test.cpp)
    target_link_libraries(actuator_model_test fsai_sim_core)
    ament_add_gtest(bicycle_backend_test test/bicycle_backend_test.cpp)
    target_link_libraries(bicycle_backend_test fsai_sim_core)
    ament_add_gtest(hybrid_integrator_test test/hybrid_integrator_test.cpp)
    target_link_libraries(hybrid_integrator_test fsai_sim_core)
    ament_add_gtest(plant_test test/plant_test.cpp)
    target_link_libraries(plant_test fsai_sim_core)
    ament_add_gtest(scenario_test test/scenario_test.cpp)
    target_link_libraries(scenario_test fsai_sim_core)
  endif()
  ament_package()
else()
  enable_testing()
  find_package(GTest REQUIRED)
  function(fsai_core_test name)
    add_executable(${name} test/${name}.cpp)
    target_link_libraries(${name} PRIVATE fsai_sim_core GTest::gtest GTest::gtest_main)
    add_test(NAME ${name} COMMAND ${name})
  endfunction()
  fsai_core_test(types_test)
  fsai_core_test(parameters_test)
  fsai_core_test(safety_chain_test)
  fsai_core_test(actuator_model_test)
  fsai_core_test(bicycle_backend_test)
  fsai_core_test(hybrid_integrator_test)
  fsai_core_test(plant_test)
  fsai_core_test(scenario_test)
endif()
