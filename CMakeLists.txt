cmake_minimum_required(VERSION 3.16)
project(lems_data VERSION 0.1.0 LANGUAGES CXX)

option(LEMS_DATA_BUILD_TESTS "Build tests" ON)
add_library(lems_data src/dataset.cpp src/config.cpp)
add_library(lems::data ALIAS lems_data)
target_compile_features(lems_data PUBLIC cxx_std_17)
target_include_directories(lems_data PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)

add_executable(lems-data-inspect apps/inspect.cpp)
target_link_libraries(lems-data-inspect PRIVATE lems::data)

if(LEMS_DATA_BUILD_TESTS)
  enable_testing()
  add_executable(lems_data_tests tests/dataset_test.cpp)
  target_link_libraries(lems_data_tests PRIVATE lems::data)
  add_test(NAME lems_data_tests COMMAND lems_data_tests)
endif()

include(GNUInstallDirs)
install(TARGETS lems_data EXPORT lemsDataTargets)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(EXPORT lemsDataTargets NAMESPACE lems:: DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/lemsData)
