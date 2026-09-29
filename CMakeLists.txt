cmake_minimum_required(VERSION 3.16)
project(lems_data VERSION 0.2.0 LANGUAGES CXX)

if(POLICY CMP0177)
  cmake_policy(SET CMP0177 NEW)
endif()

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

option(LEMS_DATA_BUILD_TESTS "Build the lems-data tests" ON)
option(LEMS_DATA_BUILD_APPS "Build the command-line examples" ON)
option(LEMS_DATA_BUILD_COMPAT "Build/install legacy Brown-LEMS VO adapters" ON)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(OpenCV REQUIRED COMPONENTS core imgcodecs imgproc)
find_package(yaml-cpp REQUIRED)

if(TARGET yaml-cpp::yaml-cpp)
  set(LEMS_YAML_CPP_TARGET yaml-cpp::yaml-cpp)
elseif(TARGET yaml-cpp)
  # Older Ubuntu yaml-cpp packages exported the un-namespaced target.
  set(LEMS_YAML_CPP_TARGET yaml-cpp)
else()
  # Keep compatibility with FindYamlCpp modules that expose only a library
  # name; the package config still asks consumers to find yaml-cpp.
  set(LEMS_YAML_CPP_TARGET yaml-cpp)
endif()

set(LEMS_OPENCV_TARGETS)
foreach(component core imgcodecs imgproc)
  if(TARGET opencv_${component})
    list(APPEND LEMS_OPENCV_TARGETS opencv_${component})
  endif()
endforeach()
if(NOT LEMS_OPENCV_TARGETS)
  set(LEMS_OPENCV_TARGETS ${OpenCV_LIBS})
endif()

add_library(lems_data
  src/config.cpp
  src/dataset.cpp
  src/utility.cpp
)
add_library(lems::data ALIAS lems_data)
set_target_properties(lems_data PROPERTIES EXPORT_NAME data)

target_compile_features(lems_data PUBLIC cxx_std_17)
target_include_directories(lems_data
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_link_libraries(lems_data
  PUBLIC
    Eigen3::Eigen
    ${LEMS_YAML_CPP_TARGET}
    ${LEMS_OPENCV_TARGETS}
)

if(LEMS_DATA_BUILD_COMPAT)
  add_library(lems_vo_compat INTERFACE)
  add_library(lems::vo_compat ALIAS lems_vo_compat)
  set_target_properties(lems_vo_compat PROPERTIES EXPORT_NAME vo_compat)
  target_link_libraries(lems_vo_compat INTERFACE lems::data)
  target_include_directories(lems_vo_compat
    INTERFACE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/compat/vo>
      # Installed consumers may include <lems/vo/Dataset.h>; the source-tree
      # adapter path above keeps legacy <Dataset.h> includes working.
      $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
      $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/lems/vo>
  )

  # The stereo repository has a larger, historically different set of
  # matching containers than the GPU multinocular repository.  Keep that
  # profile selectable instead of aliasing incompatible layouts globally.
  add_library(lems_stereo_compat INTERFACE)
  add_library(lems::stereo_compat ALIAS lems_stereo_compat)
  set_target_properties(lems_stereo_compat PROPERTIES EXPORT_NAME stereo_compat)
  target_link_libraries(lems_stereo_compat INTERFACE lems::vo_compat)
  target_compile_definitions(lems_stereo_compat INTERFACE LEMS_DATA_STEREO_COMPAT=1)
  target_include_directories(lems_stereo_compat
    INTERFACE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/compat/vo/stereo_profile>
      $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
      $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/lems/vo/stereo_profile>
  )
endif()

if(LEMS_DATA_BUILD_APPS)
  add_executable(lems-data-inspect apps/inspect.cpp)
  target_link_libraries(lems-data-inspect PRIVATE lems::data)
endif()

if(LEMS_DATA_BUILD_TESTS)
  enable_testing()

  function(lems_add_test target source)
    add_executable(${target} ${source})
    target_link_libraries(${target} PRIVATE lems::data)
    add_test(NAME ${target} COMMAND ${target})
  endfunction()

  lems_add_test(lems_data_tests tests/dataset_test.cpp)
  lems_add_test(lems_data_core_types_tests tests/core_types_test.cpp)
  lems_add_test(lems_data_utility_tests tests/utility_test.cpp)
  lems_add_test(lems_data_dataset_formats_tests tests/dataset_formats_test.cpp)
  lems_add_test(lems_data_config_validation_tests
                tests/config_validation_test.cpp)

  if(LEMS_DATA_BUILD_COMPAT)
    add_executable(lems_vo_compat_tests tests/compat_test.cpp)
    target_link_libraries(lems_vo_compat_tests PRIVATE lems::vo_compat)
    add_test(NAME lems_vo_compat_tests COMMAND lems_vo_compat_tests)
  endif()
endif()

if(LEMS_DATA_BUILD_APPS)
  install(TARGETS lems-data-inspect RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
endif()

install(TARGETS lems_data EXPORT lemsDataTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
if(LEMS_DATA_BUILD_COMPAT)
  install(TARGETS lems_vo_compat EXPORT lemsDataTargets)
  install(TARGETS lems_stereo_compat EXPORT lemsDataTargets)
endif()

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
if(LEMS_DATA_BUILD_COMPAT)
  install(DIRECTORY compat/vo/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/lems/vo
    FILES_MATCHING PATTERN "*.h"
  )
endif()

set(LEMS_DATA_INSTALL_CMAKEDIR ${CMAKE_INSTALL_LIBDIR}/cmake/lemsData)
configure_package_config_file(
  cmake/lemsDataConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/lemsDataConfig.cmake
  INSTALL_DESTINATION ${LEMS_DATA_INSTALL_CMAKEDIR}
)
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/lemsDataConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion
)

install(EXPORT lemsDataTargets
  FILE lemsDataTargets.cmake
  NAMESPACE lems::
  DESTINATION ${LEMS_DATA_INSTALL_CMAKEDIR}
)
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/lemsDataConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/lemsDataConfigVersion.cmake
  DESTINATION ${LEMS_DATA_INSTALL_CMAKEDIR}
)

install(FILES README.md DESTINATION ${CMAKE_INSTALL_DOCDIR})
install(DIRECTORY docs/ DESTINATION ${CMAKE_INSTALL_DOCDIR}/docs)
install(DIRECTORY migration/
  DESTINATION ${CMAKE_INSTALL_DOCDIR}/migration
  FILES_MATCHING PATTERN "*.sh" PATTERN "*.cpp"
)
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/examples)
  install(DIRECTORY examples/ DESTINATION ${CMAKE_INSTALL_DATADIR}/lems-data/examples)
endif()
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/configs)
  install(DIRECTORY configs/ DESTINATION ${CMAKE_INSTALL_DATADIR}/lems-data/configs)
endif()
