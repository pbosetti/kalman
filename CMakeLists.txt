cmake_minimum_required(VERSION 3.16)

project(
  Kalman
  VERSION 1.0.0
  DESCRIPTION
    "Header-only C++ Kalman filter library (EKF, SR-EKF, UKF, SR-UKF)"
  HOMEPAGE_URL "https://github.com/pbosetti/kalman"
  LANGUAGES CXX)

# ----------------------------------------------------------------------------
# Project options
# ----------------------------------------------------------------------------

# PROJECT_IS_TOP_LEVEL is provided automatically by CMake >= 3.21. Define it
# manually for older versions so the same logic works everywhere.
if(NOT DEFINED PROJECT_IS_TOP_LEVEL)
  if(CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
    set(PROJECT_IS_TOP_LEVEL TRUE)
  else()
    set(PROJECT_IS_TOP_LEVEL FALSE)
  endif()
endif()

# By default examples, tests and install rules are only enabled when Kalman is
# the top-level project. Consumers that pull Kalman in via add_subdirectory()
# or FetchContent therefore get nothing but the library target.
option(KALMAN_BUILD_EXAMPLES "Build the Kalman examples" ${PROJECT_IS_TOP_LEVEL})
option(KALMAN_BUILD_TESTS "Build the Kalman unit tests" ${PROJECT_IS_TOP_LEVEL})
option(KALMAN_INSTALL "Generate install and export rules for Kalman"
       ${PROJECT_IS_TOP_LEVEL})

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

# ----------------------------------------------------------------------------
# Dependencies
# ----------------------------------------------------------------------------

include(Eigen)

# ----------------------------------------------------------------------------
# The library (header-only INTERFACE target)
# ----------------------------------------------------------------------------

# The real target is named `Kalman` so that, combined with the `Kalman::`
# export namespace, both build-tree (add_subdirectory / FetchContent) and
# installed (find_package) consumers reference the identical `Kalman::Kalman`.
add_library(Kalman INTERFACE)
add_library(Kalman::Kalman ALIAS Kalman)

target_include_directories(
  Kalman INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                   $<INSTALL_INTERFACE:include>)
target_link_libraries(Kalman INTERFACE Eigen3::Eigen)
target_compile_features(Kalman INTERFACE cxx_std_11)

# ----------------------------------------------------------------------------
# Examples and tests
# ----------------------------------------------------------------------------

if(KALMAN_BUILD_EXAMPLES)
  add_subdirectory(examples)
endif()

if(KALMAN_BUILD_TESTS)
  enable_testing()
  add_subdirectory(test)
endif()

# ----------------------------------------------------------------------------
# Install / export
# ----------------------------------------------------------------------------

if(KALMAN_INSTALL AND KALMAN_EIGEN_FETCHED)
  message(
    STATUS
      "Kalman: Eigen was fetched with FetchContent, so install/export rules "
      "are disabled. Install Eigen system-wide to enable them.")
endif()

if(KALMAN_INSTALL AND NOT KALMAN_EIGEN_FETCHED)
  include(GNUInstallDirs)
  include(CMakePackageConfigHelpers)

  set(KALMAN_CMAKE_CONFIG_DIR "${CMAKE_INSTALL_LIBDIR}/cmake/Kalman")

  install(TARGETS Kalman EXPORT KalmanTargets)
  install(DIRECTORY include/kalman DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

  install(
    EXPORT KalmanTargets
    FILE KalmanTargets.cmake
    NAMESPACE Kalman::
    DESTINATION ${KALMAN_CMAKE_CONFIG_DIR})

  configure_package_config_file(
    cmake/KalmanConfig.cmake.in
    "${CMAKE_CURRENT_BINARY_DIR}/KalmanConfig.cmake"
    INSTALL_DESTINATION ${KALMAN_CMAKE_CONFIG_DIR})

  write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/KalmanConfigVersion.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion)

  install(FILES "${CMAKE_CURRENT_BINARY_DIR}/KalmanConfig.cmake"
                "${CMAKE_CURRENT_BINARY_DIR}/KalmanConfigVersion.cmake"
          DESTINATION ${KALMAN_CMAKE_CONFIG_DIR})

  export(EXPORT KalmanTargets NAMESPACE Kalman::
         FILE "${CMAKE_CURRENT_BINARY_DIR}/KalmanTargets.cmake")
endif()
