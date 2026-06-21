# Eigen.cmake
#
# Make the Eigen3 linear-algebra library available as the imported target
# `Eigen3::Eigen`.
#
# Resolution strategy:
#   1. Re-use an Eigen3 target already provided by an enclosing project
#      (e.g. when Kalman itself is pulled in via FetchContent).
#   2. Use a system installation discovered through `find_package(Eigen3)`.
#   3. Fall back to fetching a pinned Eigen release with FetchContent.
#
# The variable `KALMAN_EIGEN_FETCHED` is set to TRUE when Eigen had to be
# downloaded. In that case Eigen is a build target rather than an imported
# one and therefore cannot be referenced from an install/export set; the
# top-level project uses this flag to disable installation accordingly.

set(KALMAN_EIGEN_FETCHED FALSE)

if(TARGET Eigen3::Eigen)
  message(STATUS "Kalman: using Eigen3::Eigen provided by the parent project")
  return()
endif()

# Allow opting out of the system search (useful for reproducible CI builds).
option(KALMAN_USE_SYSTEM_EIGEN "Search for a system installation of Eigen3" ON)

if(KALMAN_USE_SYSTEM_EIGEN)
  find_package(Eigen3 3.3 QUIET NO_MODULE)
endif()

if(TARGET Eigen3::Eigen)
  message(STATUS "Kalman: using system Eigen3 ${Eigen3_VERSION}")
  return()
endif()

set(KALMAN_EIGEN_VERSION
    "3.4.0"
    CACHE STRING "Eigen version to fetch when no system installation is found")

message(STATUS "Kalman: fetching Eigen ${KALMAN_EIGEN_VERSION} via FetchContent")

include(FetchContent)

# Prevent Eigen from configuring its own (large) test / blas / lapack targets.
set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
set(EIGEN_BUILD_DOC OFF CACHE BOOL "" FORCE)
set(EIGEN_BUILD_TESTING OFF CACHE BOOL "" FORCE)
set(EIGEN_BUILD_PKGCONFIG OFF CACHE BOOL "" FORCE)

FetchContent_Declare(
  Eigen3
  GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
  GIT_TAG ${KALMAN_EIGEN_VERSION}
  GIT_SHALLOW TRUE)

FetchContent_MakeAvailable(Eigen3)

set(KALMAN_EIGEN_FETCHED TRUE)
