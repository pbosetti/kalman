# Python bindings (pybind11).
#
# Enabled with -DKALMAN_BUILD_PYTHON=ON. pybind11 is located via find_package
# (e.g. `pip install pybind11`, then pass
# -Dpybind11_DIR=$(python -m pybind11 --cmakedir)) and otherwise fetched.

find_package(pybind11 CONFIG QUIET)

if(NOT pybind11_FOUND)
  message(STATUS "Kalman: fetching pybind11 via FetchContent")
  include(FetchContent)
  FetchContent_Declare(
    pybind11
    GIT_REPOSITORY https://github.com/pybind/pybind11.git
    GIT_TAG v2.13.6
    GIT_SHALLOW TRUE)
  FetchContent_MakeAvailable(pybind11)
endif()

# Target name differs from the importable module name to avoid clashing with the
# case-insensitive `Kalman` interface target on some filesystems.
pybind11_add_module(kalman_python kalman_py.cpp)

target_link_libraries(kalman_python PRIVATE Kalman::Kalman)
target_include_directories(kalman_python
                           PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/..)
set_target_properties(kalman_python PROPERTIES OUTPUT_NAME kalman)
