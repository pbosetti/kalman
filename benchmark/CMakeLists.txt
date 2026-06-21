# Micro-benchmark executable. Uses the bundled header-only benchmark harness in
# benchmark/lib/Benchmark.hpp and reuses the Robot1 example models.

add_executable(kalman_benchmark main.cpp)

target_link_libraries(kalman_benchmark PRIVATE Kalman::Kalman)

# Make the harness ("lib/Benchmark.hpp") and the reused Robot1 models available.
target_include_directories(
  kalman_benchmark PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}
                           ${CMAKE_CURRENT_SOURCE_DIR}/../examples/Robot1)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(kalman_benchmark PRIVATE -Wall -Wextra)
endif()

# Instantiating all four filters in one translation unit produces a very large
# object; MSVC (especially in Debug) needs /bigobj to stay under the COFF
# section limit.
if(MSVC)
  target_compile_options(kalman_benchmark PRIVATE /bigobj)
endif()

# ---------------------------------------------------------------------------
# Optional: build a second benchmark against the upstream mherb/kalman headers
# to produce a side-by-side comparison table in CI.
#
# The upstream mherb/kalman repo has its headers at the repository root (not in
# a kalman/ subdirectory).  To make #include <kalman/...> resolve correctly, the
# repo must be cloned into a directory named "kalman" and the *parent* of that
# directory must be on the include path.
#
# Set KALMAN_BENCHMARK_UPSTREAM_DIR to that parent directory, i.e. the
# directory that *contains* the cloned "kalman/" folder.  The CI workflow
# creates this layout with:
#   mkdir -p <UPSTREAM_DIR>
#   git clone https://github.com/mherb/kalman.git <UPSTREAM_DIR>/kalman
# ---------------------------------------------------------------------------
set(KALMAN_BENCHMARK_UPSTREAM_DIR "" CACHE PATH
  "Parent of an mherb/kalman checkout named 'kalman/'; enables kalman_benchmark_upstream")

if(KALMAN_BENCHMARK_UPSTREAM_DIR)
  set(_upstream_root "${KALMAN_BENCHMARK_UPSTREAM_DIR}/kalman")
  if(NOT EXISTS "${_upstream_root}")
    message(WARNING
      "KALMAN_BENCHMARK_UPSTREAM_DIR='${KALMAN_BENCHMARK_UPSTREAM_DIR}' does not "
      "contain a 'kalman/' sub-directory. Clone mherb/kalman into that sub-directory.")
  else()
    # Reuse the same main.cpp but point at upstream Robot1 models and headers.
    # Eigen is already available from the outer project.
    add_executable(kalman_benchmark_upstream main.cpp)

    target_include_directories(
      kalman_benchmark_upstream PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}                       # lib/Benchmark.hpp
        ${_upstream_root}/examples/Robot1                 # SystemModel.hpp etc.
        ${KALMAN_BENCHMARK_UPSTREAM_DIR})                 # <kalman/...> headers

    target_link_libraries(kalman_benchmark_upstream PRIVATE Eigen3::Eigen)

    # Upstream is C++11/14; the harness header uses inline variables (C++17).
    target_compile_features(kalman_benchmark_upstream PRIVATE cxx_std_17)

    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
      target_compile_options(kalman_benchmark_upstream PRIVATE -Wall -Wextra)
    endif()
    if(MSVC)
      target_compile_options(kalman_benchmark_upstream PRIVATE /bigobj)
    endif()
  endif()
endif()
