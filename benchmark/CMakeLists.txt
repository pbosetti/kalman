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
# Set KALMAN_BENCHMARK_UPSTREAM_DIR to the root of a local checkout of
# https://github.com/mherb/kalman (not fetched automatically to keep the
# normal build fast and offline-friendly).
# ---------------------------------------------------------------------------
set(KALMAN_BENCHMARK_UPSTREAM_DIR "" CACHE PATH
  "Path to a checkout of the upstream mherb/kalman repo; enables kalman_benchmark_upstream")

if(KALMAN_BENCHMARK_UPSTREAM_DIR)
  if(NOT EXISTS "${KALMAN_BENCHMARK_UPSTREAM_DIR}/kalman")
    message(WARNING
      "KALMAN_BENCHMARK_UPSTREAM_DIR is set but does not contain a 'kalman/' "
      "sub-directory: ${KALMAN_BENCHMARK_UPSTREAM_DIR}")
  else()
    # Reuse the same main.cpp but point at upstream Robot1 models and headers.
    # Eigen is already available from the outer project.
    add_executable(kalman_benchmark_upstream main.cpp)

    target_include_directories(
      kalman_benchmark_upstream PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}                             # lib/Benchmark.hpp
        ${KALMAN_BENCHMARK_UPSTREAM_DIR}/examples/Robot1       # SystemModel.hpp etc.
        ${KALMAN_BENCHMARK_UPSTREAM_DIR})                      # <kalman/...> headers

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
