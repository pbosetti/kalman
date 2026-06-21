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
# Optional: build a second benchmark linked against the upstream mherb/kalman
# headers so CI can emit a side-by-side comparison table.
# ---------------------------------------------------------------------------
option(KALMAN_BENCHMARK_UPSTREAM
  "Build kalman_benchmark_upstream against the upstream mherb/kalman headers" OFF)

if(KALMAN_BENCHMARK_UPSTREAM)
  include(FetchContent)

  FetchContent_Declare(
    kalman_upstream
    GIT_REPOSITORY https://github.com/mherb/kalman.git
    GIT_TAG        master
    GIT_SHALLOW    TRUE)

  FetchContent_GetProperties(kalman_upstream)
  if(NOT kalman_upstream_POPULATED)
    # Use Populate (not MakeAvailable) so that the upstream CMakeLists.txt —
    # which expects Eigen as a git submodule — is never executed.
    FetchContent_Populate(kalman_upstream)
  endif()

  # Reuse the same main.cpp but point at the upstream Robot1 models and
  # upstream <kalman/...> headers.  Eigen is already available from the outer
  # project so we can link against it directly.
  add_executable(kalman_benchmark_upstream main.cpp)

  target_include_directories(
    kalman_benchmark_upstream PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}                    # lib/Benchmark.hpp
      ${kalman_upstream_SOURCE_DIR}/examples/Robot1  # SystemModel.hpp etc.
      ${kalman_upstream_SOURCE_DIR})                 # <kalman/...> headers

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
