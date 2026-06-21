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
