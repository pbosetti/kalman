# Unit-test executable. Uses the bundled header-only test harness in
# test/lib/Test.hpp instead of an external testing framework.

add_executable(
  kalman_test
  main.cpp
  Matrix.cpp
  StandardBase.cpp
  SquareRootBase.cpp
  ExtendedKalmanFilter.cpp
  UnscentedKalmanFilterBase.cpp
  UnscentedKalmanFilter.cpp
  SquareRootUnscentedKalmanFilter.cpp)

target_link_libraries(kalman_test PRIVATE Kalman::Kalman)

# Allow tests to include "TestHelper.h", "lib/Test.hpp" and "models/..." using
# paths relative to this directory.
target_include_directories(kalman_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(kalman_test PRIVATE -Wall -Wextra)
endif()

add_test(NAME kalman_test COMMAND kalman_test)
