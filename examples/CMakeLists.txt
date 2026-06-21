# Example programs demonstrating how to use the library.

add_executable(example_robot1 Robot1/main.cpp)
target_link_libraries(example_robot1 PRIVATE Kalman::Kalman)
