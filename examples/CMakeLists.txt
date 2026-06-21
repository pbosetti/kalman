# Example programs demonstrating how to use the library.

# Minimal, self-contained starting point (1-D constant-velocity tracker).
add_executable(example_template Template/main.cpp)
target_link_libraries(example_template PRIVATE Kalman::Kalman)

# Fully worked 3-DOF planar robot with two measurement models.
add_executable(example_robot1 Robot1/main.cpp)
target_link_libraries(example_robot1 PRIVATE Kalman::Kalman)
