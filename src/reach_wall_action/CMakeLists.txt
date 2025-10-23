cmake_minimum_required(VERSION 3.5)
project(reach_wall_action)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ------------------------------------------------------------
# Dependencies
# ------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ------------------------------------------------------------
# Generate the action interface
# ------------------------------------------------------------
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ReachWall.action"
)

# ------------------------------------------------------------
# This line is required so other packages (Python/C++) can use your action
# ------------------------------------------------------------
ament_export_dependencies(rosidl_default_runtime)

# ------------------------------------------------------------
# Tests (optional)
# ------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
