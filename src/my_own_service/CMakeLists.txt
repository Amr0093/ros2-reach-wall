cmake_minimum_required(VERSION 3.5)
project(my_own_service)

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

# ------------------------------------------------------------------------------
# Find dependencies
# ------------------------------------------------------------------------------

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ------------------------------------------------------------------------------
# Generate service interface code
# ------------------------------------------------------------------------------

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/AddTwoInt.srv"
  DEPENDENCIES std_msgs
)

# ------------------------------------------------------------------------------
# Testing and linting (optional)
# ------------------------------------------------------------------------------

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Uncomment these lines if needed:
  # set(ament_cmake_copyright_FOUND TRUE)
  # set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# ------------------------------------------------------------------------------
# Package export
# ------------------------------------------------------------------------------

ament_package()
