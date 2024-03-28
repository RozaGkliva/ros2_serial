cmake_minimum_required(VERSION 3.8)
project(ros2_cppserial)

# # Default to C99
# if(NOT CMAKE_C_STANDARD)
#   set(CMAKE_C_STANDARD 99)
# endif()

# Default to C++14
# if(NOT CMAKE_CXX_STANDARD)
#   set(CMAKE_CXX_STANDARD 14)
# endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(serial REQUIRED)
find_package(ros2_serial_interfaces REQUIRED)

include_directories(
  include
)

# install executable
add_executable(serial_interface src/serial_interface.cpp)
ament_target_dependencies(serial_interface rclcpp serial ros2_serial_interfaces)

# add_executable(serial_interface_node src/serial_interface_node.cpp)

# install executable
install(TARGETS
  serial_interface
  DESTINATION lib/${PROJECT_NAME}
)

# install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
