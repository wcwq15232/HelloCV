cmake_minimum_required(VERSION 3.8)
project(demo_use_my_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(example_interfaces_my REQUIRED)

add_executable(server src/server_echo.cpp)
add_executable(client src/client_echo.cpp)
ament_target_dependencies(server rclcpp example_interfaces_my)
ament_target_dependencies(client rclcpp example_interfaces_my)

install(TARGETS
  server client
  DESTINATION lib/${PROJECT_NAME}/
)
install(DIRECTORY launch
DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
