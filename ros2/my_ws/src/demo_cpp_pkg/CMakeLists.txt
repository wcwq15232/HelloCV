cmake_minimum_required(VERSION 3.8)

project(demo_cpp_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(example_interfaces REQUIRED)

add_executable(demo_node src/demo_node.cpp)
add_executable(demo_publisher src/demo_publisher.cpp)
add_executable(demo_subscriber src/demo_subscriber.cpp)
add_executable(demo_server src/demo_service_server.cpp)
add_executable(demo_client src/demo_service_client.cpp)

ament_target_dependencies(demo_node rclcpp)
ament_target_dependencies(demo_publisher rclcpp std_msgs)
ament_target_dependencies(demo_subscriber rclcpp std_msgs)
ament_target_dependencies(demo_server rclcpp std_msgs example_interfaces)
ament_target_dependencies(demo_client rclcpp std_msgs example_interfaces) 


install(TARGETS
  demo_node demo_publisher demo_subscriber demo_server demo_client
  DESTINATION lib/${PROJECT_NAME}/
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
