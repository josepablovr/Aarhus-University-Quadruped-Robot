cmake_minimum_required(VERSION 3.8)
project(control_interface_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(custom_interfaces REQUIRED)

add_executable(button_reader src/control_interface.cpp)
add_executable(led_state src/led_interface.cpp)
add_executable(led_test src/led_publisher.cpp)
ament_target_dependencies(button_reader rclcpp std_msgs custom_interfaces)
ament_target_dependencies(led_state rclcpp std_msgs custom_interfaces)
ament_target_dependencies(led_test rclcpp std_msgs custom_interfaces)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


install(TARGETS
  button_reader
  led_state
  led_test
  DESTINATION lib/${PROJECT_NAME})

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

target_link_libraries(button_reader
  ${rclcpp_LIBRARIES}
  pigpiod_if2
  pthread
)
target_link_libraries(led_state
  ${rclcpp_LIBRARIES}
  pigpiod_if2
  pthread
)

target_link_libraries(led_state
  ${rclcpp_LIBRARIES}
  pigpiod_if2
  pthread
)
target_link_libraries(led_test
  ${rclcpp_LIBRARIES}
  pigpiod_if2
  pthread
)
ament_package()
