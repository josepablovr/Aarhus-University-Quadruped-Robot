cmake_minimum_required(VERSION 3.8)
project(motor_communication)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(custom_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(can_network_library REQUIRED)


# Add executable
add_executable(leg_commm src/Leg_Communication.cpp)
ament_target_dependencies(leg_commm rclcpp std_msgs custom_interfaces can_network_library)

# Link executable with libraries
target_link_libraries(leg_commm 
	${rclcpp_LIBRARIES}
  pthread)

# Install executable
install(TARGETS
    leg_commm
    DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY
    include/
    DESTINATION include/${PROJECT_NAME}
)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    set(ament_cmake_cppcheck_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()
