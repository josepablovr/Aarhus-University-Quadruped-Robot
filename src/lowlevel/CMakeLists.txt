cmake_minimum_required(VERSION 3.14)
project(lowlevel)

# Set C++ standard
set(CMAKE_CXX_STANDARD 14)  # C++14 is recommended for ROS 2

# Compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(custom_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Boost REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Add executable
add_executable(lowlevel 
    src/lowlevel_node.cpp
    src/control/Estimator.cpp
    src/common/unitreeRobot.cpp
    src/common/LowPassFilter.cpp
    src/Gait/WaveGenerator.cpp
    src/common/unitreeLeg.cpp)

# Specify include directories
target_include_directories(lowlevel PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Link executable with dependencies
ament_target_dependencies(lowlevel
    rclcpp
    sensor_msgs
    custom_interfaces
    std_msgs
    tf2 
    tf2_ros
    nav_msgs
    geometry_msgs
)

# Install executable
install(TARGETS lowlevel
    DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
    DESTINATION include/${PROJECT_NAME}
)

# If building tests
if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    set(ament_cmake_cppcheck_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()
