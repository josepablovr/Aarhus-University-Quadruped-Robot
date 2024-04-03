cmake_minimum_required(VERSION 3.8)
project(comm_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(custom_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

# Add executable
add_executable(leg_comm
    src/Leg_Communication.cpp
    src/Leg_CAN_Network.cpp
    src/RMD_X8.cpp
    src/CAN_BUS.cpp
    src/EncoderCounter.cpp
)

ament_target_dependencies(leg_comm rclcpp std_msgs custom_interfaces)

# Include directories for your project
target_include_directories(leg_comm PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# Link executable with libraries
target_link_libraries(leg_comm
    ${rclcpp_LIBRARIES}   
    ${std_msgs_LIBRARIES}
    ${custom_interfaces_LIBRARIES} # Link against custom_interfaces
    pthread
)

# Install executable
install(TARGETS
    leg_comm
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