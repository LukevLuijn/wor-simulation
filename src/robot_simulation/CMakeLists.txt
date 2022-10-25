cmake_minimum_required(VERSION 3.5)
project(robot_simulation)

set(CUP_EXE cup_node)
set(ARM_EXE arm_node)

# Default to C99
if (NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif ()

# Default to C++14
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(simulation_msgs REQUIRED)

include_directories(include/${PROJECT_NAME}/)
include_directories(include/utils/)

add_executable(${CUP_EXE}
        src/cup_node.cpp)
target_include_directories(${CUP_EXE}
        PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${CUP_EXE}
        ament_index_cpp
        rclcpp
        tf2_ros
        visualization_msgs
        geometry_msgs
        simulation_msgs)
install(TARGETS ${CUP_EXE}
        DESTINATION lib/${PROJECT_NAME})

add_executable(${ARM_EXE}
        src/arm_node.cpp
        src/robot_arm.cpp
        src/math_utils.cpp
        src/string_utils.cpp
        )
target_include_directories(${ARM_EXE}
        PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${ARM_EXE}
        rclcpp
        simulation_msgs)
install(TARGETS ${ARM_EXE}
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch model urdf
        DESTINATION share/${PROJECT_NAME})

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif ()

ament_package()
