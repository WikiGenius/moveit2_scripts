cmake_minimum_required(VERSION 3.8)
project(moveit2_scripts)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)

# Create the library from approach_and_retreat_node.cpp
add_library(approach_and_retreat_lib SHARED src/approach_and_retreat_node.cpp)

# Generate the executable
add_executable(test_trajectory
  src/test_trajectory.cpp)
add_executable(test_trajectory2
  src/test_trajectory2.cpp)
add_executable(test_trajectory3
  src/test_trajectory3.cpp)
add_executable(test_gripper
  src/test_gripper.cpp)
add_executable(approach_retreat
  src/approach_retreat.cpp)

target_include_directories(test_trajectory
  PUBLIC)
target_include_directories(test_trajectory2
  PUBLIC)
target_include_directories(test_trajectory3
  PUBLIC)
target_include_directories(test_gripper
  PUBLIC)
# Specify include directories for the library (if your header files are in the include folder)
target_include_directories(approach_and_retreat_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
# Specify include directories for the executable
target_include_directories(approach_retreat PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)



ament_target_dependencies(test_trajectory
  ament_cmake
  rclcpp
  rclcpp_action
  moveit_core
  moveit_ros_planning_interface
  interactive_markers
  moveit_ros_planning
  control_msgs)
ament_target_dependencies(test_trajectory2
  ament_cmake
  rclcpp
  rclcpp_action
  moveit_core
  moveit_ros_planning_interface
  interactive_markers
  moveit_ros_planning
  control_msgs)
ament_target_dependencies(test_trajectory3
  ament_cmake
  rclcpp
  rclcpp_action
  moveit_core
  moveit_ros_planning_interface
  interactive_markers
  moveit_ros_planning
  control_msgs)
ament_target_dependencies(test_gripper
  ament_cmake
  rclcpp
  rclcpp_action
  moveit_core
  moveit_ros_planning_interface
  interactive_markers
  moveit_ros_planning
  control_msgs)

ament_target_dependencies(approach_and_retreat_lib
  ament_cmake
  rclcpp
  rclcpp_action
  moveit_core
  moveit_ros_planning_interface
  interactive_markers
  moveit_ros_planning
  control_msgs)

# Link the approach_retreat_node executable with the approach_and_retreat_node library
target_link_libraries(approach_retreat approach_and_retreat_lib)


# Install the library
install(TARGETS approach_and_retreat_lib
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION)

# Install the executable
install(TARGETS
  test_trajectory
  test_trajectory2
  test_trajectory3
  test_gripper
  approach_retreat
  DESTINATION lib/${PROJECT_NAME}
)
# Install header files
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
# Install the launch file
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
