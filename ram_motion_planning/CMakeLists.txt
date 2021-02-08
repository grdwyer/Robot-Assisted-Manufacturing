cmake_minimum_required(VERSION 3.5)
project(ram_motion_planning)

# Fix for missing FastRTPS error
set(FastRTPS_INCLUDE_DIR /opt/ros/foxy/include)
set(FastRTPS_LIBRARY_RELEASE /opt/ros/foxy/lib/libfastrtps.so)

# Common cmake code applied to all moveit packages
find_package(moveit_common REQUIRED)
moveit_package()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_common REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
# This shouldn't be necessary (required by moveit_simple_controller_manager)
find_package(rosidl_default_runtime REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(ram_interfaces REQUIRED)

include_directories(include)

add_library(ram_helpers src/helpers.cpp)
ament_target_dependencies(ram_helpers
        rclcpp
        ram_interfaces
        std_srvs
        )

install(TARGETS ram_helpers
        DESTINATION lib/${PROJECT_NAME})

add_executable(example_program src/example_program.cpp)
target_include_directories(example_program PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(example_program
        ram_helpers)
ament_target_dependencies(example_program
        moveit_ros_planning_interface
        Boost
        )

add_executable(toolpath_follower src/toolpath_follower_position.cpp)
target_include_directories(toolpath_follower PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(toolpath_follower
        ram_helpers)
ament_target_dependencies(toolpath_follower
        moveit_ros_planning_interface
        Boost
        )

install(TARGETS example_program toolpath_follower
  DESTINATION lib/${PROJECT_NAME})

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

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})