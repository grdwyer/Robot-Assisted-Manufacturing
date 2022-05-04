cmake_minimum_required(VERSION 3.5)
project(ram_behaviour)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behavior_tree REQUIRED)
find_package(ram_interfaces REQUIRED)
find_package(std_srvs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp_action REQUIRED)


set(dependencies
        rclcpp
        rclcpp_action
        behaviortree_cpp_v3
        geometry_msgs
        std_msgs
        std_srvs
        behavior_tree
        ram_interfaces
        )

## Plugins
add_library(trigger_component_bt_node SHARED plugins/service/trigger_component.cpp)
list(APPEND plugin_libs trigger_component_bt_node)

add_library(set_bool_component_bt_node SHARED plugins/service/set_bool_component.cpp)
list(APPEND plugin_libs set_bool_component_bt_node)

add_library(request_trigger_component_bt_node SHARED plugins/service/request_trigger_component.cpp)
list(APPEND plugin_libs request_trigger_component_bt_node)

#add_library(hold_tweezer_bt_node SHARED plugins/service/hold_tweezer.cpp)
#list(APPEND plugin_libs hold_tweezer_bt_node)
#
#add_library(orient_sample_holder_bt_node SHARED plugins/service/orient_sample_holder.cpp)
#list(APPEND plugin_libs orient_sample_holder_bt_node)
#
#add_library(random_cut_bt_node SHARED plugins/service/random_cut.cpp)
#list(APPEND plugin_libs random_cut_bt_node)
#
##add_library(random_move_bt_node SHARED plugins/service/random_move.cpp)
##list(APPEND plugin_libs random_move_bt_node)
#
#add_library(release_tweezer_bt_node SHARED plugins/service/release_tweezer.cpp)
#list(APPEND plugin_libs release_tweezer_bt_node)
#
#add_library(display_markers_bt_node SHARED plugins/service/display_markers.cpp)
#list(APPEND plugin_libs display_markers_bt_node)
#
#add_library(remove_markers_bt_node SHARED plugins/service/remove_markers.cpp)
#list(APPEND plugin_libs remove_markers_bt_node)
#
#add_library(get_cut_from_interface_bt_node SHARED plugins/service/get_cut_from_interface.cpp)
#list(APPEND plugin_libs get_cut_from_interface_bt_node)
#
#add_library(get_move_from_interface_bt_node SHARED plugins/service/get_move_from_interface.cpp)
#list(APPEND plugin_libs get_move_from_interface_bt_node)

## Plugin deps
foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

##Install
install(TARGETS ${plugin_libs}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        )

install(DIRECTORY launch trees
        DESTINATION share/${PROJECT_NAME}
        )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()