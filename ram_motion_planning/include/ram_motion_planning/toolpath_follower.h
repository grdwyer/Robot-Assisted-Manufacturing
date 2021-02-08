//
// Created by george on 1/27/21.
//

#ifndef RAM_MOTION_PLANNING_TOOLPATH_FOLLOWER_H
#define RAM_MOTION_PLANNING_TOOLPATH_FOLLOWER_H

#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <utility>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <iostream>
#include <ram_motion_planning/helpers.h>
#include <ram_interfaces/msg/toolpath.hpp>
#include <rosidl_runtime_cpp/traits.hpp>

class ToolpathFollower{
public:
    explicit ToolpathFollower(rclcpp::Node::SharedPtr  node);

    /***
     * @brief function to load toolpath from the toolpath handler
     * Load toolpath currently a polygon message and
     * @return
     */
    bool load_toolpath();

    /***
     * @brief formulates the toolpath into a motion planning request for moveit
     * @return
     */
    bool construct_plan_request();

    /***
     * @brief sends trajectory to be displayed in rviz
     */
    void display_planned_trajectory();

private:
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<ToolpathHelper> toolpath_helper_;
    std::shared_ptr<StockHelper> stockHelper_;
    std::shared_ptr<GripperHelper> gripperHelper_;

    ram_interfaces::msg::Toolpath toolpath_;
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;


};



#endif //RAM_MOTION_PLANNING_TOOLPATH_FOLLOWER_H
