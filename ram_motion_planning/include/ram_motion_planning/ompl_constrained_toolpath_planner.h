//
// Created by george on 3/30/21.
//

#ifndef RAM_MOTION_PLANNING_OMPL_CONSTRAINED_TOOLPATH_PLANNER_H
#define RAM_MOTION_PLANNING_OMPL_CONSTRAINED_TOOLPATH_PLANNER_H

#include <ram_motion_planning/base_toolpath_planner.h>
#include <moveit_msgs/msg/constraints.hpp>

class OMPLToolpathPlanner : public BaseToolpathPlanner{
//    using BaseToolpathPlanner::BaseToolpathPlanner;
public:
    OMPLToolpathPlanner(const rclcpp::NodeOptions & options);
    bool construct_plan_request();
    bool plan_between_points(const moveit::core::RobotStatePtr& start_state, geometry_msgs::msg::Pose& end, moveit::planning_interface::MoveGroupInterface::Plan& plan);
    bool append_plans(moveit::planning_interface::MoveGroupInterface::Plan& first, moveit::planning_interface::MoveGroupInterface::Plan& second);
    void display_line(const geometry_msgs::msg::Pose& pose, const rosidl_runtime_cpp::BoundedVector<double, 3, std::allocator<double>>& dimensions);
    moveit::core::RobotStatePtr get_end_state_from_plan(moveit::planning_interface::MoveGroupInterface::Plan& plan);

protected:
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
};

#endif //RAM_MOTION_PLANNING_OMPL_CONSTRAINED_TOOLPATH_PLANNER_H
