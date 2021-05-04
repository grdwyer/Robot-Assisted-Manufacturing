//
// Created by george on 3/30/21.
//

#ifndef RAM_MOTION_PLANNING_OMPL_CONSTRAINED_TOOLPATH_PLANNER_H
#define RAM_MOTION_PLANNING_OMPL_CONSTRAINED_TOOLPATH_PLANNER_H

#include <ram_motion_planning/base_toolpath_planner.h>
#include <moveit_msgs/msg/constraints.hpp>

class OMPLToolpathPlanner : public BaseToolpathPlanner{
    using BaseToolpathPlanner::BaseToolpathPlanner;
    bool construct_plan_request();
    bool plan_between_points(moveit::core::RobotStatePtr start_state, geometry_msgs::msg::Pose& end, moveit::planning_interface::MoveGroupInterface::Plan& plan);
    bool append_plans(moveit::planning_interface::MoveGroupInterface::Plan& first, moveit::planning_interface::MoveGroupInterface::Plan& second);
    moveit::core::RobotStatePtr get_end_state_from_plan(moveit::planning_interface::MoveGroupInterface::Plan& plan);
};

#endif //RAM_MOTION_PLANNING_OMPL_CONSTRAINED_TOOLPATH_PLANNER_H
