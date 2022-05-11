//
// Created by george on 3/30/21.
//

#ifndef RAM_MOTION_PLANNING_OMPL_CONSTRAINED_TOOLPATH_PLANNER_H
#define RAM_MOTION_PLANNING_OMPL_CONSTRAINED_TOOLPATH_PLANNER_H

#include <ram_motion_planning/base_toolpath_planner.h>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit/planning_scene_monitor/current_state_monitor.h>
#include <trajectory_msgs/msg/joint_trajectory_point.h>

class OMPLToolpathPlanner : public BaseToolpathPlanner{
//    using BaseToolpathPlanner::BaseToolpathPlanner;
public:
    OMPLToolpathPlanner(const rclcpp::NodeOptions & options);
    bool construct_toolpath_plan();
    bool plan_between_points(const moveit::core::RobotStatePtr& start_state, geometry_msgs::msg::Pose& start, geometry_msgs::msg::Pose& end, moveit::planning_interface::MoveGroupInterface::Plan& plan);
    bool append_plans(moveit::planning_interface::MoveGroupInterface::Plan& first, moveit::planning_interface::MoveGroupInterface::Plan& second);
    void display_line(const geometry_msgs::msg::Pose& pose, const rosidl_runtime_cpp::BoundedVector<double, 3, std::allocator<double>>& dimensions);
    void display_sphere(const geometry_msgs::msg::Pose& pose, const std::string& color, int id);
    void delete_markers();
    moveit_msgs::msg::RobotState get_end_state_from_plan(moveit::planning_interface::MoveGroupInterface::Plan& plan);

protected:
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    std::shared_ptr<planning_scene_monitor::CurrentStateMonitor> state_monitor_;
    /***
     * Callback for setup service
     * this will:
     *   load toolpath
     *   move to setup
     *   plan
     * @param request
     * @param response
     */
    void callback_setup(std_srvs::srv::Trigger::Request::SharedPtr request,
                        std_srvs::srv::Trigger::Response::SharedPtr response);

    /***
     * Callback for execute service, this will execute the planned trajectory
     * @param request
     * @param response
     */
    void callback_execute(std_srvs::srv::Trigger::Request::SharedPtr request,
                          std_srvs::srv::Trigger::Response::SharedPtr response);

};

#endif //RAM_MOTION_PLANNING_OMPL_CONSTRAINED_TOOLPATH_PLANNER_H
