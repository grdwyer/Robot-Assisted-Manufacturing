//
// Created by george on 3/30/21.
//

#ifndef RAM_MOTION_PLANNING_OMPL_CONSTRAINED_TOOLPATH_PLANNER_H
#define RAM_MOTION_PLANNING_OMPL_CONSTRAINED_TOOLPATH_PLANNER_H

#include <ram_motion_planning/base_toolpath_planner.h>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit/planning_scene_monitor/current_state_monitor.h>
#include <trajectory_msgs/msg/joint_trajectory_point.h>
#include <geometry_msgs/msg/pose_stamped.hpp>

class ServoToolpathPlanner : public BaseToolpathPlanner{

public:
    ServoToolpathPlanner(const rclcpp::NodeOptions & options);
    bool construct_plan_request();
    bool execute_trajectory();

protected:
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr servo_pose_pub_;

    std::vector<geometry_msgs::msg::PoseStamped> pose_trajectory_;
    std::shared_ptr<ServoHelper> servo_helper_;

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
