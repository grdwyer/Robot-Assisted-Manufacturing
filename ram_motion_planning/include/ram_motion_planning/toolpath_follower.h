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
#include <functional>
#include <kdl/frames.hpp>
#include <math.h>
#include <geometry_msgs/msg/pose_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_kdl/tf2_kdl.h>


class ToolpathFollower : public rclcpp::Node{
public:
    explicit ToolpathFollower(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

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
    void display_planned_trajectory(std::vector<geometry_msgs::msg::Pose> &poses);

    /***
     * Helper function to move to the initial position and load the stock material
     * @return
     */
    bool move_to_setup();
    /***
     * Executes the planned trajectory
     * @return
     */
    bool execute_trajectory();

    /***
     * Callback for setup service
     * this will:
     *   load toolpath
     *   move to setup
     *   plan
     * @param request
     * @param response
     */
    void callback_setup(const std_srvs::srv::Trigger::Request::SharedPtr request,
                        std_srvs::srv::Trigger::Response::SharedPtr response);

    /***
     * Callback for execute service, this will execute the planned trajectory
     * @param request
     * @param response
     */
    void callback_execute(const std_srvs::srv::Trigger::Request::SharedPtr request,
                          std_srvs::srv::Trigger::Response::SharedPtr response);

    bool follow_waypoints_sequentially(std::vector<geometry_msgs::msg::Pose> &waypoints);

private:

    std::shared_ptr<ToolpathHelper> toolpath_helper_;
    std::shared_ptr<StockHelper> stockHelper_;
    std::shared_ptr<GripperHelper> gripperHelper_;

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_setup_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_execute_;

    ram_interfaces::msg::Toolpath toolpath_;
    geometry_msgs::msg::PoseArray toolpath_poses_;
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit_msgs::msg::RobotTrajectory trajectory_toolpath_;

    //rviz pose array publisher
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr publisher_toolpath_poses_;
    rclcpp::TimerBase::SharedPtr timer_toolpath_poses_;

    //TF2
    tf2_ros::Buffer buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tfl_;


};



#endif //RAM_MOTION_PLANNING_TOOLPATH_FOLLOWER_H
