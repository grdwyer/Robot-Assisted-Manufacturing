//
// Created by george on 1/27/21.
//

#ifndef RAM_MOTION_PLANNING_BASE_TOOLPATH_PLANNER_H
#define RAM_MOTION_PLANNING_BASE_TOOLPATH_PLANNER_H

#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <ram_interfaces/srv/get_toolpath_parameters.hpp>
#include <ram_interfaces/srv/set_toolpath_parameters.hpp>
#include <ram_interfaces/srv/set_toolpath.hpp>
#include <utility>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <iostream>
#include <ram_motion_planning/logging.h>
#include <ram_motion_planning/trajectory_utils.h>
#include <ram_interfaces/msg/toolpath.hpp>
#include <rosidl_runtime_cpp/traits.hpp>
#include <functional>
#include <kdl/frames.hpp>
#include <math.h>
#include <geometry_msgs/msg/pose_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_kdl/tf2_kdl.hpp>

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/collision_detection/collision_matrix.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/msg/planning_scene.hpp>


class BaseToolpathPlanner : public rclcpp::Node{
public:
    explicit BaseToolpathPlanner(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

    /***
     * Prints the configuration of the node using the parameters declared
     */
    void configuration_message();

    /***
     * @brief formulates the toolpath into a motion planning request for moveit
     * @return
     */
    virtual bool construct_toolpath_plan();

    /***
     * @brief formulates the toolpath into a motion planning request for moveit
     * @return
     */
    virtual bool construct_approach_plan();

    /***
     * Helper function to move to the initial position and load the stock material
     * @return
     */
    bool move_to_setup();
    /***
     * Executes the planned trajectory
     * @return
     */
    virtual bool execute_trajectory(moveit_msgs::msg::RobotTrajectory &trajectory);

    /***
     * @brief Uses TF to get the transform between the EE link and the stock material (or held part)
     * The EE link is taken from moveits EE link `move_group_->getEndEffectorLink()` and the stock frame from the
     * parameter `part_reference_frame`
     * @return Transform from end effector to the stock frame
     */
    KDL::Frame get_ee_to_stock_transform();

    /***
     * @brief processes the received toolpath to get the cartesian path required for the end effector
     * @return boolean for sucess processing the toolpath
     */
    bool process_toolpath(std::vector<KDL::Frame> &ee_cartesian_path);

    bool generate_cartesian_trajectory(std::vector<KDL::Frame> &ee_cartesian_path);

    /***
     * Callback for setup service
     * this will plan the trajectory for the toolpath provided the approach has been planned already.
     * @param request
     * @param response
     */
    virtual void callback_setup_toolpath(ram_interfaces::srv::SetToolpath::Request::SharedPtr request,
                                ram_interfaces::srv::SetToolpath::Response::SharedPtr response);

    /***
     * Callback for setup service
     * this will plan the motion to the approach position of the toolpath, will take the first position of the toolapth
     * and transform it by the approach offset and plan to that
     * @param request
     * @param response
     */
    virtual void callback_setup_approach(ram_interfaces::srv::SetToolpath::Request::SharedPtr request,
                                         ram_interfaces::srv::SetToolpath::Response::SharedPtr response);

    /***
     * Callback for execute service, this will execute the planned toolpath trajectory
     * @param request
     * @param response
     */
    virtual void callback_execute_toolpath(std_srvs::srv::Trigger::Request::SharedPtr request,
                                           std_srvs::srv::Trigger::Response::SharedPtr response);

    /***
     * Callback for execute service, this will execute the planned trajectory
     * @param request
     * @param response
     */
    virtual void callback_execute_approach(std_srvs::srv::Trigger::Request::SharedPtr request,
                                           std_srvs::srv::Trigger::Response::SharedPtr response);

    /***
     * Gets the parameters for planning and executing toolpaths
     * @param request
     * @param response
     */
    virtual void callback_get_parameters(ram_interfaces::srv::GetToolpathParameters::Request::SharedPtr request,
                                         ram_interfaces::srv::GetToolpathParameters::Response::SharedPtr response);

    /***
     * Sets the parameters for planning and executing toolpaths
     * @param request
     * @param response
     */
    virtual void callback_set_parameters(ram_interfaces::srv::SetToolpathParameters::Request::SharedPtr request,
                                         ram_interfaces::srv::SetToolpathParameters::Response::SharedPtr response);

    // DEBUG functions

    /***
    * @brief sends trajectory to be displayed in rviz
    */
    void display_planned_trajectory(std::vector<geometry_msgs::msg::Pose> &poses);


protected:
    void setup_parameters();
    void run_moveit_executor();

    void debug_mode_wait();

    rclcpp::Service<ram_interfaces::srv::SetToolpath>::SharedPtr service_setup_toolpath_;
    rclcpp::Service<ram_interfaces::srv::SetToolpath>::SharedPtr service_setup_approach_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_execute_toolpath_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_execute_approach_;

    rclcpp::Service<ram_interfaces::srv::GetToolpathParameters>::SharedPtr service_get_parameters_;
    rclcpp::Service<ram_interfaces::srv::SetToolpathParameters>::SharedPtr service_set_parameters_;

    ram_interfaces::msg::Toolpath toolpath_;
    std::vector<KDL::Frame> ee_cartesian_path_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::core::RobotStatePtr robot_state_;
    moveit_msgs::msg::RobotTrajectory trajectory_toolpath_;
    moveit_msgs::msg::RobotTrajectory trajectory_approach_;

    //rviz pose array publisher
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr publisher_toolpath_poses_;
    rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr publisher_trajectory_;
    geometry_msgs::msg::PoseArray toolpath_poses_;
    rclcpp::TimerBase::SharedPtr timer_toolpath_poses_;

    //TF2
    tf2_ros::Buffer buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tfl_;

    rclcpp::executors::SingleThreadedExecutor executor_moveit_;
    std::thread thread_moveit_executor_;
};



#endif //RAM_MOTION_PLANNING_BASE_TOOLPATH_PLANNER_H
