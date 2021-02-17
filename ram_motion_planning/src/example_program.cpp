/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, PickNik LLC
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PickNik LLC nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Henning Kayser
   Desc: A simple demo node running MoveItCpp for planning and execution
*/

#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <utility>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <iostream>
#include <ram_motion_planning/helpers.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_cpp_demo");

class MoveItCppDemo
{
public:
    explicit MoveItCppDemo(rclcpp::Node::SharedPtr  node)
            : node_(std::move(node))
            , robot_state_publisher_(node_->create_publisher<moveit_msgs::msg::DisplayRobotState>("display_robot_state", 1))
    {
        RCLCPP_INFO(LOGGER, "Initialize MoveItCpp");
        moveit_cpp_ = std::make_shared<moveit::planning_interface::MoveItCpp>(node_);
        moveit_cpp_->getPlanningSceneMonitor()->providePlanningSceneService();  // let RViz display query PlanningScene
        moveit_cpp_->getPlanningSceneMonitor()->setPlanningScenePublishingFrequency(100);

        RCLCPP_INFO(LOGGER, "Initialize PlanningComponent");
        arm_ = std::make_unique<moveit::planning_interface::PlanningComponent>("iiwa", moveit_cpp_);

        auto stock_node = node_->create_sub_node("stock");
        auto gripper_node = node_->create_sub_node("gripper");
        stockHelper_ = std::make_shared<StockHelper>(stock_node);
        gripperHelper_ = std::make_shared<GripperHelper>(gripper_node);

    }


    void run()
    {
        // A little delay before running the plan

        // Load stock
        stockHelper_->load_stock(true);
        rclcpp::sleep_for(std::chrono::seconds(1));
//        system("pause");
        // move to grasp pose
        RCLCPP_INFO(LOGGER, "Set goal");
        geometry_msgs::msg::PoseStamped iiwa_pose;
        iiwa_pose.header.frame_id = "optical_table_back_right_bolt";
        iiwa_pose.header.stamp = this->node_->get_clock()->now();

        iiwa_pose.pose.position.x = -0.475;
        iiwa_pose.pose.position.y = -0.3;
        iiwa_pose.pose.position.z = 0.001;

        iiwa_pose.pose.orientation.y = 1.0;
        iiwa_pose.pose.orientation.w = 6.12e-17;

        arm_->setGoal(iiwa_pose, "gripper_jaw_centre");

        const auto grip_solution = arm_->plan();
        rclcpp::sleep_for(std::chrono::seconds(2));
        if (grip_solution)
        {
            RCLCPP_INFO(LOGGER, "arm.execute()");
            arm_->execute();
        }
        rclcpp::sleep_for(std::chrono::seconds(2));

        // attach stock
        stockHelper_->attach_stock(true);
        stockHelper_->modify_touch_link("cutting_tool_base", true);
        gripperHelper_->gripper(false);
        rclcpp::sleep_for(std::chrono::seconds(1));

        // move to cutter
        RCLCPP_INFO(LOGGER, "To cutter");
        iiwa_pose.header.frame_id = "cutting_tool_tip";
        iiwa_pose.header.stamp = this->node_->get_clock()->now();

        iiwa_pose.pose.position.x = -0.021;
        iiwa_pose.pose.position.y = 0.0;
        iiwa_pose.pose.position.z = -0.01;

        iiwa_pose.pose.orientation.y = 1.0;
        iiwa_pose.pose.orientation.w = 6.12e-17;

        arm_->setGoal(iiwa_pose, "gripper_jaw_centre");

        // Run actual plan
        RCLCPP_INFO(LOGGER, "Plan to goal");
        const auto plan_solution = arm_->plan();
//        moveit_msgs::msg::RobotTrajectory traj;
//        plan_solution.trajectory->getRobotTrajectoryMsg(traj);
//        RCLCPP_INFO_STREAM(LOGGER, traj.joint_trajectory);
        rclcpp::sleep_for(std::chrono::seconds(2));
        if (plan_solution)
        {
            RCLCPP_INFO(LOGGER, "arm.execute()");
            arm_->execute();
        }

        rclcpp::sleep_for(std::chrono::seconds(3));

        stockHelper_->attach_stock(false);
        stockHelper_->load_stock(false);
        gripperHelper_->gripper(true);
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<moveit_msgs::msg::DisplayRobotState>::SharedPtr robot_state_publisher_;
    moveit::planning_interface::MoveItCppPtr moveit_cpp_;
    std::shared_ptr<StockHelper> stockHelper_;
    std::shared_ptr<GripperHelper> gripperHelper_;

    std::unique_ptr<moveit::planning_interface::PlanningComponent> arm_;
};

int main(int argc, char** argv)
{
    RCLCPP_INFO(LOGGER, "Initialize node");
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    // This enables loading undeclared parameters
    // best practice would be to declare parameters in the corresponding classes
    // and provide descriptions about expected use
    node_options.automatically_declare_parameters_from_overrides(true);
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("run_moveit_cpp", "", node_options);

    MoveItCppDemo demo(node);
    std::thread run_demo([&demo]() {
        // Let RViz initialize before running demo
        // TODO(henningkayser): use lifecycle events to launch node
        rclcpp::sleep_for(std::chrono::seconds(5));
        demo.run();
    });

//    rclcpp::spin(node);
    run_demo.join();

    return 0;
}
