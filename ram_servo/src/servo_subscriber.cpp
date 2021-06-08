/*******************************************************************************
 *      Title     : pose_tracking_example.cpp
 *      Project   : moveit_servo
 *      Created   : 09/04/2020
 *      Author    : Adam Pettinger
 *
 * BSD 3-Clause License
 *
 * Copyright (c) 2019, Los Alamos National Security, LLC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

#include <std_msgs/msg/int8.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_srvs/srv/empty.hpp>

#include <moveit_servo/servo.h>
#include <moveit_servo/pose_tracking.h>
#include <moveit_servo/status_codes.h>
#include <moveit_servo/servo_parameters.h>
#include <moveit_servo/servo_parameters.h>
#include <moveit_servo/make_shared_from_pool.h>
#include <thread>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("ram.servo");

// Class for monitoring status of moveit_servo
class StatusMonitor
{
public:
    StatusMonitor(const rclcpp::Node::SharedPtr& node, const std::string& topic)
    {
        sub_ = node->create_subscription<std_msgs::msg::Int8>(
                topic, 1, std::bind(&StatusMonitor::statusCB, this, std::placeholders::_1));
    }

private:
    void statusCB(const std_msgs::msg::Int8::ConstSharedPtr msg)
    {
        moveit_servo::StatusCode latest_status = static_cast<moveit_servo::StatusCode>(msg->data);
        if (latest_status != status_)
        {
            status_ = latest_status;
            const auto& status_str = moveit_servo::SERVO_STATUS_CODE_MAP.at(status_);
            RCLCPP_INFO_STREAM(LOGGER, "Servo status: " << status_str);
        }
    }

    moveit_servo::StatusCode status_ = moveit_servo::StatusCode::INVALID;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr sub_;
};

/**
 * Instantiate the pose tracking interface.
 * Send a pose slightly different from the starting pose
 * Then keep updating the target pose a little bit
 */
class ServoSubscriber{
public:
    rclcpp::Node::SharedPtr node_;
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
    moveit_servo::ServoParameters::SharedConstPtr servo_parameters_;
    std::shared_ptr<moveit_servo::PoseTracking> tracker_;
    std::shared_ptr<StatusMonitor> status_monitor_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr service_enable_tracking_;

    std::thread tracker_thread_;

    // TODO: Param this
    Eigen::Vector3d lin_tol_{ 0.001, 0.001, 0.001 };
    double rot_tol_ = 0.01;

    ServoSubscriber(rclcpp::Node::SharedPtr node){
        node_ = node;

        servo_parameters_ = moveit_servo::ServoParameters::makeServoParameters(node_, LOGGER);
        if (servo_parameters_ == nullptr)
        {
            RCLCPP_FATAL(LOGGER, "Could not get servo parameters!");
            exit(EXIT_FAILURE);
        }

        // Load the planning scene monitor

        planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(node_, "robot_description");
        if (!planning_scene_monitor_->getPlanningScene())
        {
            RCLCPP_ERROR_STREAM(LOGGER, "Error in setting up the PlanningSceneMonitor.");
            exit(EXIT_FAILURE);
        }

        planning_scene_monitor_->providePlanningSceneService();
        planning_scene_monitor_->startSceneMonitor();
        planning_scene_monitor_->startWorldGeometryMonitor(
                planning_scene_monitor::PlanningSceneMonitor::DEFAULT_COLLISION_OBJECT_TOPIC,
                planning_scene_monitor::PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_WORLD_TOPIC,
                false /* skip octomap monitor */);
        planning_scene_monitor_->startStateMonitor(servo_parameters_->joint_topic);
        planning_scene_monitor_->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE);

        tracker_ = std::make_shared<moveit_servo::PoseTracking>(node_, servo_parameters_, planning_scene_monitor_);

        // Subscribe to servo status (and log it when it changes)
        status_monitor_ = std::make_shared<StatusMonitor>(node, servo_parameters_->status_topic);

        service_enable_tracking_ = node_->create_service<std_srvs::srv::Empty>(std::string("/enable_servoing"),
                                                                               std::bind(&ServoSubscriber::initialise_pose_tracking, this, std::placeholders::_1, std::placeholders::_2));

    }

    void initialise_pose_tracking(std_srvs::srv::Empty::Request::SharedPtr req, std_srvs::srv::Empty::Response::SharedPtr res){
        RCLCPP_INFO_STREAM(LOGGER, "\n\nInitialising pose tracking\n");
        tracker_->resetTargetPose();
        tracker_thread_ = std::thread(&ServoSubscriber::track_pose, this);
    }

    void track_pose(){
        moveit_servo::PoseTrackingStatusCode tracking_status =
                tracker_->moveToPose(lin_tol_, rot_tol_, 1.0 /* target pose timeout */);
        RCLCPP_INFO_STREAM(LOGGER, "Pose tracker exited with status: "
                << moveit_servo::POSE_TRACKING_STATUS_CODE_MAP.at(tracking_status));
    }

    ~ServoSubscriber(){
        RCLCPP_WARN_STREAM(LOGGER, "Stopping motion and destroying tracker thread");
        tracker_->stopMotion();
        tracker_thread_.join();
    }

};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("ram_servo");

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
//    std::thread executor_thread([&executor]() { executor.spin(); });

    std::shared_ptr<ServoSubscriber> servo = std::make_shared<ServoSubscriber>(node);
    executor.spin();
    rclcpp::shutdown();
}

int old(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("ram_servo");

    std::shared_ptr<ServoSubscriber> servo = std::make_shared<ServoSubscriber>(node);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread executor_thread([&executor]() { executor.spin(); });
    // Make sure the tracker is stopped and clean up

    // Kill executor thread before shutdown
    executor.cancel();
    executor_thread.join();

    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
