//
// Created by george on 6/17/21.
//

#ifndef RAM_TRAJECTORY_UTILS_H
#define RAM_TRAJECTORY_UTILS_H

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.h>
#include <eigen3/Eigen/Geometry>
#include <tf2_eigen/tf2_eigen.h>

// Utilities
bool append_plans(moveit::planning_interface::MoveGroupInterface::Plan &first,
                                       moveit::planning_interface::MoveGroupInterface::Plan &second);

double distance_between_poses(geometry_msgs::msg::Pose a, geometry_msgs::msg::Pose b);
double angular_distance_between_poses(geometry_msgs::msg::Pose a, geometry_msgs::msg::Pose b);

double distance_along_trajectory(std::vector<geometry_msgs::msg::PoseStamped> &trajectory);

bool check_joint_velocities(moveit::core::RobotModelConstPtr robot_model, const std::map<std::string, double>& joint_diff);

// Interpolation
geometry_msgs::msg::Pose interpolate_between_pose(geometry_msgs::msg::Pose start, geometry_msgs::msg::Pose end, double factor);

void interpolate_pose_trajectory(std::vector<geometry_msgs::msg::PoseStamped> &original, double max_distance, double max_angle, std::vector<geometry_msgs::msg::PoseStamped> &interpolated);

void interpolate_pose_trajectory(std::vector<geometry_msgs::msg::Pose> &original, double max_distance, double max_angle, std::vector<geometry_msgs::msg::Pose> &interpolated);

// Trajectory manipulation
bool retime_trajectory_constant_velocity(moveit::planning_interface::MoveGroupInterface::Plan &plan, moveit::core::RobotStatePtr robot_state, double desired_velocity, moveit::planning_interface::MoveGroupInterface::Plan &retimed_plan);

bool retime_trajectory_trapezoidal_velocity(moveit::planning_interface::MoveGroupInterface::Plan &plan,
                                            moveit::core::RobotStatePtr robot_state, double desired_velocity,
                                            double desired_acceleration,
                                            moveit::planning_interface::MoveGroupInterface::Plan &retimed_plan);


#endif //RAM_TRAJECTORY_UTILS_H
