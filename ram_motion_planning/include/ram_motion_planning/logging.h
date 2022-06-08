//
// Created by george on 6/8/22.
//

#ifndef RAM_MOTION_PLANNING_LOGGING_H
#define RAM_MOTION_PLANNING_LOGGING_H

#include <ram_interfaces/msg/toolpath.hpp>
#include <iostream>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <kdl/frames.hpp>

std::ostream& operator<<(std::ostream& os, const geometry_msgs::msg::Point32& point);

std::ostream& operator<<(std::ostream& os, const geometry_msgs::msg::Point& point);

std::ostream& operator<<(std::ostream& os, const ram_interfaces::msg::Toolpath& toolpath);

std::ostream& operator<<(std::ostream& os, const std::vector<geometry_msgs::msg::Pose>& waypoints);

std::ostream& operator<<(std::ostream& os, const KDL::Frame& frame);

std::ostream& operator<<(std::ostream& os, const std::vector<KDL::Frame>& waypoints);

std::ostream& operator<<(std::ostream& os, const geometry_msgs::msg::TransformStamped& trans);

std::ostream& operator<<(std::ostream& os, const sensor_msgs::msg::JointState & joint_state);
#endif //RAM_MOTION_PLANNING_LOGGING_H
