//
// Created by george on 1/28/21.
//

#ifndef RAM_MOTION_PLANNING_HELPERS_H
#define RAM_MOTION_PLANNING_HELPERS_H
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <ram_interfaces/srv/get_toolpath.hpp>
#include <ram_interfaces/msg/toolpath.hpp>
#include <utility>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <iostream>

class ToolpathHelper{
public:
    explicit ToolpathHelper(rclcpp::Node::SharedPtr  node);
    ToolpathHelper();

    bool load_toolpath();
    bool get_toolpath(ram_interfaces::msg::Toolpath &toolpath);

private:
    rclcpp::Node::SharedPtr  node_;
    rclcpp::executors::SingleThreadedExecutor executor_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_load_toolpath_;
    rclcpp::Client<ram_interfaces::srv::GetToolpath>::SharedPtr client_get_toolpath_;
};

class StockHelper{
public:
    explicit StockHelper(rclcpp::Node::SharedPtr &node);
    StockHelper();

    bool load_stock(bool load);
    bool attach_stock(bool attach);
private:
    rclcpp::Node::SharedPtr  node_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client_load_stock_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client_attach_stock_;
};

class GripperHelper{
public:
    GripperHelper();
    explicit GripperHelper(rclcpp::Node::SharedPtr  &node);
    bool gripper(bool open);

private:
    rclcpp::Node::SharedPtr  node_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_gripper_open_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_gripper_close_;
};

std::ostream& operator<<(std::ostream& os, const geometry_msgs::msg::Point32& point);

std::ostream& operator<<(std::ostream& os, const ram_interfaces::msg::Toolpath& toolpath);


#endif //RAM_MOTION_PLANNING_HELPERS_H
