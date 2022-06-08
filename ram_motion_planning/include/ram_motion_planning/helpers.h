//
// Created by george on 1/28/21.
//

#ifndef RAM_MOTION_PLANNING_HELPERS_H
#define RAM_MOTION_PLANNING_HELPERS_H
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <ram_interfaces/srv/get_toolpath.hpp>
#include <ram_interfaces/srv/set_touch_links.hpp>
#include <ram_interfaces/msg/toolpath.hpp>
#include <utility>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <iostream>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <kdl/frames.hpp>
#include <utility>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <moveit_msgs/msg/planning_scene_components.hpp>
#include <moveit/collision_detection/collision_matrix.h>

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
    ram_interfaces::msg::Toolpath toolpath_;
};

class StockHelper{
public:
    explicit StockHelper(rclcpp::Node::SharedPtr &node);
    StockHelper();

    bool load_stock(bool load);
    bool attach_stock(bool attach);
    bool modify_touch_link(std::string link_name, bool operation);
private:
    rclcpp::Node::SharedPtr  node_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client_load_stock_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client_attach_stock_;
    rclcpp::Client<ram_interfaces::srv::SetTouchLinks>::SharedPtr client_modify_touch_links_;
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

class USCutterHelper{
public:
    USCutterHelper();
    explicit USCutterHelper(rclcpp::Node::SharedPtr  &node);
    bool enable(bool open);
    bool exists();

private:
    rclcpp::Node::SharedPtr  node_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client_us_enable_;
};

class ServoHelper{
public:
    ServoHelper();
    explicit ServoHelper(rclcpp::Node::SharedPtr  &node);
    bool enable_servo();

private:
    rclcpp::Node::SharedPtr  node_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr client_enable_servo_;
};

class ACMHelper{
public:
    explicit ACMHelper();
    bool set_acm_entry(std::string first, std::string second, bool allow);

private:
    rclcpp::Node::SharedPtr  node_;
    rclcpp::executors::SingleThreadedExecutor executor_;
    rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr client_get_planning_scene_;
    rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr client_set_planning_scene_;

};



#endif //RAM_MOTION_PLANNING_HELPERS_H
