//
// Created by george on 1/28/21.
//

#include <ram_motion_planning/helpers.h>

#include <utility>

ToolpathHelper::ToolpathHelper(rclcpp::Node::SharedPtr node) {
    node_ = std::move(node);
    client_get_toolpath_ = node_->create_client<ram_interfaces::srv::GetToolpath>("/toolpath_handler/get_toolpath");
    client_load_toolpath_ = node_->create_client<std_srvs::srv::Trigger>("/toolpath_handler/load_toolpath");
}

ToolpathHelper::ToolpathHelper() {
    node_ = rclcpp::Node::make_shared("toolpath_helper");
    client_get_toolpath_ = node_->create_client<ram_interfaces::srv::GetToolpath>("/toolpath_handler/get_toolpath");
    client_load_toolpath_ = node_->create_client<std_srvs::srv::Trigger>("/toolpath_handler/load_toolpath");
}

bool ToolpathHelper::load_toolpath() {
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

    using ServiceResponseFuture =
    rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture;
    auto response_received_callback = [this](ServiceResponseFuture future) {
        auto result = future.get();
        RCLCPP_INFO_STREAM(rclcpp::get_logger("Toolpath helper"), "Loaded toolpath: " << result->success);
    };
    auto result = client_load_toolpath_->async_send_request(request, response_received_callback);

    return true;
}

bool ToolpathHelper::get_toolpath(ram_interfaces::msg::Toolpath &toolpath) {
    auto request = std::make_shared<ram_interfaces::srv::GetToolpath::Request>();

    auto result = client_get_toolpath_->async_send_request(request);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(node_, result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        toolpath = result.get()->toolpath;
        RCLCPP_INFO_STREAM(rclcpp::get_logger("Toolpath helper"), "Received toolpath");
        return true;

    } else {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("Toolpath helper"), "Failed to call service " << client_get_toolpath_->get_service_name());
        return false;
    }
}


StockHelper::StockHelper(rclcpp::Node::SharedPtr &node) {
    node_ = std::move(node);
    client_load_stock_ = node_->create_client<std_srvs::srv::SetBool>("/stock_handler/load_stock");
    client_attach_stock_ = node_->create_client<std_srvs::srv::SetBool>("/stock_handler/attach_stock");
}

StockHelper::StockHelper() {
    node_ = rclcpp::Node::make_shared("stock_helper");
    client_load_stock_ = node_->create_client<std_srvs::srv::SetBool>("/stock_handler/load_stock");
    client_attach_stock_ = node_->create_client<std_srvs::srv::SetBool>("/stock_handler/attach_stock");
}

bool StockHelper::load_stock(bool load) {
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = load;

    using ServiceResponseFuture =
    rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture;
    auto response_received_callback = [this](ServiceResponseFuture future) {
        auto result = future.get();
        RCLCPP_INFO_STREAM(rclcpp::get_logger("Stock helper"), result.get()->message);
    };
    auto result = client_load_stock_->async_send_request(request, response_received_callback);
    return true;
}

bool StockHelper::attach_stock(bool attach) {
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = attach;

    using ServiceResponseFuture =
    rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture;
    auto response_received_callback = [this](ServiceResponseFuture future) {
        auto result = future.get();
        RCLCPP_INFO_STREAM(rclcpp::get_logger("Stock helper"), result.get()->message);
    };
    auto result = client_attach_stock_->async_send_request(request, response_received_callback);
    return true;
}

GripperHelper::GripperHelper(rclcpp::Node::SharedPtr &node) {
    node_ = std::move(node);
    client_gripper_open_ = node_->create_client<std_srvs::srv::Trigger>("/sim_gripper_controller/open");
    client_gripper_close_ = node_->create_client<std_srvs::srv::Trigger>("/sim_gripper_controller/close");
}

GripperHelper::GripperHelper() {
    node_ = rclcpp::Node::make_shared("toolpath_helper");
    client_gripper_open_ = node_->create_client<std_srvs::srv::Trigger>("/sim_gripper_controller/open");
    client_gripper_close_ = node_->create_client<std_srvs::srv::Trigger>("/sim_gripper_controller/close");
}

bool GripperHelper::gripper(bool open) {
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    std::shared_future<std_srvs::srv::Trigger::Response::SharedPtr> result;

    using ServiceResponseFuture =
    rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture;
    auto response_received_callback = [this](ServiceResponseFuture future) {
        auto result = future.get();
        RCLCPP_INFO_STREAM(rclcpp::get_logger("Gripper helper"), result.get()->message);
    };
    if(open) {
        result = client_gripper_open_->async_send_request(request, response_received_callback);
    } else{
        result = client_gripper_close_->async_send_request(request, response_received_callback);
    }

    return true;
}

std::ostream& operator<<(std::ostream& os, const geometry_msgs::msg::Point32& point)
{
    os << "X:" << point.x << ", Y: " << point.y << ", Z: " << point.z;
    return os;
}

std::ostream& operator<<(std::ostream& os, const ram_interfaces::msg::Toolpath& toolpath)
{
    os << "Frame: " << toolpath.header.frame_id << "\nPoints: \n";
    for(const auto &point : toolpath.path.points ){
        os << "\t" << point << "\n";
    }

    return os;
}