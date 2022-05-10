//
// Created by george on 1/28/21.
//

#include <ram_motion_planning/helpers.h>

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
    auto request_load = std::make_shared<std_srvs::srv::Trigger::Request>();

    using ServiceResponseLoadFuture =
    rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture;
    auto response_received_load_callback = [this](ServiceResponseLoadFuture future) {
        auto result = future.get();
        RCLCPP_INFO_STREAM(rclcpp::get_logger("Toolpath helper"), "Loaded toolpath: " << result->success);
    };
    client_load_toolpath_->async_send_request(request_load, response_received_load_callback);

    auto request_get = std::make_shared<ram_interfaces::srv::GetToolpath::Request>();
    using ServiceResponseGetFuture =
    rclcpp::Client<ram_interfaces::srv::GetToolpath>::SharedFuture;
    auto response_received_get_callback = [this](ServiceResponseGetFuture future) {
        auto result = future.get();
        toolpath_ = result->toolpath;
        RCLCPP_INFO_STREAM(rclcpp::get_logger("Toolpath helper"), "Got toolpath");
    };
    client_get_toolpath_->async_send_request(request_get, response_received_get_callback);

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
//    if(!toolpath_.path.points.empty()){
//        toolpath = toolpath_;
//        return true;
//    } else{
//        RCLCPP_ERROR_STREAM(rclcpp::get_logger("Toolpath helper"), "Toolpath was empty, it probably has not been loaded");
//        return false;
//    }
}


StockHelper::StockHelper(rclcpp::Node::SharedPtr &node) {
    node_ = std::move(node);
    client_load_stock_ = node_->create_client<std_srvs::srv::SetBool>("/stock_handler/load_stock");
    client_attach_stock_ = node_->create_client<std_srvs::srv::SetBool>("/stock_handler/attach_stock");
    client_modify_touch_links_ = node_->create_client<ram_interfaces::srv::SetTouchLinks>("/stock_handler/set_touch_links");
}

StockHelper::StockHelper() {
    node_ = rclcpp::Node::make_shared("stock_helper");
    client_load_stock_ = node_->create_client<std_srvs::srv::SetBool>("/stock_handler/load_stock");
    client_attach_stock_ = node_->create_client<std_srvs::srv::SetBool>("/stock_handler/attach_stock");
    client_modify_touch_links_ = node_->create_client<ram_interfaces::srv::SetTouchLinks>("/stock_handler/set_touch_links");
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

bool StockHelper::modify_touch_link(std::string link_name, bool operation) {
    auto request = std::make_shared<ram_interfaces::srv::SetTouchLinks::Request>();
    request->links.push_back(link_name);
    request->modify.push_back(operation);

    using ServiceResponseFuture =
    rclcpp::Client<ram_interfaces::srv::SetTouchLinks>::SharedFuture;
    auto response_received_callback = [this](ServiceResponseFuture future) {
        auto result = future.get();
        RCLCPP_INFO_STREAM(rclcpp::get_logger("Stock helper"), result.get()->message);
    };
    auto result = client_modify_touch_links_->async_send_request(request, response_received_callback);
    return true;
}

GripperHelper::GripperHelper(rclcpp::Node::SharedPtr &node) {
    node_ = std::move(node);
    client_gripper_open_ = node_->create_client<std_srvs::srv::Trigger>("/gripper_controller/open");
    client_gripper_close_ = node_->create_client<std_srvs::srv::Trigger>("/gripper_controller/close");
}

GripperHelper::GripperHelper() {
    node_ = rclcpp::Node::make_shared("gripper_helper");
    client_gripper_open_ = node_->create_client<std_srvs::srv::Trigger>("/gripper_controller/open");
    client_gripper_close_ = node_->create_client<std_srvs::srv::Trigger>("/gripper_controller/close");
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

ServoHelper::ServoHelper() {
    node_ = rclcpp::Node::make_shared("servo_helper");
    client_enable_servo_ = node_->create_client<std_srvs::srv::Empty>("/enable_servoing");
}

ServoHelper::ServoHelper(rclcpp::Node::SharedPtr &node) {
    node_ = std::move(node);
    client_enable_servo_ = node_->create_client<std_srvs::srv::Empty>("/enable_servoing");
}

bool ServoHelper::enable_servo() {
    auto request = std::make_shared<std_srvs::srv::Empty::Request>();
    std::shared_future<std_srvs::srv::Empty::Response::SharedPtr> result;

    using ServiceResponseFuture =
    rclcpp::Client<std_srvs::srv::Empty>::SharedFuture;
    auto response_received_callback = [this](ServiceResponseFuture future) {
        auto result = future.get();
        RCLCPP_INFO_STREAM(rclcpp::get_logger("Servo helper"), "returned");
    };

    result = client_enable_servo_->async_send_request(request, response_received_callback);
    return true;
}

USCutterHelper::USCutterHelper() {
    node_ = rclcpp::Node::make_shared("us_cutter_helper");
    client_us_enable_ = node_->create_client<std_srvs::srv::SetBool>("/us_cutter_controller/activate");
}

USCutterHelper::USCutterHelper(rclcpp::Node::SharedPtr &node) {
    node_ = std::move(node);
    client_us_enable_ = node_->create_client<std_srvs::srv::SetBool>("/us_cutter_controller/activate");
}

bool USCutterHelper::enable(bool open) {
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = open;

    using ServiceResponseFuture =
    rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture;
    auto response_received_callback = [this](ServiceResponseFuture future) {
        auto result = future.get();
        RCLCPP_INFO_STREAM(rclcpp::get_logger("US cutter helper"), result.get()->message);
    };
    auto result = client_us_enable_->async_send_request(request, response_received_callback);
    return true;
}

bool USCutterHelper::exists() {
    return client_us_enable_->wait_for_service(std::chrono::seconds(1));
}


ACMHelper::ACMHelper() : executor_(rclcpp::ExecutorOptions()) {
    node_ = rclcpp::Node::make_shared("acm_helper");
    client_set_planning_scene_ = node_->create_client<moveit_msgs::srv::ApplyPlanningScene>("/apply_planning_scene");
    client_get_planning_scene_ = node_->create_client<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene");
    executor_.add_node(node_);
}

bool ACMHelper::set_acm_entry(std::string first, std::string second, bool allow) {
    if(client_get_planning_scene_->wait_for_service(std::chrono::seconds(5))) {
        //get current scene
        auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
        request->components.components = moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;
        auto future = client_get_planning_scene_->async_send_request(request);
        auto status = executor_.spin_until_future_complete(future, std::chrono::seconds(5));

        if (status == rclcpp::FutureReturnCode::SUCCESS && future.valid()) {
            auto response = future.get();
            //convert to acm object
            collision_detection::AllowedCollisionMatrix acm(response->scene.allowed_collision_matrix);

            //set value of acm given
            if (acm.hasEntry(first, second)) {
                acm.setEntry(first, second, allow);
            };

            //apply planning scene
            auto request_apply = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
            request_apply->scene.is_diff = true;
            moveit_msgs::msg::AllowedCollisionMatrix msg;
            acm.getMessage(msg);
            request_apply->scene.allowed_collision_matrix = msg;
            auto future_apply = client_set_planning_scene_->async_send_request(request_apply);
            status = executor_.spin_until_future_complete(future_apply, std::chrono::seconds(5));
            return future_apply.get()->success;
        }
        return false;
    }
    else{
        return false;
    }
}

std::ostream& operator<<(std::ostream& os, const geometry_msgs::msg::Point32& point)
{
    os << "X:" << point.x << ", Y: " << point.y << ", Z: " << point.z;
    return os;
}

std::ostream& operator<<(std::ostream& os, const geometry_msgs::msg::Point& point)
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

std::ostream& operator<<(std::ostream& os, const std::vector<geometry_msgs::msg::Pose>& waypoints)
{
    os << "Waypoint positions: \n";
    for(const auto &pose : waypoints ){
        os << "\t" << pose.position << "\n";
    }

    return os;
}

std::ostream& operator<<(std::ostream& os, const std::vector<KDL::Frame>& waypoints)
{
    os << "Positions: \n";
    for(const auto &frame : waypoints ){
        os << "\t" << "X:" << frame.p.x() << ", Y: " << frame.p.y() << ", Z: " << frame.p.z() << "\n";
    }

    return os;
}

std::ostream& operator<<(std::ostream& os, const geometry_msgs::msg::TransformStamped& trans)
{
    os << "Parent: " << trans.header.frame_id << "\nChild: " << trans.child_frame_id <<
    "\nPosition: \n\tX: " << trans.transform.translation.x << "\n\tY: " << trans.transform.translation.y <<
    "\n\tZ: " << trans.transform.translation.z << "\nOrientation: \n\tX: " << trans.transform.rotation.x <<
    "\n\tY: " << trans.transform.rotation.y << "\n\tZ: " << trans.transform.rotation.z << "\n\tW: " <<
    trans.transform.rotation.w << "\n";

    return os;
}
