
#include <ram_motion_planning/toolpath_follower.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("toolpath_follower");


ToolpathFollower::ToolpathFollower(const rclcpp::NodeOptions & options): Node("toolpath_follower", options){

    auto move_group_node = this->create_sub_node("");
    move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(move_group_node, "iiwa");
    move_group_->setMaxVelocityScalingFactor(0.1);
    auto toolpath_node = this->create_sub_node("toolpath");
    auto stock_node = this->create_sub_node("stock");
    auto gripper_node = this->create_sub_node("gripper");
    toolpath_helper_ = std::make_shared<ToolpathHelper>();
    stockHelper_ = std::make_shared<StockHelper>(stock_node);
    gripperHelper_ = std::make_shared<GripperHelper>(gripper_node);

    // TODO: put this into node namespace
    service_setup_ = this->create_service<std_srvs::srv::Trigger>("/toolpath_setup", std::bind(&ToolpathFollower::callback_setup, this, std::placeholders::_1, std::placeholders::_2));
    service_execute_ = this->create_service<std_srvs::srv::Trigger>("/toolpath_execute", std::bind(&ToolpathFollower::callback_execute, this, std::placeholders::_1, std::placeholders::_2));

    rclcpp::sleep_for(std::chrono::seconds(2));
}

bool ToolpathFollower::load_toolpath() {
    // load the toolpath
    RCLCPP_INFO(LOGGER, "Loading toolpath");
    auto success =toolpath_helper_->load_toolpath();
    success = toolpath_helper_->get_toolpath(toolpath_);
    RCLCPP_INFO_STREAM(LOGGER, "Received toolpath with " << toolpath_.path.points.size() << " points");
    return success;
}

bool ToolpathFollower::construct_plan_request() {
    RCLCPP_INFO(LOGGER, "Constructing request");
    RCLCPP_INFO_STREAM(LOGGER, "using toolpath: \n" << toolpath_);

    move_group_->setPoseReferenceFrame("cutting_tool_tip");
    move_group_->setEndEffectorLink("gripper_jaw_centre");

    // Flip the pose about the x axis to have the gripper upside down
    geometry_msgs::msg::Pose pose;
    pose.orientation.x = 1.0;
    pose.orientation.w = 6e-17;
    std::vector<geometry_msgs::msg::Pose> waypoints;
    for(const auto &point : toolpath_.path.points){
        pose.position.x = (double) point.x;
        pose.position.y = (double) point.y;
        pose.position.z = (double) point.z;

        waypoints.push_back(pose);
    }

    const double jump_threshold = 0.0;
    const double eef_step = 0.001;
    double fraction = move_group_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory_toolpath_);
    RCLCPP_INFO(LOGGER, "Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);
    return true;
}

void ToolpathFollower::display_planned_trajectory() {

}

bool ToolpathFollower::move_to_setup() {
    move_group_->setPoseReferenceFrame("cutting_tool_tip");
    move_group_->setEndEffectorLink("gripper_jaw_centre");

    // Flip the pose about the x axis to have the gripper upside down
    geometry_msgs::msg::Pose pose;
    pose.orientation.x = 1.0;
    pose.orientation.w = 6e-17;
    pose.position.x = -0.04;
    move_group_->setPoseTarget(pose);

    // TODO: remove this, it will be handled by the manager
    stockHelper_->load_stock(true);
    stockHelper_->attach_stock(true);
    gripperHelper_->gripper(false);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    bool success = (move_group_->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if(success){
        if(move_group_->execute(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS){
            return true;
        }
    } else{
        return false;
    }
    return false;
}

bool ToolpathFollower::execute_trajectory() {
    if (!trajectory_toolpath_.joint_trajectory.points.empty()){
        bool success = (move_group_->execute(trajectory_toolpath_) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        return success;
    } else{
        RCLCPP_WARN(LOGGER, "Toolpath trajectory is empty, try construct the plan request first.");
        return false;
    }
}

void ToolpathFollower::callback_setup(const std_srvs::srv::Trigger::Request::SharedPtr request,
                                      std_srvs::srv::Trigger::Response::SharedPtr response) {
    if(load_toolpath()){
        if(move_to_setup()){
            if(construct_plan_request()){
                response->success = true;
                response->message = "Toolpath fully setup";
            } else{
                response->success = false;
                response->message = "Toolpath failed to plan";
            }
        } else{
            response->success = false;
            response->message = "Unable to move to setup";
        }
    } else{
        response->success = false;
        response->message = "Unable to load toolpath";
    }

}

void ToolpathFollower::callback_execute(const std_srvs::srv::Trigger::Request::SharedPtr request,
                                        std_srvs::srv::Trigger::Response::SharedPtr response) {
    if(execute_trajectory()){
        response->success = true;
        response->message = "executed toolpath";
    } else{
        response->success = false;
        response->message = "failed to execute toolpath";
    }
}

int main(int argc, char** argv)
{
    RCLCPP_INFO(LOGGER, "Initialize node");
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    // This enables loading undeclared parameters
    // best practice would be to declare parameters in the corresponding classes
    // and provide descriptions about expected use
    node_options.automatically_declare_parameters_from_overrides(true);


    auto node = std::make_shared<ToolpathFollower>(node_options);

////    std::thread run_demo([&toolpath_follower]() {
//        // TODO: use lifecycle events to launch node
//        toolpath_follower.load_toolpath();
//        rclcpp::sleep_for(std::chrono::seconds(2));
//        toolpath_follower.move_to_setup();
//        rclcpp::sleep_for(std::chrono::seconds(2));
//        toolpath_follower.construct_plan_request();
//        rclcpp::sleep_for(std::chrono::seconds(4));
//
//
////    });
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();

    return 0;
}
