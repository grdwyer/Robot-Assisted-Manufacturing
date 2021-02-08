
#include <ram_motion_planning/toolpath_follower.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("toolpath_follower");

ToolpathFollower::ToolpathFollower(rclcpp::Node::SharedPtr node): node_(std::move(node)){

    move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(node_, "iiwa");
    move_group_->setMaxVelocityScalingFactor(0.1);
    auto toolpath_node = node_->create_sub_node("toolpath");
    auto stock_node = node_->create_sub_node("stock");
    auto gripper_node = node_->create_sub_node("gripper");
    toolpath_helper_ = std::make_shared<ToolpathHelper>(toolpath_node);
    stockHelper_ = std::make_shared<StockHelper>(stock_node);
    gripperHelper_ = std::make_shared<GripperHelper>(gripper_node);

    rclcpp::sleep_for(std::chrono::seconds(2));
    // TODO: remove this, it will be handled by the manager
    stockHelper_->load_stock(true);
    stockHelper_->attach_stock(true);
    gripperHelper_->gripper(false);
}

bool ToolpathFollower::load_toolpath() {
    // load the toolpath
    RCLCPP_INFO(LOGGER, "Loading toolpath");
    toolpath_helper_->load_toolpath();
    auto success = toolpath_helper_->get_toolpath(toolpath_);
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

    moveit_msgs::msg::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    RCLCPP_INFO(LOGGER, "Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);
//    RCLCPP_INFO_STREAM(LOGGER, "Planned robot trajectory:\n" << trajectory.joint_trajectory.points);

    // You can execute a trajectory like this.
//    move_group_->execute(trajectory);
    return true;
}

void ToolpathFollower::display_planned_trajectory() {

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
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("toolpath_follower", "", node_options);

    ToolpathFollower toolpath_follower(node);

    std::thread run_demo([&toolpath_follower]() {
        // TODO: use lifecycle events to launch node
        toolpath_follower.load_toolpath();
        toolpath_follower.construct_plan_request();
    });

//    rclcpp::spin(node);
    run_demo.join();

    return 0;
}
