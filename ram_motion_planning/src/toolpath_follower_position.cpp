
#include <ram_motion_planning/toolpath_follower.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("toolpath_follower");

ToolpathFollower::ToolpathFollower(rclcpp::Node::SharedPtr node): node_(std::move(node)), toolpath_helper_(node_){

    moveit_cpp_ = std::make_shared<moveit::planning_interface::MoveItCpp>(node_);
    moveit_cpp_->getPlanningSceneMonitor()->providePlanningSceneService();  // let RViz display query PlanningScene
    moveit_cpp_->getPlanningSceneMonitor()->setPlanningScenePublishingFrequency(100);

    RCLCPP_INFO(LOGGER, "Initialize PlanningComponent");
    arm_ = std::make_unique<moveit::planning_interface::PlanningComponent>("iiwa", moveit_cpp_);
}

bool ToolpathFollower::load_toolpath() {
    // load the toolpath
    toolpath_helper_.load_toolpath();
    return toolpath_helper_.get_toolpath(toolpath_);;
}

bool ToolpathFollower::construct_plan_request() {
    RCLCPP_INFO(LOGGER, "Constructing request");
    RCLCPP_INFO_STREAM(LOGGER, "using toolpath: \n" << toolpath_);
    std::vector<moveit_msgs::msg::Constraints> constraints;

    moveit_msgs::msg::Constraints con;
    moveit_msgs::msg::PositionConstraint p_constrain;
    p_constrain.link_name = "gripper_jaw_centre";
    p_constrain.header.frame_id = "cutting_tool_tip";
    p_constrain.header.stamp = this->node_->get_clock()->now();
    p_constrain.target_point_offset.x = (double) toolpath_.path.points[0].x;
    p_constrain.target_point_offset.y = (double) toolpath_.path.points[0].y;
    p_constrain.target_point_offset.z = (double) toolpath_.path.points[0].z;

    // Doesn't acknowledge this as being a plan.


    con.position_constraints.push_back(p_constrain);

    arm_->setGoal(constraints);
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
