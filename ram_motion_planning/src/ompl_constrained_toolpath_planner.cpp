//
// Created by george on 3/30/21.
//

#include <ram_motion_planning/ompl_constrained_toolpath_planner.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("ompl_toolpath_planner");

bool OMPLToolpathPlanner::construct_plan_request() {
    // Needs to create a trajectory in trajectory_toolpath_ and modify the touch link to
    RCLCPP_INFO_STREAM(LOGGER, "Constructing request\n\tusing toolpath: " << toolpath_ <<
                                                                          "\n\tTool reference frame: " << this->get_parameter("tool_reference_frame").as_string() <<
                                                                          "\n\tEnd-effector reference frame: " << this->get_parameter("end_effector_reference_frame").as_string() <<
                                                                          "\n\tPart reference frame: " << this->get_parameter("part_reference_frame").as_string() << "\n\n");

    move_group_->setPoseReferenceFrame(this->get_parameter("tool_reference_frame").as_string());
    move_group_->setEndEffectorLink(this->get_parameter("end_effector_reference_frame").as_string());

    // Get TF from ee to stock frame
    geometry_msgs::msg::TransformStamped tf_trans;
    std::vector<KDL::Frame> ee_cartesian_path;
    tf2_ros::TransformBroadcaster broadcaster(this);

    // Flip the pose about the x axis to have the gripper upside down
    // Tool base - flipped on the x (should be paramed), reorient - rotation to face the next point, tool pose - resultant pose to convert to pose msg

    bool success = process_toolpath(ee_cartesian_path);

    // Determine an approach pose for the toolpath
    geometry_msgs::msg::Pose approach_pose;
    KDL::Frame initial_path_frame, approach_frame;
    initial_path_frame = ee_cartesian_path.front();
    approach_frame = KDL::Frame(KDL::Vector(-0.01,0,0)) * initial_path_frame; // TODO: param this
    tf_trans = tf2::kdlToTransform(approach_frame);
    tf_trans.header.frame_id = move_group_->getPoseReferenceFrame();
    tf_trans.header.stamp = this->get_clock()->now();
    tf_trans.child_frame_id = "approach_frame";
    broadcaster.sendTransform(tf_trans);
    rclcpp::sleep_for(std::chrono::milliseconds(this->get_parameter("debug_wait_time").as_int()));

    // Determine an retreat pose for the toolpath
    geometry_msgs::msg::Pose retreat_pose;
    KDL::Frame end_path_frame, retreat_frame;
    end_path_frame = ee_cartesian_path.back();
    retreat_frame = KDL::Frame(KDL::Vector(0.01,0,0)) * end_path_frame; // TODO: param this
    tf_trans = tf2::kdlToTransform(retreat_frame);
    tf_trans.header.frame_id = move_group_->getPoseReferenceFrame();
    tf_trans.header.stamp = this->get_clock()->now();
    tf_trans.child_frame_id = "retreat_frame";
    broadcaster.sendTransform(tf_trans);

    //Add to the waypoints vector for now, look into a nicer way of doing this during the cleanup possibly check the stock size and see if the toolpath already includes the retreat.
    ee_cartesian_path.push_back(retreat_frame);
    rclcpp::sleep_for(std::chrono::milliseconds(this->get_parameter("debug_wait_time").as_int()));

    approach_pose = tf2::toMsg(approach_frame);
    moveit::planning_interface::MoveGroupInterface::Plan approach_plan;
    move_group_->setPoseTarget(approach_pose);
    if(move_group_->plan(approach_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS){
        move_group_->execute(approach_plan);
    }
    stockHelper_->modify_touch_link("cutting_tool", true);

    rclcpp::sleep_for(std::chrono::milliseconds(this->get_parameter("debug_wait_time").as_int()));

    std::vector<geometry_msgs::msg::Pose> waypoints;
    for(const auto & frame : ee_cartesian_path){
        waypoints.push_back(tf2::toMsg(frame));
    }

    //TODO: Set start state for cartesian planning
    RCLCPP_INFO_STREAM(LOGGER, "Cartesian planning for toolpath using Waypoints: \n" << waypoints);
    const double jump_threshold = 0.0;
    const double eef_step = 0.001;
    double fraction = move_group_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory_toolpath_);
    RCLCPP_INFO(LOGGER, "Visualizing Cartesian path (%.2f%% acheived)", fraction * 100.0);
    return fraction > 0.99;
}

bool OMPLToolpathPlanner::plan_between_points(geometry_msgs::msg::Pose &start, geometry_msgs::msg::Pose &end,
                                              moveit::planning_interface::MoveGroupInterface::Plan &plan) {
    return false;
}

bool OMPLToolpathPlanner::append_plans(moveit::planning_interface::MoveGroupInterface::Plan &first,
                                       moveit::planning_interface::MoveGroupInterface::Plan &second) {
    return false;
}
