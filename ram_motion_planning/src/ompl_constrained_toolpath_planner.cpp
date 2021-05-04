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

    std::vector<moveit::planning_interface::MoveGroupInterface::Plan> separated_plans;
    moveit::core::RobotStatePtr start_state;
    moveit::planning_interface::MoveGroupInterface::Plan current_plan;

    for(auto &  desired_pose : waypoints){
        if(separated_plans.empty()){
           RCLCPP_DEBUG_STREAM(LOGGER, "First point in toolpath loaded, taking current state as start");
           start_state = move_group_->getCurrentState();
        }
        else{
            RCLCPP_DEBUG_STREAM(LOGGER, "Using previous plan for start state");
            start_state = get_end_state_from_plan(separated_plans.back());
        }
        if(plan_between_points(start_state, desired_pose, current_plan)){
            RCLCPP_DEBUG_STREAM(LOGGER, "Plan between points succeeded");
        }

    }
}

bool OMPLToolpathPlanner::plan_between_points(moveit::core::RobotStatePtr start_state, geometry_msgs::msg::Pose &end,
                                              moveit::planning_interface::MoveGroupInterface::Plan &plan) {
    move_group_->clearPoseTargets();
    move_group_->clearPathConstraints();

    moveit_msgs::msg::PositionConstraint pcm;

    pcm.header.frame_id = this->get_parameter("tool_reference_frame").as_string();
    pcm.link_name = this->get_parameter("end_effector_reference_frame").as_string();
    pcm.weight = 1.0;

    shape_msgs::msg::SolidPrimitive cbox;
    cbox.type = shape_msgs::msg::SolidPrimitive::BOX;

    // For equality constraint set box dimension to: 1e-3 > 0.0005 > 1e-4
    cbox.dimensions = { 0.0005, 0.0005, 1.0 };
    pcm.constraint_region.primitives.emplace_back(cbox);

    geometry_msgs::msg::PoseStamped pose = move_group_->getCurrentPose();

    geometry_msgs::msg::Pose cbox_pose;
    cbox_pose.position = pose.pose.position;

    // turn the constraint region 45 degrees around the x-axis
    tf2::Quaternion quat;
    quat.setRPY(0.0, 0.0, 0.0);

    cbox_pose.orientation.x = quat.x();
    cbox_pose.orientation.y = quat.y();
    cbox_pose.orientation.z = quat.z();
    cbox_pose.orientation.w = quat.w();

    pcm.constraint_region.primitive_poses.emplace_back(cbox_pose);

//    displayBox(cbox_pose, cbox.dimensions);


    moveit_msgs::msg::Constraints path_constraints;

    // For equality constraints set to: "use_equality_constraints"
    path_constraints.name = "use_equality_constraints";

    path_constraints.position_constraints.emplace_back(pcm);

    move_group_->setStartState(*start_state.get());
    move_group_->setPoseTarget(end);
    move_group_->setPathConstraints(path_constraints);

    const bool plan_success = (move_group_->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(LOGGER, "Plan between 2 points %s", plan_success ? "SUCCEEDED" : "FAILED");
    return plan_success;
}

bool OMPLToolpathPlanner::append_plans(moveit::planning_interface::MoveGroupInterface::Plan &first,
                                       moveit::planning_interface::MoveGroupInterface::Plan &second) {
    return false;
}

moveit::core::RobotStatePtr OMPLToolpathPlanner::get_end_state_from_plan(moveit::planning_interface::MoveGroupInterface::Plan &plan) {
    return moveit::core::RobotStatePtr();
}
