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

    // Flip the pose about the x axis to have the gripper upside down
    // Tool base - flipped on the x (should be paramed), reorient - rotation to face the next point, tool pose - resultant pose to convert to pose msg
    KDL::Frame tool_base, reorient, tool_pose;
    KDL::Vector diff;
    geometry_msgs::msg::Pose pose;
    std::vector<geometry_msgs::msg::Pose> waypoints;
    std::vector<KDL::Frame> ee_toolpath, ee_cartesian_path;

    // transform toolpath to ee frame
    tool_base = get_ee_to_stock_transform();

    for(const auto &point : toolpath_.path.points){
        ee_toolpath.push_back(tool_base * KDL::Frame(KDL::Vector((double)point.x, (double)point.y, (double)point.z)));
    }
    RCLCPP_INFO_STREAM(LOGGER, "EE Toolpath: \n" << ee_toolpath);

    tf2_ros::TransformBroadcaster broadcaster(this);
    // Run the marker across each untransformed
    for(const auto &frame : ee_toolpath){
        tf_trans = tf2::kdlToTransform(frame);
        tf_trans.header.frame_id = move_group_->getEndEffectorLink();
        tf_trans.header.stamp = this->get_clock()->now();
        tf_trans.child_frame_id = "ee_toolpath_point";
        broadcaster.sendTransform(tf_trans);
        rclcpp::sleep_for(std::chrono::milliseconds(this->get_parameter("debug_wait_time").as_int()));
    }

    for(unsigned long i=0; i < ee_toolpath.size(); i++){
        if(i != ee_toolpath.size()-1){
            //determine orientation to go to next point
            diff = ee_toolpath[i+1].p - ee_toolpath[i].p;
            reorient = KDL::Frame(KDL::Rotation::RotZ(-atan2(diff.y(), diff.x())));
        }

        tool_pose = ee_toolpath[i] * reorient;

        // Debug tf frames
        // Frame on the implant
        tf_trans = tf2::kdlToTransform(tool_pose);
        tf_trans.header.frame_id = move_group_->getEndEffectorLink();
        tf_trans.header.stamp = this->get_clock()->now();
        tf_trans.child_frame_id = "ee_toolpath_point";
        broadcaster.sendTransform(tf_trans);

        //Frame part to tool
        tf_trans = tf2::kdlToTransform(tool_pose.Inverse());
        tf_trans.header.frame_id = move_group_->getPoseReferenceFrame();
        tf_trans.header.stamp = this->get_clock()->now();
        tf_trans.child_frame_id = "planned_ee_pose";
        broadcaster.sendTransform(tf_trans);
        rclcpp::sleep_for(std::chrono::milliseconds(this->get_parameter("debug_wait_time").as_int()));

        pose = tf2::toMsg(tool_pose.Inverse());
        waypoints.push_back(pose);
    }

    // Determine an approach pose for the toolpath
    geometry_msgs::msg::Pose approach_pose;
    KDL::Frame initial_path_frame, approach_frame;
    tf2::fromMsg(waypoints[0], initial_path_frame);
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
    tf2::fromMsg(waypoints.back(), end_path_frame);
    retreat_frame = end_path_frame * KDL::Frame(KDL::Vector(0.01,0,0)); // TODO: param this
    tf_trans = tf2::kdlToTransform(retreat_frame);
    tf_trans.header.frame_id = move_group_->getPoseReferenceFrame();
    tf_trans.header.stamp = this->get_clock()->now();
    tf_trans.child_frame_id = "retreat_frame";
    broadcaster.sendTransform(tf_trans);

    //Add to the waypoints vector for now, look into a nicer way of doing this during the cleanup possibly check the stock size and see if the toolpath already includes the retreat.
    retreat_pose = tf2::toMsg(retreat_frame);
    waypoints.push_back(retreat_pose);
    rclcpp::sleep_for(std::chrono::milliseconds(this->get_parameter("debug_wait_time").as_int()));

    approach_pose = tf2::toMsg(approach_frame);
    moveit::planning_interface::MoveGroupInterface::Plan approach_plan;
    move_group_->setPoseTarget(approach_pose);
    if(move_group_->plan(approach_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS){
        move_group_->execute(approach_plan);
    }
    stockHelper_->modify_touch_link("cutting_tool", true);

    rclcpp::sleep_for(std::chrono::milliseconds(this->get_parameter("debug_wait_time").as_int()));

    //TODO: Set start state for cartesian planning
    RCLCPP_INFO_STREAM(LOGGER, "Cartesian planning for toolpath using Waypoints: \n" << waypoints);
    const double jump_threshold = 0.0;
    const double eef_step = 0.001;
    double fraction = move_group_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory_toolpath_);
    RCLCPP_INFO(LOGGER, "Visualizing Cartesian path (%.2f%% acheived)", fraction * 100.0);
    return fraction > 0.99;
}