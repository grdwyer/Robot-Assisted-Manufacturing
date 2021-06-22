//
// Created by george on 3/30/21.
//

#include <ram_motion_planning/ompl_constrained_toolpath_planner.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("ompl_toolpath_planner");

OMPLToolpathPlanner::OMPLToolpathPlanner(const rclcpp::NodeOptions & options) : BaseToolpathPlanner(options){
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/visualization_marker", 10);
    service_setup_ = this->create_service<std_srvs::srv::Trigger>(this->get_fully_qualified_name() + std::string("/toolpath_setup"),
                                                                  std::bind(&OMPLToolpathPlanner::callback_setup, this, std::placeholders::_1, std::placeholders::_2));
    service_execute_ = this->create_service<std_srvs::srv::Trigger>(this->get_fully_qualified_name() + std::string("/toolpath_execute"),
                                                                    std::bind(&OMPLToolpathPlanner::callback_execute, this, std::placeholders::_1, std::placeholders::_2));

//    move_group_->startStateMonitor(5.0);
////    auto sub_node = this->create_sub_node("state");
////    state_monitor_ = std::make_shared<planning_scene_monitor::CurrentStateMonitor>(sub_node, move_group_->getRobotModel(), std::shared_ptr<tf2_ros::Buffer>());
////    state_monitor_->startStateMonitor("/joint_states");
////    std::this_thread::sleep_for(std::chrono::seconds(2));
////    auto state = state_monitor_->getCurrentState();
////    state->printStateInfo();
////    state->printStatePositions();
}

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

    if(!process_toolpath(ee_cartesian_path)){
        RCLCPP_WARN_STREAM(LOGGER, "Failed to process toolpath, exiting planning stage");
        return false;
    };

    // Determine an approach pose for the toolpath
    geometry_msgs::msg::Pose approach_pose;
    KDL::Frame initial_path_frame, approach_frame;
    initial_path_frame = ee_cartesian_path.front();
    approach_frame = KDL::Frame(KDL::Vector(-0.01,0,0)) * initial_path_frame; // TODO: param this
//    tf_trans = tf2::kdlToTransform(approach_frame);
//    tf_trans.header.frame_id = move_group_->getPoseReferenceFrame();
//    tf_trans.header.stamp = this->get_clock()->now();
//    tf_trans.child_frame_id = "approach_frame";
//    broadcaster.sendTransform(tf_trans);
//    rclcpp::sleep_for(std::chrono::milliseconds(this->get_parameter("debug_wait_time").as_int()));

    // Determine an retreat pose for the toolpath
    geometry_msgs::msg::Pose retreat_pose;
    KDL::Frame end_path_frame, retreat_frame;
    end_path_frame = ee_cartesian_path.back();
    retreat_frame = KDL::Frame(KDL::Vector(0.01,0,0)) * end_path_frame; // TODO: param this
//    tf_trans = tf2::kdlToTransform(retreat_frame);
//    tf_trans.header.frame_id = move_group_->getPoseReferenceFrame();
//    tf_trans.header.stamp = this->get_clock()->now();
//    tf_trans.child_frame_id = "retreat_frame";
//    broadcaster.sendTransform(tf_trans);

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

    std::vector<geometry_msgs::msg::Pose> waypoints, interpolated_waypoints;
    for(const auto & frame : ee_cartesian_path){
        waypoints.push_back(tf2::toMsg(frame));
    }
    interpolate_pose_trajectory(waypoints, 0.0005, tf2Radians(1), interpolated_waypoints);

    std::vector<moveit::planning_interface::MoveGroupInterface::Plan> separated_plans;
    moveit::core::RobotStatePtr start_state;
    moveit::planning_interface::MoveGroupInterface::Plan current_plan;
    geometry_msgs::msg::Pose start_pose;

    for(auto &  desired_pose : interpolated_waypoints){
        if(separated_plans.empty()){
//            RCLCPP_INFO_STREAM(LOGGER, "First point in toolpath loaded, taking current state as start\nState monitor: " << state_monitor_->isActive(););
            RCLCPP_INFO_STREAM(LOGGER, "First point in toolpath loaded, taking current state as start");
//            state_monitor_->waitForCurrentState();
//
//            auto state_time = state_monitor_->getCurrentStateTime();
//            auto time_now = this->get_clock()->now();
//            RCLCPP_INFO_STREAM(LOGGER, "Difference between state and current time: " << (time_now - state_time).seconds());
//            start_state = state_monitor_->getCurrentState();
            start_state = move_group_->getCurrentState(5);
            start_pose = approach_pose;
        }
        else{
            RCLCPP_DEBUG_STREAM(LOGGER, "Using previous plan for start state");
            moveit_msgs::msg::RobotState msg_state = get_end_state_from_plan(separated_plans.back());
            moveit::core::robotStateMsgToRobotState(msg_state, *start_state.get());
        }

        if(plan_between_points(start_state, start_pose, desired_pose, current_plan)){
            RCLCPP_DEBUG_STREAM(LOGGER, "Plan between points succeeded");
            separated_plans.push_back(current_plan);
            start_pose = desired_pose;
        } else{
            RCLCPP_WARN_STREAM(LOGGER, "Failed to plan to desired pose:  \n");// << desired_pose);
            move_group_->clearPoseTargets();
            move_group_->clearPathConstraints();
            return false;
        }
    }
    RCLCPP_INFO_STREAM(LOGGER, "Each section has been planned successfully will now merge the plans");
    auto combined_plans = separated_plans.front();
    separated_plans.erase(separated_plans.begin());
    for(auto & plan : separated_plans){
        append_plans(combined_plans, plan);
    }
    moveit::planning_interface::MoveGroupInterface::Plan retimed_plan;
    robot_state_ = move_group_->getCurrentState(2.0);
    retime_trajectory_constant_velocity(combined_plans, robot_state_, 0.08, retimed_plan);
    trajectory_toolpath_ = retimed_plan.trajectory_;
    delete_markers();
    return true;
}

bool OMPLToolpathPlanner::plan_between_points(const moveit::core::RobotStatePtr& start_state,
                                              geometry_msgs::msg::Pose& start,
                                              geometry_msgs::msg::Pose &end,
                                              moveit::planning_interface::MoveGroupInterface::Plan &plan) {
    delete_markers();
    display_sphere(start, "green", 2);
    display_sphere(end, "red", 3);

    move_group_->clearPoseTargets();
    move_group_->clearPathConstraints();

    moveit_msgs::msg::PositionConstraint pcm;

    pcm.header.frame_id = this->get_parameter("tool_reference_frame").as_string();
    pcm.link_name = this->get_parameter("end_effector_reference_frame").as_string();
    pcm.weight = 1.0;

    shape_msgs::msg::SolidPrimitive cbox;
    cbox.type = shape_msgs::msg::SolidPrimitive::BOX;
    cbox.dimensions = { 0.1, 0.01, 0.01 };
    pcm.constraint_region.primitives.emplace_back(cbox);

//    auto trans_ee = start_state->getGlobalLinkTransform(this->get_parameter("end_effector_reference_frame").as_string());
//    auto trans_base = start_state->getGlobalLinkTransform(this->get_parameter("tool_reference_frame").as_string());
//    auto start_pose = trans_base.inverse() * trans_ee;

    geometry_msgs::msg::Pose cbox_pose = start;

    pcm.constraint_region.primitive_poses.emplace_back(cbox_pose);

    display_line(cbox_pose, cbox.dimensions);

    moveit_msgs::msg::Constraints path_constraints;

    // For equality constraints set to: "use_equality_constraints"
    path_constraints.name = "box constraints";

    path_constraints.position_constraints.emplace_back(pcm);

    move_group_->setStartState(*start_state);
    move_group_->setPoseTarget(end);
//    move_group_->setPathConstraints(path_constraints);

    const bool plan_success = (move_group_->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(LOGGER, "Plan between 2 points %s", plan_success ? "SUCCEEDED" : "FAILED");
    return plan_success;
}

void OMPLToolpathPlanner::display_line(const geometry_msgs::msg::Pose &pose, const rosidl_runtime_cpp::BoundedVector<double, 3, std::allocator<double>> &dimensions) {

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = this->get_parameter("tool_reference_frame").as_string();
    marker.header.stamp = this->now();
    marker.ns = "/";
    marker.id = 1;

    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.lifetime = rclcpp::Duration(0);

    marker.color.a = 0.5;
    marker.pose = pose;
    marker.scale.x = dimensions.at(0);
    marker.scale.y = dimensions.at(1);
    marker.scale.z = dimensions.at(2);

    marker_pub_->publish(marker);
}

void OMPLToolpathPlanner::display_sphere(const geometry_msgs::msg::Pose& pose, const std::string& color, int id)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = this->get_parameter("tool_reference_frame").as_string();
    marker.header.stamp = this->now();
    marker.ns = "/";
    marker.id = id;

    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.lifetime = rclcpp::Duration(0);

    marker.pose = pose;
    marker.scale.x = 0.001;
    marker.scale.y = 0.001;
    marker.scale.z = 0.001;

    if (color == "red")
    {
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
    }
    else if (color == "green")
    {
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
    }
    else
    {
        RCLCPP_ERROR(LOGGER, "Sphere color not specified");
    }

    marker_pub_->publish(marker);
}

void OMPLToolpathPlanner::delete_markers() {
    RCLCPP_INFO(LOGGER, "Delete all markers");

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = this->get_parameter("tool_reference_frame").as_string();
    marker.header.stamp = this->now();
    marker.ns = "/";
    marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_pub_->publish(marker);
}

bool OMPLToolpathPlanner::append_plans(moveit::planning_interface::MoveGroupInterface::Plan &first,
                                       moveit::planning_interface::MoveGroupInterface::Plan &second) {
    // Adding each point from the second trajectory to the first offsetting the time from start
    if(first.trajectory_.joint_trajectory.joint_names.size() == second.trajectory_.joint_trajectory.joint_names.size()){
        auto base_end_time = first.trajectory_.joint_trajectory.points.back().time_from_start;
        auto first_size = first.trajectory_.joint_trajectory.points.size();

        second.trajectory_.joint_trajectory.points.erase(second.trajectory_.joint_trajectory.points.begin());
        auto second_size = second.trajectory_.joint_trajectory.points.size();

        for(auto & point : second.trajectory_.joint_trajectory.points){
            point.time_from_start.sec += base_end_time.sec;
            point.time_from_start.nanosec += base_end_time.nanosec;
            first.trajectory_.joint_trajectory.points.push_back(point);
        }
        if(first.trajectory_.joint_trajectory.points.size() != first_size + second_size){
            RCLCPP_WARN_STREAM(LOGGER, "Combined trajectory size (" << first.trajectory_.joint_trajectory.points.size() <<
                                                                    "is not the sum of the first and second (" << first_size << " and " << second_size << ")" );
        }
        return first.trajectory_.joint_trajectory.points.size() == first_size + second_size;
    }
    else{
        RCLCPP_WARN_STREAM(LOGGER, "Not the same number of joint names in first as the second");
        return false;
    }

}

moveit_msgs::msg::RobotState OMPLToolpathPlanner::get_end_state_from_plan(moveit::planning_interface::MoveGroupInterface::Plan &plan) {
    auto state = moveit_msgs::msg::RobotState();
    state.joint_state.header.frame_id = plan.trajectory_.joint_trajectory.header.frame_id;
    state.joint_state.header.stamp = this->get_clock()->now();
    if(plan.trajectory_.joint_trajectory.joint_names.size() == plan.trajectory_.joint_trajectory.points.back().positions.size()){
        for(const auto & name : plan.trajectory_.joint_trajectory.joint_names){
            state.joint_state.name.emplace_back(name);
        }
        for(const auto & position : plan.trajectory_.joint_trajectory.points.back().positions){
            state.joint_state.position.emplace_back(position);
        }
    }
    return state;
}

void OMPLToolpathPlanner::callback_setup(std_srvs::srv::Trigger::Request::SharedPtr request,
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

void OMPLToolpathPlanner::callback_execute(std_srvs::srv::Trigger::Request::SharedPtr request,
                                           std_srvs::srv::Trigger::Response::SharedPtr response) {
    BaseToolpathPlanner::callback_execute(request, response);
}





