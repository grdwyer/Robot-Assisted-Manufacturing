//
// Created by george on 3/30/21.
//

#include <ram_motion_planning/servo_toolpath_planner.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("servo_toolpath_planner");

ServoToolpathPlanner::ServoToolpathPlanner(const rclcpp::NodeOptions & options) : BaseToolpathPlanner(options){
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/visualization_marker", 10);
    servo_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("target_pose", 1 /* queue */);

    service_setup_ = this->create_service<std_srvs::srv::Trigger>(this->get_fully_qualified_name() + std::string("/toolpath_setup"),
                                                                  std::bind(&ServoToolpathPlanner::callback_setup, this, std::placeholders::_1, std::placeholders::_2));
    service_execute_ = this->create_service<std_srvs::srv::Trigger>(this->get_fully_qualified_name() + std::string("/toolpath_execute"),
                                                                    std::bind(&ServoToolpathPlanner::callback_execute, this, std::placeholders::_1, std::placeholders::_2));

    auto servo_node = this->create_sub_node("servo");
    servo_helper_ = std::make_shared<ServoHelper>();

    this->declare_parameter("time_between_points", 0.1);
    move_group_->startStateMonitor(5.0);
}

bool ServoToolpathPlanner::construct_plan_request() {
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

//    std::vector<geometry_msgs::msg::Pose> waypoints;
    geometry_msgs::msg::PoseStamped pose_stamped;
    std::string ref_frame = this->get_parameter("tool_reference_frame").as_string();

    pose_stamped.pose = tf2::toMsg(approach_frame);
    pose_stamped.header.frame_id = ref_frame;
    pose_trajectory_.push_back(pose_stamped);

    for(const auto & frame : ee_cartesian_path){
        pose_stamped.pose = tf2::toMsg(frame);
        pose_stamped.header.frame_id = ref_frame;
        pose_trajectory_.push_back(pose_stamped);
    }

    pose_stamped.pose = tf2::toMsg(retreat_frame);
    pose_stamped.header.frame_id = ref_frame;
    pose_trajectory_.push_back(pose_stamped);

    return true;
}

bool ServoToolpathPlanner::execute_trajectory() {
    double time_between_points = this->get_parameter("time_between_points").as_double();
    rclcpp::Rate loop_rate(1./time_between_points);

    servo_helper_->enable_servo();
    auto pose = pose_trajectory_.front();

    // Send a couple to start.
    for(int i = 0; i < 10; i++){
        pose.header.stamp = this->now();
        servo_pose_pub_->publish(pose);
        loop_rate.sleep();
    }

    RCLCPP_INFO_STREAM(LOGGER, "starting trajectory\ncontrol rate: " << 1./time_between_points);
    for(auto &  desired_pose : pose_trajectory_){
        desired_pose.header.stamp = this->now();
        servo_pose_pub_->publish(desired_pose);
        loop_rate.sleep();
    }

    return true;
}

void ServoToolpathPlanner::callback_setup(std_srvs::srv::Trigger::Request::SharedPtr request,
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

void ServoToolpathPlanner::callback_execute(std_srvs::srv::Trigger::Request::SharedPtr request,
                                           std_srvs::srv::Trigger::Response::SharedPtr response) {
    if(execute_trajectory()){
        response->success = true;
        response->message = "executed toolpath";
    } else{
        response->success = false;
        response->message = "failed to execute toolpath";
    }
}





