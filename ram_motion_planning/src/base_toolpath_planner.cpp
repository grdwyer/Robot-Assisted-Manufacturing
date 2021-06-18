
#include <ram_motion_planning/base_toolpath_planner.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("toolpath_planner");


BaseToolpathPlanner::BaseToolpathPlanner(const rclcpp::NodeOptions & options): Node("toolpath_follower", options),
                                                                               buffer_(this->get_clock()){
    // Declare parameters
    // TODO: add parameter decriptions for each
    this->declare_parameter<std::string>("moveit_planning_group", "iiwa");
    this->declare_parameter<std::string>("tool_reference_frame", "cutting_tool_tip");
    this->declare_parameter<std::string>("end_effector_reference_frame", "gripper_jaw_centre");
    this->declare_parameter<std::string>("part_reference_frame", "implant");
    this->declare_parameter<int>("debug_wait_time", 500);

    // Initialise
    auto move_group_node = this->create_sub_node("");
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(move_group_node,
            this->get_parameter("moveit_planning_group").as_string());
    move_group_->startStateMonitor(2.0);

    auto sub_node = this->create_sub_node("state");
    std::shared_ptr<planning_scene_monitor::CurrentStateMonitor> state_monitor = std::make_shared<planning_scene_monitor::CurrentStateMonitor>(sub_node, move_group_->getRobotModel(), std::shared_ptr<tf2_ros::Buffer>());
    state_monitor->startStateMonitor("/joint_states");
    std::this_thread::sleep_for(std::chrono::seconds(2));
    robot_state_ = state_monitor->getCurrentState();

    move_group_->setMaxVelocityScalingFactor(0.1);
    move_group_->setMaxAccelerationScalingFactor(1.0);
    move_group_->setPlanningTime(30.0);
    move_group_->setNumPlanningAttempts(5);

    auto toolpath_node = this->create_sub_node("toolpath");
    auto stock_node = this->create_sub_node("stock");
    auto gripper_node = this->create_sub_node("gripper");
    toolpath_helper_ = std::make_shared<ToolpathHelper>();
    stockHelper_ = std::make_shared<StockHelper>(stock_node);
    gripperHelper_ = std::make_shared<GripperHelper>(gripper_node);

    // TODO: put service names into node namespace
    service_setup_ = this->create_service<std_srvs::srv::Trigger>(this->get_fully_qualified_name() + std::string("/toolpath_setup"),
            std::bind(&BaseToolpathPlanner::callback_setup, this, std::placeholders::_1, std::placeholders::_2));
    service_execute_ = this->create_service<std_srvs::srv::Trigger>(this->get_fully_qualified_name() + std::string("/toolpath_execute"),
            std::bind(&BaseToolpathPlanner::callback_execute, this, std::placeholders::_1, std::placeholders::_2));
    publisher_toolpath_poses_ = this->create_publisher<geometry_msgs::msg::PoseArray>(this->get_fully_qualified_name() + std::string("/planned_toolpath"), 10);

    //TF2
    tfl_ = std::make_shared<tf2_ros::TransformListener>(buffer_);

    configuration_message();
    rclcpp::sleep_for(std::chrono::seconds(1));
}


void BaseToolpathPlanner::configuration_message() {
    RCLCPP_INFO_STREAM(LOGGER, "Initialising node in " << this->get_fully_qualified_name() <<
    "\n\tMoveit planning group: " << this->get_parameter("moveit_planning_group").as_string() <<
    "\n\tTool reference frame: " << this->get_parameter("tool_reference_frame").as_string() <<
    "\n\tEnd-effector reference frame: " << this->get_parameter("end_effector_reference_frame").as_string() <<
    "\n\tPart reference frame: " << this->get_parameter("part_reference_frame").as_string());
}

bool BaseToolpathPlanner::load_toolpath() {
    // load the toolpath
    RCLCPP_INFO(LOGGER, "Loading toolpath");
    auto success =toolpath_helper_->load_toolpath();
    toolpath_helper_->get_toolpath(toolpath_);
    return success;
}

bool BaseToolpathPlanner::construct_plan_request() {
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
    moveit::planning_interface::MoveGroupInterface::Plan plan, retimed_plan;

    plan.trajectory_ = trajectory_toolpath_;
    retime_trajectory_constant_velocity(plan, robot_state_, 0.08, retimed_plan);
    trajectory_toolpath_ = retimed_plan.trajectory_;
    return fraction > 0.99;
    }

bool BaseToolpathPlanner::follow_waypoints_sequentially(std::vector<geometry_msgs::msg::Pose> &waypoints) {
    for(const auto &pose : waypoints){
        move_group_->setPoseTarget(pose);

        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        if(move_group_->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS){
            rclcpp::sleep_for(std::chrono::seconds(2));
            if(move_group_->execute(my_plan) != moveit::planning_interface::MoveItErrorCode::SUCCESS){
                RCLCPP_WARN_STREAM(LOGGER, "Could not plan to waypoint at " << pose.position);
                return false;
            }
        }
    }
    move_to_setup();
    return true;
}

// Doesn't display with RVIZ at the moment, possibly unneeded.
void BaseToolpathPlanner::display_planned_trajectory(std::vector<geometry_msgs::msg::Pose> &poses) {
    geometry_msgs::msg::PoseArray msg;
    toolpath_poses_.header.frame_id = move_group_->getPoseReferenceFrame();
    toolpath_poses_.header.stamp = rclcpp::Clock().now();

    for(const auto &pose : poses){
        toolpath_poses_.poses.push_back(pose);
    }
    auto timer_callback = [this]() {
        toolpath_poses_.header.stamp = rclcpp::Clock().now();
        publisher_toolpath_poses_->publish(toolpath_poses_);
    };
    timer_toolpath_poses_ = this->create_wall_timer(std::chrono::seconds(1), timer_callback);
}

bool BaseToolpathPlanner::move_to_setup() {
    move_group_->setPoseReferenceFrame(this->get_parameter("tool_reference_frame").as_string());
    move_group_->setEndEffectorLink(this->get_parameter("end_effector_reference_frame").as_string());

    // Flip the pose about the x axis to have the gripper upside down
    geometry_msgs::msg::Pose pose;
    pose.orientation.x = 1.0;
    pose.orientation.w = 6e-17;
    pose.position.x = -0.04;
    pose.position.z = 0.01;
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

bool BaseToolpathPlanner::execute_trajectory() {
    if (!trajectory_toolpath_.joint_trajectory.points.empty()){
        bool success = (move_group_->execute(trajectory_toolpath_) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        // Remove tool from touch links
        stockHelper_->modify_touch_link("cutting_tool", false); // TODO: param this
        return success;
    } else{
        RCLCPP_WARN(LOGGER, "Toolpath trajectory is empty, try construct the plan request first.");
        return false;
    }
}

void BaseToolpathPlanner::callback_setup(const std_srvs::srv::Trigger::Request::SharedPtr request,
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

void BaseToolpathPlanner::callback_execute(const std_srvs::srv::Trigger::Request::SharedPtr request,
                                           std_srvs::srv::Trigger::Response::SharedPtr response) {
    if(execute_trajectory()){
        response->success = true;
        response->message = "executed toolpath";
    } else{
        response->success = false;
        response->message = "failed to execute toolpath";
    }
}

KDL::Frame BaseToolpathPlanner::get_ee_to_stock_transform() {
    geometry_msgs::msg::TransformStamped stock_pose;
    try {
        stock_pose = buffer_.lookupTransform(move_group_->getEndEffectorLink(), this->get_parameter("part_reference_frame").as_string(), tf2::TimePoint());
        RCLCPP_INFO_STREAM(LOGGER, "Found stock to ee transform\n" << stock_pose);
    } catch (const tf2::TransformException & ex) {
        std::cout << "Failure at " << this->get_clock()->now().seconds() << std::endl;
        std::cout << "Exception thrown:" << ex.what() << std::endl;
        std::cout << "The current list of frames is:" << std::endl;
        std::cout << buffer_.allFramesAsString() << std::endl;
    }
    return tf2::transformToKDL(stock_pose);
}

bool BaseToolpathPlanner::process_toolpath(std::vector<KDL::Frame> &ee_cartesian_path) {
    geometry_msgs::msg::TransformStamped tf_trans;
    KDL::Frame tool_base, reorient, tool_pose;
    KDL::Vector diff;
    geometry_msgs::msg::Pose pose;
    std::vector<KDL::Frame> ee_toolpath;

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
//        broadcaster.sendTransform(tf_trans);
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
//        broadcaster.sendTransform(tf_trans);

        //Frame part to tool
        tf_trans = tf2::kdlToTransform(tool_pose.Inverse());
        tf_trans.header.frame_id = move_group_->getPoseReferenceFrame();
        tf_trans.header.stamp = this->get_clock()->now();
        tf_trans.child_frame_id = "planned_ee_pose";
//        broadcaster.sendTransform(tf_trans);
        rclcpp::sleep_for(std::chrono::milliseconds(this->get_parameter("debug_wait_time").as_int()));

        ee_cartesian_path.push_back(tool_pose.Inverse());
    }
    return !ee_cartesian_path.empty() && ee_cartesian_path.size() == toolpath_.path.points.size();
}


