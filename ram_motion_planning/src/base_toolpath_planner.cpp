
#include <ram_motion_planning/base_toolpath_planner.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("toolpath_planner");


BaseToolpathPlanner::BaseToolpathPlanner(const rclcpp::NodeOptions & options): Node("toolpath_planner", options),
                                                                               buffer_(this->get_clock()){
    // Declare parameters
    setup_parameters();

    // Initialise
    auto move_group_node = std::make_shared<rclcpp::Node>("toolpath_moveit", rclcpp::NodeOptions());
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(move_group_node,
            this->get_parameter("moveit_planning_group").as_string());

    move_group_->setMaxVelocityScalingFactor(this->get_parameter("moveit_scale_velocity").as_double());
    move_group_->setMaxAccelerationScalingFactor(this->get_parameter("moveit_scale_acceleration").as_double());
    move_group_->setPlanningTime(this->get_parameter("moveit_planning_time").as_double());
    move_group_->setNumPlanningAttempts(this->get_parameter("moveit_planning_attempts").as_int());

    auto toolpath_node = this->create_sub_node("toolpath");
    auto stock_node = this->create_sub_node("stock");
    auto gripper_node = this->create_sub_node("gripper");
    auto us_cutter_node = this->create_sub_node("us_cutter");
    toolpath_helper_ = std::make_shared<ToolpathHelper>();
    stock_helper_ = std::make_shared<StockHelper>(stock_node);
    gripper_helper_ = std::make_shared<GripperHelper>(gripper_node);
    us_cutter_helper_ = std::make_shared<USCutterHelper>(us_cutter_node);

    // TODO: put service names into node namespace
    auto planner_node = this->create_sub_node("planner");
    service_setup_ = planner_node->create_service<std_srvs::srv::Trigger>(this->get_fully_qualified_name() + std::string("/toolpath_setup"),
            std::bind(&BaseToolpathPlanner::callback_setup, this, std::placeholders::_1, std::placeholders::_2));
    service_execute_ = planner_node->create_service<std_srvs::srv::Trigger>(this->get_fully_qualified_name() + std::string("/toolpath_execute"),
            std::bind(&BaseToolpathPlanner::callback_execute, this, std::placeholders::_1, std::placeholders::_2));
    publisher_toolpath_poses_ = planner_node->create_publisher<geometry_msgs::msg::PoseArray>(this->get_fully_qualified_name() + std::string("/planned_toolpath"), 10);
    publisher_trajectory_ = planner_node->create_publisher<moveit_msgs::msg::DisplayTrajectory>("/retimed_planned_path", 10);
    //TF2
    tfl_ = std::make_shared<tf2_ros::TransformListener>(buffer_);

    // Executor threads
    executor_moveit_.add_node(move_group_node);
    thread_moveit_executor_ = std::thread(&BaseToolpathPlanner::run_moveit_executor, this);

    rclcpp::sleep_for(std::chrono::seconds(1));

    move_group_->startStateMonitor(2.0);
    robot_state_ = move_group_->getCurrentState(2.0);

    rclcpp::sleep_for(std::chrono::seconds(1));
    configuration_message();
}

void BaseToolpathPlanner::setup_parameters(){
    auto parameter_descriptor = rcl_interfaces::msg::ParameterDescriptor();
    // Move group
    parameter_descriptor.name = "moveit_planning_group";
    parameter_descriptor.description = "The move group that will be used to plan for the manipulator.";
    parameter_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    parameter_descriptor.additional_constraints = "valid group as found in the loaded srdf";
    this->declare_parameter<std::string>("moveit_planning_group", "iiwa", parameter_descriptor);

    // Reference frame
    // These are all quite similar and mostly require a valid frame name
    parameter_descriptor.name = "tool_reference_frame";
    parameter_descriptor.description = "The frame of the tool to be used.";
    parameter_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    parameter_descriptor.additional_constraints = "valid frame within the tf tree";
    this->declare_parameter<std::string>("tool_reference_frame", "cutting_tool_tip", parameter_descriptor);

    parameter_descriptor.name = "end_effector_reference_frame";
    parameter_descriptor.description = "The frame of the end effector to be used.";
    parameter_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    parameter_descriptor.additional_constraints = "valid frame within the tf tree";
    this->declare_parameter<std::string>("end_effector_reference_frame", "gripper_jaw_centre", parameter_descriptor);

    parameter_descriptor.name = "part_reference_frame";
    parameter_descriptor.description = "The frame of the stock to be used.";
    parameter_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    parameter_descriptor.additional_constraints = "must be the stock frame as given by the stock handler";
    this->declare_parameter<std::string>("part_reference_frame", "implant", parameter_descriptor);

    // Debug params
    parameter_descriptor.name = "debug_wait_time";
    parameter_descriptor.description = "Time to wait in milliseconds between each step when in debug mode.";
    parameter_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    parameter_descriptor.additional_constraints = "";
    this->declare_parameter<int>("debug_wait_time", 500, parameter_descriptor);

    parameter_descriptor.name = "debug_mode";
    parameter_descriptor.description = "Outputs extra information and pauses longer in each processing step.";
    parameter_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
    parameter_descriptor.additional_constraints = "";
    this->declare_parameter<bool>("debug_mode", true, parameter_descriptor);

    // Moveit params
    parameter_descriptor.name = "moveit_scale_velocity";
    parameter_descriptor.description = "Scaled joint velocity of moveit plans.";
    parameter_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    parameter_descriptor.additional_constraints = "value has to be between 0 and 1";
    this->declare_parameter<double>("moveit_scale_velocity", 0.2, parameter_descriptor);

    parameter_descriptor.name = "moveit_scale_acceleration";
    parameter_descriptor.description = "Scaled joint acceleration of moveit plans.";
    parameter_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    parameter_descriptor.additional_constraints = "value has to be between 0 and 1";
    this->declare_parameter<double>("moveit_scale_acceleration", 1.0, parameter_descriptor);

    parameter_descriptor.name = "moveit_planning_time";
    parameter_descriptor.description = "Time available to solve the motion plan";
    parameter_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    parameter_descriptor.additional_constraints = "value has to be more than 0 and is in seconds";
    this->declare_parameter<double>("moveit_planning_time", 30.0, parameter_descriptor);

    parameter_descriptor.name = "moveit_planning_attempts";
    parameter_descriptor.description = "Number of attempts available to solve the motion plan";
    parameter_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    parameter_descriptor.additional_constraints = "value has to be more than 0 and is in seconds";
    this->declare_parameter<int>("moveit_planning_attempts", 5, parameter_descriptor);

    // Toolpath processing
    parameter_descriptor.name = "desired_cartesian_velocity";
    parameter_descriptor.description = "Desired cartesian velocity to retime the trajectory to";
    parameter_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    parameter_descriptor.additional_constraints = "value is in m/s and needs to be achievable by the robot";
    this->declare_parameter<double>("desired_cartesian_velocity", 0.08, parameter_descriptor);

    parameter_descriptor.name = "desired_cartesian_acceleration";
    parameter_descriptor.description = "Desired cartesian acceleration to retime the trajectory to";
    parameter_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    parameter_descriptor.additional_constraints = "value is in m/s^2 and needs to be achievable by the robot";
    this->declare_parameter<double>("desired_cartesian_acceleration", 0.2, parameter_descriptor);

    parameter_descriptor.name = "approach_offset";
    parameter_descriptor.description = "Offset distance from the first toolpath point to allow the robot to ramp up before the operation";
    parameter_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    parameter_descriptor.additional_constraints = "value is in m";
    this->declare_parameter<double>("approach_offset", 0.02, parameter_descriptor);

    parameter_descriptor.name = "retreat_offset";
    parameter_descriptor.description = "Offset distance from the last toolpath point to allow the robot to ramp down after the operation";
    parameter_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    parameter_descriptor.additional_constraints = "value is in m";
    this->declare_parameter<double>("retreat_offset", 0.02, parameter_descriptor);

    parameter_descriptor.name = "retreat_height";
    parameter_descriptor.description = "The height to raise the part by at the end of the operation";
    parameter_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    parameter_descriptor.additional_constraints = "value is in m";
    this->declare_parameter<double>("retreat_height", 0.01, parameter_descriptor);

    parameter_descriptor.name = "toolpath_height_offset";
    parameter_descriptor.description = "A value to offset the height of the main toolpath to compensate for the slight differences in the model and tune the downward force applied.";
    parameter_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    parameter_descriptor.additional_constraints = "value is in m";
    this->declare_parameter<double>("toolpath_height_offset", 0.0, parameter_descriptor);
}

void BaseToolpathPlanner::run_moveit_executor(){
    executor_moveit_.spin();
}

void BaseToolpathPlanner::configuration_message() {
    RCLCPP_INFO_STREAM(LOGGER, "Initialising node in " << this->get_fully_qualified_name() <<
    "\n\tMoveit planning group: " << this->get_parameter("moveit_planning_group").as_string() <<
    "\n\tTool reference frame: " << this->get_parameter("tool_reference_frame").as_string() <<
    "\n\tEnd-effector reference frame: " << this->get_parameter("end_effector_reference_frame").as_string() <<
    "\n\tPart reference frame: " << this->get_parameter("part_reference_frame").as_string() <<
    "\n\tMoveit\n\t\tScaled motion velocity: " << this->get_parameter("moveit_scale_velocity").as_double() <<
    "\n\t\tScaled motion acceleration: " << this->get_parameter("moveit_scale_acceleration").as_double() <<
    "\n\t\tPlanning time: " << this->get_parameter("moveit_planning_time").as_double() <<
    "\n\t\tPlanning attempts: " << this->get_parameter("moveit_planning_attempts").as_int() <<
    "\n\tDesired cartesian velocity: " << this->get_parameter("desired_cartesian_velocity").as_double() <<
    "\n\tDesired cartesian acceleration: " << this->get_parameter("desired_cartesian_acceleration").as_double() <<
    "\n\tToolpath height offset: " << this->get_parameter("toolpath_height_offset").as_double() <<
    "\n\tApproach pose offset: " << this->get_parameter("approach_offset").as_double() <<
    "\n\tRetreat pose offset: " << this->get_parameter("retreat_offset").as_double() <<
    "\n\tRetreat pose height: " << this->get_parameter("retreat_height").as_double()
    );
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
    if(!success){
        RCLCPP_ERROR_STREAM(LOGGER, "Toolpath was not processed correctly");
        return false;
    }

    // Determine an approach pose for the toolpath
    geometry_msgs::msg::Pose approach_pose;
    KDL::Frame initial_path_frame, approach_frame;
    initial_path_frame = ee_cartesian_path.front();
    double approach_offset = std::max(this->get_parameter("approach_offset").as_double(), 0.001);
    approach_frame = KDL::Frame(KDL::Vector(-approach_offset,0,0)) * initial_path_frame; // TODO: param this

    if(this->get_parameter("debug_mode").as_bool()) {
        tf_trans = tf2::kdlToTransform(approach_frame);
        tf_trans.header.frame_id = move_group_->getPoseReferenceFrame();
        tf_trans.header.stamp = this->get_clock()->now();
        tf_trans.child_frame_id = "approach_frame";
        broadcaster.sendTransform(tf_trans);
        rclcpp::sleep_for(std::chrono::milliseconds(this->get_parameter("debug_wait_time").as_int()));
    }

    // Determine an retreat pose for the toolpath
    geometry_msgs::msg::Pose retreat_pose;
    KDL::Frame end_path_frame, retreat_frame, raised_retreat_frame;
    double retreat_offset = std::max(this->get_parameter("retreat_offset").as_double(), 0.001);
    double retreat_height = std::max(this->get_parameter("retreat_height").as_double(), 0.001);
    end_path_frame = ee_cartesian_path.back();
    retreat_frame = KDL::Frame(KDL::Vector(retreat_offset/2,0,0)) * end_path_frame;
    raised_retreat_frame = KDL::Frame(KDL::Vector(retreat_offset,0,retreat_height)) * end_path_frame;

    if(this->get_parameter("debug_mode").as_bool()) {
        tf_trans = tf2::kdlToTransform(retreat_frame);
        tf_trans.header.frame_id = move_group_->getPoseReferenceFrame();
        tf_trans.header.stamp = this->get_clock()->now();
        tf_trans.child_frame_id = "retreat_frame";
        broadcaster.sendTransform(tf_trans);
    }

    //Add to the waypoints vector for now, look into a nicer way of doing this during the cleanup possibly check the stock size and see if the toolpath already includes the retreat.
    ee_cartesian_path.push_back(retreat_frame);
    ee_cartesian_path.push_back(raised_retreat_frame);
    stock_helper_->modify_touch_link("cutting_plate", true);
    debug_mode_wait();

    approach_pose = tf2::toMsg(approach_frame);
    moveit::planning_interface::MoveGroupInterface::Plan approach_plan;
    move_group_->setPoseTarget(approach_pose);
    if(move_group_->plan(approach_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS){
        move_group_->execute(approach_plan);
    }
    stock_helper_->modify_touch_link("cutting_tool", true);
    debug_mode_wait();

    std::vector<geometry_msgs::msg::Pose> waypoints, interpolated_waypoints;
    for(const auto & frame : ee_cartesian_path){
        waypoints.push_back(tf2::toMsg(frame));
    }

    interpolate_pose_trajectory(waypoints, 0.0005, tf2Radians(1), interpolated_waypoints);

    //TODO: Set start state for cartesian planning
    RCLCPP_INFO_STREAM(LOGGER, "Cartesian planning for toolpath using Waypoints: \n" << interpolated_waypoints);
    const double jump_threshold = 0.0;
    const double eef_step = 0.001;
    double fraction = move_group_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory_toolpath_);
    RCLCPP_INFO(LOGGER, "Visualizing Cartesian path (%.2f%% acheived)", fraction * 100.0);
    moveit::planning_interface::MoveGroupInterface::Plan plan, retimed_plan;
    robot_state_ = move_group_->getCurrentState(2.0);
    plan.trajectory_ = trajectory_toolpath_;

    RCLCPP_INFO_STREAM(LOGGER, "Retiming trajectory for trapezoidal velocity profile of: \n\tMax Velocity: "
    << this->get_parameter("desired_cartesian_velocity").as_double() << "\n\tMax Acceleration: "
    << this->get_parameter("desired_cartesian_acceleration").as_double());
    retime_trajectory_trapezoidal_velocity(plan, robot_state_,
                                           this->get_parameter("desired_cartesian_velocity").as_double(),
                                           this->get_parameter("desired_cartesian_acceleration").as_double(),
                                           retimed_plan);
//    retime_trajectory_constant_velocity(plan, robot_state_,
//                                       this->get_parameter("desired_cartesian_velocity").as_double(),
//                                       retimed_plan);

    trajectory_toolpath_ = retimed_plan.trajectory_;
    RCLCPP_INFO_STREAM(LOGGER, "Sending retimed trajectory to be displayed");
    moveit_msgs::msg::DisplayTrajectory msg;
    msg.model_id = "iiwa_workcell";
    msg.trajectory.push_back(trajectory_toolpath_);
    moveit::core::robotStateToRobotStateMsg(*robot_state_, msg.trajectory_start);
    publisher_trajectory_->publish(msg);
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
    stock_helper_->load_stock(true);
    stock_helper_->attach_stock(true);
    gripper_helper_->gripper(false);

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
    bool us_cutter = false;
    if(us_cutter_helper_->exists()){
        RCLCPP_WARN_STREAM(LOGGER, "Enabling US cutter");
        us_cutter = true;
        us_cutter_helper_->enable(true);
    }
    if (!trajectory_toolpath_.joint_trajectory.points.empty()){
        bool success = (move_group_->execute(trajectory_toolpath_) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        // Remove tool from touch links
        stock_helper_->modify_touch_link("cutting_tool", false); // TODO: param this
        stock_helper_->modify_touch_link("cutting_plate", false); // TODO: param this
        if(us_cutter){
            RCLCPP_WARN_STREAM(LOGGER, "Disabling US cutter");
            us_cutter_helper_->enable(false);
        }
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
    double height_offset = this->get_parameter("toolpath_height_offset").as_double();

    // transform toolpath to ee frame
    tool_base = get_ee_to_stock_transform();

    for(const auto &point : toolpath_.path.points){
        ee_toolpath.push_back(tool_base * KDL::Frame(KDL::Vector((double)point.x, (double)point.y, (double)point.z - height_offset)));
    }
    RCLCPP_INFO_STREAM(LOGGER, "EE Toolpath: \n" << ee_toolpath);


    for(unsigned long i=0; i < ee_toolpath.size(); i++){
        if(i != ee_toolpath.size()-1){
            //determine orientation to go to next point
            diff = ee_toolpath[i+1].p - ee_toolpath[i].p;
            reorient = KDL::Frame(KDL::Rotation::RotZ(-atan2(diff.y(), diff.x())));
        }

        tool_pose = ee_toolpath[i] * reorient;
        ee_cartesian_path.push_back(tool_pose.Inverse());

        // Debug tf frames
        if(this->get_parameter("debug_mode").as_bool()) {
            tf2_ros::TransformBroadcaster broadcaster(this);
            // Run the marker across each untransformed
            for (const auto &frame : ee_toolpath) {
                tf_trans = tf2::kdlToTransform(frame);
                tf_trans.header.frame_id = move_group_->getEndEffectorLink();
                tf_trans.header.stamp = this->get_clock()->now();
                tf_trans.child_frame_id = "ee_toolpath_point";
                broadcaster.sendTransform(tf_trans);
                rclcpp::sleep_for(std::chrono::milliseconds(this->get_parameter("debug_wait_time").as_int()));
            }
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
        }
    }
    return !ee_cartesian_path.empty() && ee_cartesian_path.size() == toolpath_.path.points.size();
}

void BaseToolpathPlanner::debug_mode_wait() {
    if(this->get_parameter("debug_mode").as_bool()){
        rclcpp::sleep_for(std::chrono::milliseconds(this->get_parameter("debug_wait_time").as_int()));
    }
}


