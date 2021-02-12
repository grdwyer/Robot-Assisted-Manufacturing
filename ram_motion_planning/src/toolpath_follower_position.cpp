
#include <ram_motion_planning/toolpath_follower.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("toolpath_follower");


ToolpathFollower::ToolpathFollower(const rclcpp::NodeOptions & options): Node("toolpath_follower", options),
                                    buffer_(this->get_clock()){

    auto move_group_node = this->create_sub_node("");
    move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(move_group_node, "iiwa");
    move_group_->setMaxVelocityScalingFactor(0.1);
    auto toolpath_node = this->create_sub_node("toolpath");
    auto stock_node = this->create_sub_node("stock");
    auto gripper_node = this->create_sub_node("gripper");
    toolpath_helper_ = std::make_shared<ToolpathHelper>();
    stockHelper_ = std::make_shared<StockHelper>(stock_node);
    gripperHelper_ = std::make_shared<GripperHelper>(gripper_node);

    // TODO: put service names into node namespace
    service_setup_ = this->create_service<std_srvs::srv::Trigger>("/toolpath_setup", std::bind(&ToolpathFollower::callback_setup, this, std::placeholders::_1, std::placeholders::_2));
    service_execute_ = this->create_service<std_srvs::srv::Trigger>("/toolpath_execute", std::bind(&ToolpathFollower::callback_execute, this, std::placeholders::_1, std::placeholders::_2));
    publisher_toolpath_poses_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/toolpath_follower/planned_toolpath", 10);

    //TF2
    tfl_ = std::make_shared<tf2_ros::TransformListener>(buffer_);
    rclcpp::sleep_for(std::chrono::seconds(2));
}

bool ToolpathFollower::load_toolpath() {
    // load the toolpath
    RCLCPP_INFO(LOGGER, "Loading toolpath");
    auto success =toolpath_helper_->load_toolpath();
    toolpath_helper_->get_toolpath(toolpath_);
    return success;
}

bool ToolpathFollower::construct_plan_request() {
    RCLCPP_INFO(LOGGER, "Constructing request");
    RCLCPP_INFO_STREAM(LOGGER, "using toolpath: \n" << toolpath_);

    move_group_->setPoseReferenceFrame("cutting_tool_tip");
    move_group_->setEndEffectorLink("gripper_jaw_centre");

    // Get TF from ee to stock frame
    geometry_msgs::msg::TransformStamped stock_pose, tf_trans;

    try {
        stock_pose = buffer_.lookupTransform(move_group_->getEndEffectorLink(), "implant", tf2::TimePoint());
        RCLCPP_INFO_STREAM(LOGGER, "Found stock to ee transform\n" << stock_pose);
    } catch (const tf2::TransformException & ex) {
        std::cout << "Failure at " << this->get_clock()->now().seconds() << std::endl;
        std::cout << "Exception thrown:" << ex.what() << std::endl;
        std::cout << "The current list of frames is:" << std::endl;
        std::cout << buffer_.allFramesAsString() << std::endl;
        return false;
    }

    tf2_ros::TransformBroadcaster broadcaster(this);

    // Flip the pose about the x axis to have the gripper upside down
    // Tool base - flipped on the x (should be paramed), reorient - rotation to face the next point, tool pose - resultant pose to convert to pose msg
    KDL::Frame tool_base, reorient, tool_pose;
    KDL::Vector current, next, diff;

    geometry_msgs::msg::Pose pose;
    std::vector<geometry_msgs::msg::Pose> waypoints;
    geometry_msgs::msg::Point32 point, next_point;

    tool_base = tf2::transformToKDL(stock_pose);
    // transform toolpath to ee frame
    std::vector<KDL::Frame> ee_toolpath;
    for(const auto &point : toolpath_.path.points){
        ee_toolpath.push_back(tool_base * KDL::Frame(KDL::Vector((double)point.x, (double)point.y, (double)point.z)));
    }
    RCLCPP_INFO_STREAM(LOGGER, "EE Toolpath: \n" << ee_toolpath);

    // Run the marker across each
    for(const auto &frame : ee_toolpath){
        tf_trans = tf2::kdlToTransform(frame);
        tf_trans.header.frame_id = move_group_->getEndEffectorLink();
        tf_trans.header.stamp = this->get_clock()->now();
        tf_trans.child_frame_id = "ee_toolpath_point";
        broadcaster.sendTransform(tf_trans);
        rclcpp::sleep_for(std::chrono::microseconds (500));
    }

    for(unsigned long i=0; i < ee_toolpath.size(); i++){
//        point = toolpath_.path.points[i];
//        current = KDL::Vector((double)point.x, (double)point.y, (double)point.z);

        if(i != ee_toolpath.size()-1){
            //determine orientation to go to next point
//            next_point = toolpath_.path.points[i+1];
//            next = KDL::Vector((double)next_point.x, (double)next_point.y, (double)next_point.z);
//            tf2::convert(next, next);

            diff = ee_toolpath[i+1].p - ee_toolpath[i].p;
            reorient = KDL::Frame(KDL::Rotation::RotZ(-atan2(diff.y(), diff.x())));

        } else{
          //At the end of the trajectory use the previous orientation

        }

        tool_pose = ee_toolpath[i] * reorient;

        tf_trans = tf2::kdlToTransform(tool_pose);
        tf_trans.header.frame_id = move_group_->getEndEffectorLink();
        tf_trans.header.stamp = this->get_clock()->now();
        tf_trans.child_frame_id = "ee_toolpath_point";
        broadcaster.sendTransform(tf_trans);

        pose = tf2::toMsg(tool_pose.Inverse());
        tf_trans = tf2::kdlToTransform(tool_pose.Inverse());
        tf_trans.header.frame_id = move_group_->getPoseReferenceFrame();
        tf_trans.header.stamp = this->get_clock()->now();
        tf_trans.child_frame_id = "planned_ee_pose";
        broadcaster.sendTransform(tf_trans);
        rclcpp::sleep_for(std::chrono::seconds(4));
        waypoints.push_back(pose);
    }

    RCLCPP_INFO_STREAM(LOGGER, "Waypoints: \n" << waypoints);
//    follow_waypoints_sequentially(waypoints);
//    display_planned_trajectory(waypoints); //TODO: can't get rviz to publish pose array markers
    const double jump_threshold = 0.0;
    const double eef_step = 0.001;
    double fraction = move_group_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory_toolpath_);
    RCLCPP_INFO(LOGGER, "Visualizing Cartesian path (%.2f%% acheived)", fraction * 100.0);
    return true;
}

bool ToolpathFollower::follow_waypoints_sequentially(std::vector<geometry_msgs::msg::Pose> &waypoints) {
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

void ToolpathFollower::display_planned_trajectory(std::vector<geometry_msgs::msg::Pose> &poses) {
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


    auto toolpath_follower = std::make_shared<ToolpathFollower>(node_options);

////    std::thread run_demo([&toolpath_follower]() {
//        // TODO: use lifecycle events to launch node
//        toolpath_follower->load_toolpath();
////        rclcpp::sleep_for(std::chrono::seconds(2));
//        toolpath_follower->move_to_setup();
////        rclcpp::sleep_for(std::chrono::seconds(2));
//        toolpath_follower->construct_plan_request();
//        rclcpp::sleep_for(std::chrono::seconds(2));

////    });
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(toolpath_follower);
    executor.spin();
    rclcpp::shutdown();

    return 0;
}
