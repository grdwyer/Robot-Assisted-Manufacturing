#include <ram_motion_planning/trajectory_utils.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("trajectory_utilities");

bool append_plans(moveit::planning_interface::MoveGroupInterface::Plan &first,
                  moveit::planning_interface::MoveGroupInterface::Plan &second){
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

geometry_msgs::msg::Pose
interpolate_between_pose(geometry_msgs::msg::Pose start, geometry_msgs::msg::Pose end, double factor) {
    // Split between translation and rotation(quaternion)
    Eigen::Translation<double, 3> t_start, t_end, t_interp;
    Eigen::Quaternion<double> q_start, q_end, q_interp;

    tf2::fromMsg(start.position, t_start.translation());
    tf2::fromMsg(end.position, t_end.translation());
    tf2::fromMsg(start.orientation, q_start);
    tf2::fromMsg(end.orientation, q_end);

    q_interp = q_start.slerp(factor, q_end);

    auto comp_start = t_start.translation() * Eigen::Scaling(1 - factor);
    auto comp_end = t_end.translation() * Eigen::Scaling(factor);
    t_interp = Eigen::Translation<double, 3>(comp_start + comp_end);

    geometry_msgs::msg::Pose p_interp;
    p_interp.orientation = tf2::toMsg(q_interp);
    p_interp.position = tf2::toMsg(t_interp.translation());
    return p_interp;
}

bool retime_trajectory_constant_velocity(moveit::planning_interface::MoveGroupInterface::Plan &plan,
                                         moveit::core::RobotStatePtr robot_state, double desired_velocity,
                                         moveit::planning_interface::MoveGroupInterface::Plan &retimed_plan) {

    const moveit::core::JointModelGroup* joint_model_group = robot_state->getJointModelGroup("iiwa");
    Eigen::Isometry3d p_start, p_end;
    double distance, time_between_points;
    rclcpp::Duration new_time_from_start = rclcpp::Duration(0, 0);
    retimed_plan.trajectory_ = plan.trajectory_;
    retimed_plan.planning_time_ = plan.planning_time_;
    retimed_plan.start_state_ = plan.start_state_;

    for(ulong i = 0; i < plan.trajectory_.joint_trajectory.points.size(); i++){
        if (i > 0) {
            robot_state->setJointGroupPositions(joint_model_group, plan.trajectory_.joint_trajectory.points[i-1].positions);
            p_start = robot_state->getGlobalLinkTransform("gripper_jaw_centre");

            robot_state->setJointGroupPositions(joint_model_group, plan.trajectory_.joint_trajectory.points[i].positions);
            p_end = robot_state->getGlobalLinkTransform("gripper_jaw_centre");

            distance = (p_end.translation() - p_start.translation()).norm();
            time_between_points = distance / desired_velocity;

            new_time_from_start = rclcpp::Duration(retimed_plan.trajectory_.joint_trajectory.points[i-1].time_from_start) + \
                    rclcpp::Duration::from_seconds(time_between_points);

            RCLCPP_DEBUG_STREAM(LOGGER, "Point " << i << " of " << plan.trajectory_.joint_trajectory.points.size()
            << "\nOriginal time from start: " << rclcpp::Duration(retimed_plan.trajectory_.joint_trajectory.points[i].time_from_start).seconds()
            << "\nDistance between points: " << distance << "\nRecalculated time from start: " << new_time_from_start.seconds() << std::endl);

            retimed_plan.trajectory_.joint_trajectory.points[i].time_from_start = new_time_from_start;
        } else{
            // TODO: take the start state and determine distance from start to first point.

        }

    }
    return false;
}

bool retime_trajectory_trapezoidal_velocity(moveit::planning_interface::MoveGroupInterface::Plan &plan,
                                         moveit::core::RobotStatePtr robot_state, double desired_velocity,
                                         double desired_acceleration,
                                         moveit::planning_interface::MoveGroupInterface::Plan &retimed_plan) {

    const moveit::core::JointModelGroup* joint_model_group = robot_state->getJointModelGroup("iiwa");
    Eigen::Isometry3d p_start, p_end;
    double distance, time_between_points;
    rclcpp::Duration new_time_from_start = rclcpp::Duration(0, 0);
    retimed_plan.trajectory_ = plan.trajectory_;
    retimed_plan.planning_time_ = plan.planning_time_;
    retimed_plan.start_state_ = plan.start_state_;

    std::vector<double> distance_between_points, desired_velocities;


    for(ulong i = 0; i < plan.trajectory_.joint_trajectory.points.size(); i++) {
        if (i > 0) {
            robot_state->setJointGroupPositions(joint_model_group,
                                                plan.trajectory_.joint_trajectory.points[i - 1].positions);
            p_start = robot_state->getGlobalLinkTransform("gripper_jaw_centre");

            robot_state->setJointGroupPositions(joint_model_group,
                                                plan.trajectory_.joint_trajectory.points[i].positions);
            p_end = robot_state->getGlobalLinkTransform("gripper_jaw_centre");

            distance_between_points.emplace_back((p_end.translation() - p_start.translation()).norm());
        }
    }

    double acc_time = desired_velocity / desired_acceleration;
    double current_desired_velocity, distance_from_start, distance_to_end, distance_acceleration, total_distance = 0;

    for(const auto &dist : distance_between_points){
        total_distance += dist;
    }
    distance_acceleration = pow(desired_velocity,2) / (2 * desired_acceleration);
    distance_from_start = distance_between_points[0];

    for(ulong i = 0; i < plan.trajectory_.joint_trajectory.points.size(); i++){
        if (i > 0) {
            distance_from_start += distance_between_points[i];
            distance_to_end = total_distance - distance_from_start;

            if(distance_from_start < distance_acceleration){
                current_desired_velocity = sqrt(2 * desired_acceleration * distance_from_start);
                time_between_points = distance / current_desired_velocity;
                desired_velocities.emplace_back(current_desired_velocity);
            }else if(distance_to_end < distance_acceleration){
                current_desired_velocity = sqrt(2 * desired_acceleration * distance_to_end);
                time_between_points = distance / current_desired_velocity;
                desired_velocities.emplace_back(current_desired_velocity);
            } else{
                time_between_points = distance / desired_velocity;
                desired_velocities.emplace_back(desired_velocity);
            }

            new_time_from_start = rclcpp::Duration(retimed_plan.trajectory_.joint_trajectory.points[i-1].time_from_start) + \
                    rclcpp::Duration::from_seconds(time_between_points);

            RCLCPP_DEBUG_STREAM(LOGGER, "Point " << i << " of " << plan.trajectory_.joint_trajectory.points.size()
                                                 << "\nOriginal time from start: " << rclcpp::Duration(retimed_plan.trajectory_.joint_trajectory.points[i].time_from_start).seconds()
                                                 << "\nDistance between points: " << distance << "\nRecalculated time from start: " << new_time_from_start.seconds() << std::endl);

            retimed_plan.trajectory_.joint_trajectory.points[i].time_from_start = new_time_from_start;
        } else{
            // TODO: take the start state and determine distance from start to first point.

        }

    }
    return false;
}

void interpolate_pose_trajectory(std::vector<geometry_msgs::msg::PoseStamped> &original, double max_distance,
                                 double max_angle, std::vector<geometry_msgs::msg::PoseStamped> &interpolated) {
    double dist, angle;
    int interp_size = 10, dist_size, ang_size;
    geometry_msgs::msg::PoseStamped p_start, p_end, p_interp;
    for (int i = 0; i < original.size(); i++) {
        if (i < original.size() - 1) {
            p_start = original[i];
            p_end = original[i + 1];

            // determine the number of poses to interpolate by
            dist = distance_between_poses(p_start.pose, p_end.pose);
            angle = angular_distance_between_poses(p_start.pose, p_end.pose);

            dist_size = ceil(dist/max_distance);
            ang_size = ceil(angle/max_angle);
            interp_size = dist_size > ang_size? dist_size : ang_size;

            for (int j = 0; j < interp_size; j++) {
                p_interp.pose = interpolate_between_pose(p_start.pose, p_end.pose, (double) j / (double) interp_size);
                p_interp.header = p_start.header;
                interpolated.push_back(p_interp);
            }
        } else {
            // Last pose just needs to be added to the list
            interpolated.push_back(original[i]);
        }
    }
}

void interpolate_pose_trajectory(std::vector<geometry_msgs::msg::Pose> &original, double max_distance,
                                 double max_angle, std::vector<geometry_msgs::msg::Pose> &interpolated) {
    double dist, angle;
    int interp_size = 10, dist_size, ang_size;
    geometry_msgs::msg::Pose p_start, p_end, p_interp;
    for (int i = 0; i < original.size(); i++) {
        if (i < original.size() - 1) {
            p_start = original[i];
            p_end = original[i + 1];

            // determine the number of poses to interpolate by
            dist = distance_between_poses(p_start, p_end);
            angle = angular_distance_between_poses(p_start, p_end);

            dist_size = ceil(dist/max_distance);
            ang_size = ceil(angle/max_angle);
            interp_size = dist_size > ang_size? dist_size : ang_size;

            for (int j = 0; j < interp_size; j++) {
                p_interp = interpolate_between_pose(p_start, p_end, (double) j / (double) interp_size);
                interpolated.push_back(p_interp);
            }
        } else {
            // Last pose just needs to be added to the list
            interpolated.push_back(original[i]);
        }
    }
}

double distance_between_poses(geometry_msgs::msg::Pose a, geometry_msgs::msg::Pose b) {
    Eigen::Translation<double, 3> t_a, t_b, t_diff;

    tf2::fromMsg(a.position, t_a.translation());
    tf2::fromMsg(b.position, t_b.translation());

    t_diff = Eigen::Translation<double, 3>(t_b.translation() - t_a.translation());
    return t_diff.translation().norm();
}

double angular_distance_between_poses(geometry_msgs::msg::Pose a, geometry_msgs::msg::Pose b) {
    Eigen::Quaterniond q_a, q_b, q_diff;
    tf2::fromMsg(a.orientation, q_a);
    tf2::fromMsg(b.orientation, q_b);

    q_diff = q_a.inverse() * q_b;

    return Eigen::AngleAxisd(q_diff).angle();
}

double distance_along_trajectory(std::vector<geometry_msgs::msg::PoseStamped> &trajectory) {
    double total_dist = 0;
    geometry_msgs::msg::PoseStamped p_start, p_end;
    for (int i = 0; i < trajectory.size() - 1; i++) {
        p_start = trajectory[i];
        p_end = trajectory[i + 1];
        total_dist += distance_between_poses(p_start.pose, p_end.pose);
    }
    return total_dist;
}
