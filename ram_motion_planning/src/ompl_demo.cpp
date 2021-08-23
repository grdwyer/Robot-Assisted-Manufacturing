#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <visualization_msgs/msg/marker.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/macros/console_colors.h>
#include <moveit_msgs/msg/constraints.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("ompl_constrained_planning_demo");
static const std::string PLANNING_GROUP = "iiwa";
static const double PLANNING_TIME_S = 60.0;
static const double PLANNING_ATTEMPTS = 20.0;

class ConstrainedPlanning
{
public:
    ConstrainedPlanning(const rclcpp::Node::SharedPtr& node) : node_(node), marker_count_(0)
    {
        marker_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("/visualization_marker", 10);

        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, PLANNING_GROUP);
        move_group_->setPlanningTime(PLANNING_TIME_S);
        move_group_->setNumPlanningAttempts(PLANNING_ATTEMPTS);

        ref_link_ = move_group_->getPoseReferenceFrame();
        ee_link_ = move_group_->getEndEffectorLink();
        RCLCPP_INFO_STREAM(LOGGER, "Setting ref link as " << ref_link_ << " and end-effector link as " << ee_link_);
    }

    void run(){
        moveToStart();
        // 1. Box Constraints
        planBoxConstraints();
        deleteAllMarkers();

        moveToStart();
        planLineConstraints();
        deleteAllMarkers();
    }

    void moveToStart()
    {
        // Clear previous goals and constraints
        // May have been set previously
        move_group_->clearPoseTargets();
        move_group_->clearPathConstraints();

        RCLCPP_INFO(LOGGER, "moveToStart");

        const moveit_msgs::msg::RobotState goal_state = createRobotState("test_start");
        move_group_->setStartStateToCurrentState();
        move_group_->setJointValueTarget(goal_state.joint_state);
        move_group_->move();
    }

    moveit_msgs::msg::PositionConstraint createBoxConstraint()
    {
        moveit_msgs::msg::PositionConstraint pcm;
        pcm.header.frame_id = ref_link_;
        pcm.link_name = ee_link_;
        pcm.weight = 1.0;

        shape_msgs::msg::SolidPrimitive cbox;
        cbox.type = shape_msgs::msg::SolidPrimitive::BOX;
        cbox.dimensions = { 0.1, 0.4, 0.4 };
        pcm.constraint_region.primitives.emplace_back(cbox);

        geometry_msgs::msg::PoseStamped pose = move_group_->getCurrentPose();

        geometry_msgs::msg::Pose cbox_pose;
        cbox_pose.position.x = pose.pose.position.x;
        cbox_pose.position.y = pose.pose.position.y + 0.15;
        cbox_pose.position.z = pose.pose.position.z - 0.15;
        pcm.constraint_region.primitive_poses.emplace_back(cbox_pose);

        displayBox(cbox_pose, cbox.dimensions);

        return pcm;
    }

    void planBoxConstraints()
    {
        move_group_->clearPoseTargets();
        move_group_->clearPathConstraints();

        RCLCPP_INFO(LOGGER, "planBoxConstraints");

        const moveit_msgs::msg::RobotState start_state = createRobotState("test_start");
        const geometry_msgs::msg::PoseStamped pose_goal = createPoseGoal(0.0, 0.3, -0.3);
        const moveit_msgs::msg::PositionConstraint pcm = createBoxConstraint();

        moveit_msgs::msg::Constraints path_constraints;
        path_constraints.name = "box constraints";
        path_constraints.position_constraints.emplace_back(pcm);

        move_group_->setStartState(start_state);
        move_group_->setPoseTarget(pose_goal);
        move_group_->setPathConstraints(path_constraints);

        moveit::planning_interface::MoveGroupInterface::Plan plan1;
        const bool plan_success = (move_group_->plan(plan1) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        RCLCPP_INFO(LOGGER, "Plan 1 (box constraint) %s", plan_success ? "SUCCEEDED" : "FAILED");
        if (plan_success){
            move_group_->move();
        }

    }

    moveit_msgs::msg::PositionConstraint createLineConstraint()
    {
        moveit_msgs::msg::PositionConstraint pcm;
        pcm.header.frame_id = ref_link_;
        pcm.link_name = ee_link_;
        pcm.weight = 1.0;

        shape_msgs::msg::SolidPrimitive cbox;
        cbox.type = shape_msgs::msg::SolidPrimitive::BOX;

        // For equality constraint set box dimension to: 1e-3 > 0.0005 > 1e-4
        cbox.dimensions = { 0.005, 0.005, 1.0 };
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

        displayBox(cbox_pose, cbox.dimensions);

        return pcm;
    }

    void planLineConstraints()
    {
        move_group_->clearPoseTargets();
        move_group_->clearPathConstraints();

        const moveit_msgs::msg::RobotState start_state = createRobotState("test_start");
        const geometry_msgs::msg::PoseStamped pose_goal = createPoseGoal(0.0, 0.0, -0.1);
        const moveit_msgs::msg::PositionConstraint pcm = createLineConstraint();

        moveit_msgs::msg::Constraints path_constraints;

        // For equality constraints set to: "use_equality_constraints"
        path_constraints.name = "use_equality_constraints";
//        path_constraints.name = "box constraints";

        path_constraints.position_constraints.emplace_back(pcm);

        move_group_->setStartState(start_state);
        move_group_->setPoseTarget(pose_goal);
        move_group_->setPathConstraints(path_constraints);

        moveit::planning_interface::MoveGroupInterface::Plan plan3;
        const bool plan_success = (move_group_->plan(plan3) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        RCLCPP_INFO(LOGGER, "Plan 3 (line equality constraint) %s", plan_success ? "SUCCEEDED" : "FAILED");

        if (plan_success){
            move_group_->move();
        }
    }

    void deleteAllMarkers()
    {
        RCLCPP_INFO(LOGGER, "Delete all markers");

        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = ref_link_;
        marker.header.stamp = node_->now();
        marker.ns = "/";
        marker.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_pub_->publish(marker);
        marker_count_ = 0;
    }

    moveit_msgs::msg::RobotState createRobotState(const std::string& name) const
    {
        RCLCPP_INFO(LOGGER, "createRobotState");

        const std::map<std::string, double> joint_map = move_group_->getNamedTargetValues(name);
        moveit_msgs::msg::RobotState robot_state;
        robot_state.joint_state.header.frame_id = ref_link_;
        robot_state.joint_state.header.stamp = node_->now();

        for (const auto& joint : joint_map)
        {
            RCLCPP_INFO(LOGGER, "Starting State, joint: %s and angle %f:", joint.first.c_str(), joint.second);
            robot_state.joint_state.name.emplace_back(joint.first);
            robot_state.joint_state.position.emplace_back(joint.second);
        }

        return robot_state;
    }

    geometry_msgs::msg::PoseStamped createPoseGoal(double dx, double dy, double dz)
    {
        geometry_msgs::msg::PoseStamped pose = move_group_->getCurrentPose();

        displaySphere(pose.pose, "red");

        pose.pose.position.x += dx;
        pose.pose.position.y += dy;
        pose.pose.position.z += dz;

        displaySphere(pose.pose, "green");

        return pose;
    }

    void displayBox(const geometry_msgs::msg::Pose& pose,
                    const rosidl_runtime_cpp::BoundedVector<double, 3, std::allocator<double>>& dimensions)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = ref_link_;
        marker.header.stamp = node_->now();
        marker.ns = "/";
        marker.id = marker_count_;

        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.lifetime = rclcpp::Duration(0);

        marker.color.a = 0.5;
        marker.pose = pose;
        marker.scale.x = dimensions.at(0);
        marker.scale.y = dimensions.at(1);
        marker.scale.z = dimensions.at(2);

        marker_pub_->publish(marker);
        marker_count_++;
    }

    void displaySphere(const geometry_msgs::msg::Pose& pose, const std::string& color)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = ref_link_;
        marker.header.stamp = node_->now();
        marker.ns = "/";
        marker.id = marker_count_;

        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.lifetime = rclcpp::Duration(0);

        marker.pose = pose;
        marker.scale.x = 0.05;
        marker.scale.y = 0.05;
        marker.scale.z = 0.05;

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
        marker_count_++;
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

    moveit::planning_interface::MoveGroupInterfacePtr move_group_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

    std::string ref_link_, ee_link_;
    unsigned int marker_count_;
};

int main(int argc, char** argv)
{
    RCLCPP_INFO(LOGGER, "Initialize node");
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    // This enables loading undeclared parameters
    // best practice would be to declare parameters in the corresponding classes
    // and provide descriptions about expected use
    node_options.automatically_declare_parameters_from_overrides(true);

    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("run_ompl_constrained_planning", node_options);
    ConstrainedPlanning constrained_planning(node);

    std::thread run_demo([&constrained_planning]() {
        // Wait for rviz
        rclcpp::sleep_for(std::chrono::seconds(1));
        constrained_planning.run();
        RCLCPP_INFO_STREAM(LOGGER, "Finished demo procedure ready to exit");
        return 0;
    });

    rclcpp::spin(node);
    run_demo.join();

    rclcpp::shutdown();
    return 0;
}