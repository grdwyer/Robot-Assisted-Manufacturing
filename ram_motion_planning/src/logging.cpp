//
// Created by george on 6/8/22.
//

#include <ram_motion_planning/logging.h>

std::ostream& operator<<(std::ostream& os, const geometry_msgs::msg::Point32& point)
{
    os << "X:" << point.x << ", Y: " << point.y << ", Z: " << point.z;
    return os;
}

std::ostream& operator<<(std::ostream& os, const geometry_msgs::msg::Point& point)
{
    os << "X:" << point.x << ", Y: " << point.y << ", Z: " << point.z;
    return os;
}

std::ostream& operator<<(std::ostream& os, const ram_interfaces::msg::Toolpath& toolpath)
{
    os << "Frame: " << toolpath.header.frame_id << "\nPoints: \n";
    for(const auto &point : toolpath.path.points ){
        os << "\t" << point << "\n";
    }

    return os;
}

std::ostream& operator<<(std::ostream& os, const std::vector<geometry_msgs::msg::Pose>& waypoints)
{
    os << "Waypoint positions: \n";
    for(const auto &pose : waypoints ){
        os << "\t" << pose.position << "\n";
    }

    return os;
}

std::ostream& operator<<(std::ostream& os, const KDL::Frame& frame)
{
    os << "\t" << "X:" << frame.p.x() << ", Y: " << frame.p.y() << ", Z: " << frame.p.z();
    return os;
}

std::ostream& operator<<(std::ostream& os, const std::vector<KDL::Frame>& waypoints)
{
    os << "Positions: \n";
    for(const auto &frame : waypoints ){
        os << frame << "\n";
    }

    return os;
}

std::ostream& operator<<(std::ostream& os, const geometry_msgs::msg::TransformStamped& trans)
{
    os << "Parent: " << trans.header.frame_id << "\nChild: " << trans.child_frame_id <<
       "\nPosition: \n\tX: " << trans.transform.translation.x << "\n\tY: " << trans.transform.translation.y <<
       "\n\tZ: " << trans.transform.translation.z << "\nOrientation: \n\tX: " << trans.transform.rotation.x <<
       "\n\tY: " << trans.transform.rotation.y << "\n\tZ: " << trans.transform.rotation.z << "\n\tW: " <<
       trans.transform.rotation.w << "\n";

    return os;
}

std::ostream& operator<<(std::ostream& os, const sensor_msgs::msg::JointState & joint_state){
    os << std::endl;
    for( const auto &name : joint_state.name){
        os << name << "\t";
    }
    os << std::endl;
    for( const auto &position : joint_state.position){
        os << position << "\t";
    }
    os << std::endl;
    return os;
}

