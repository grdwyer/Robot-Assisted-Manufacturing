//
// Created by george on 4/14/21.
//

#include <ram_motion_planning/base_toolpath_planner.h>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;

    auto toolpath_follower = std::make_shared<BaseToolpathPlanner>(node_options);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(toolpath_follower);
    executor.spin();
    rclcpp::shutdown();

    return 0;
}
