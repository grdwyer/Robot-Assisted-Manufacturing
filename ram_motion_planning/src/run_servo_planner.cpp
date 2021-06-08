//
// Created by george on 4/14/21.
//

#include <ram_motion_planning/servo_toolpath_planner.h>

//int main(int argc, char** argv)
//{
//    rclcpp::init(argc, argv);
//    rclcpp::NodeOptions node_options;
//
//    auto toolpath_follower = std::make_shared<OMPLToolpathPlanner>(node_options);
//
//    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
//    executor.add_node(toolpath_follower);
//    executor.spin();
//    rclcpp::shutdown();
//
//    return 0;
//}

static const rclcpp::Logger LOG = rclcpp::get_logger("run_servo");

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;

    auto toolpath_follower = std::make_shared<ServoToolpathPlanner>(node_options);

    std::thread run_demo([&toolpath_follower]() {
        rclcpp::sleep_for(std::chrono::seconds(3));
        if(toolpath_follower->load_toolpath()){
            RCLCPP_INFO_STREAM(LOG, "Toolpath loaded");
        } else{
            RCLCPP_WARN_STREAM(LOG, "Toolpath failed to load");
        }
//        rclcpp::sleep_for(std::chrono::seconds(1));

        if(toolpath_follower->move_to_setup()){
            RCLCPP_INFO_STREAM(LOG, "Manipulator moved to setup");
        } else{
            RCLCPP_WARN_STREAM(LOG, "Manipulator failed to move to setup");
        }
        rclcpp::sleep_for(std::chrono::seconds(1));

        if(toolpath_follower->construct_plan_request()){
            RCLCPP_INFO_STREAM(LOG, "Plan request constructed");
        } else{
            RCLCPP_WARN_STREAM(LOG, "Plan request failed to construct");
        }
        rclcpp::sleep_for(std::chrono::seconds(1));

        if(toolpath_follower->execute_trajectory()){
            RCLCPP_INFO_STREAM(LOG, "trajectory executed");
        } else{
            RCLCPP_WARN_STREAM(LOG, "trajectory failed to execute");
        };

    });

    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
    executor.add_node(toolpath_follower);
    executor.spin();

    run_demo.join();
    rclcpp::shutdown();

    return 0;
}