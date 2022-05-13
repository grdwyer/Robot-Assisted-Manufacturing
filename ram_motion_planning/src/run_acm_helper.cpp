
#include <rclcpp/rclcpp.hpp>
#include <ram_motion_planning/helpers.h>

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    auto node = rclcpp::Node::make_shared("acm_helper");
    ACMHelper helper(node);
    if(helper.set_acm_entry("cutting_plate_base", "gripper_link_left", true)){
        RCLCPP_INFO_STREAM(rclcpp::get_logger("acm_helper"), "Success");
    };

    rclcpp::shutdown();
    return 0;
}


//int main(int argc, char **argv)
//{
//    rclcpp::init(argc, argv);
//    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("acm_helper");
//    rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr client =
//            node->create_client<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene");
//
//    auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
//    request->components.components = moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;
//
//    while (!client->wait_for_service(std::chrono::seconds(1))) {
//        if (!rclcpp::ok()) {
//            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
//            return 0;
//        }
//        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
//    }
//
//    auto result = client->async_send_request(request);
//    // Wait for the result.
//    if (rclcpp::spin_until_future_complete(node, result) ==
//        rclcpp::FutureReturnCode::SUCCESS)
//    {
//        collision_detection::AllowedCollisionMatrix acm(result.get()->scene.allowed_collision_matrix);
//        acm.print(std::cout);
//    } else {
//        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service");
//    }
//
//    rclcpp::shutdown();
//    return 0;
//}
