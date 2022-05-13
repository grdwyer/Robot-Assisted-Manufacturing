#include "behaviortree_cpp_v3/bt_factory.h"
#include "behavior_tree/BtService.hpp"
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <moveit/collision_detection/collision_matrix.h>

class ModifyACMComponent : public BtService<moveit_msgs::srv::ApplyPlanningScene>{
public:
    ModifyACMComponent(const std::string & name, const BT::NodeConfiguration & config)
            : BtService<moveit_msgs::srv::ApplyPlanningScene>(name, config) {
        client_get_planning_scene_ = _node->create_client<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene");
    }

    static BT::PortsList providedPorts()
    {
        return providedBasicPorts({BT::InputPort<std::string>("first"),
                BT::InputPort<std::string>("second"),
                        BT::InputPort<bool>("allow")});
    }

    moveit_msgs::srv::ApplyPlanningScene::Request::SharedPtr populate_request() override{
        auto request_apply = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
        if(client_get_planning_scene_->wait_for_service(std::chrono::seconds(5))) {
            //get current scene
            auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
            request->components.components = moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;
            auto future = client_get_planning_scene_->async_send_request(request);
            auto status = rclcpp::spin_until_future_complete(_node, future);

            if (status == rclcpp::FutureReturnCode::SUCCESS && future.valid()) {
                //convert to acm object
                collision_detection::AllowedCollisionMatrix acm(future.get()->scene.allowed_collision_matrix);

                //set value of acm given
                if (acm.hasEntry(*getInput<std::string>("first"), *getInput<std::string>("second"))) {
                    acm.setEntry(*getInput<std::string>("first"), *getInput<std::string>("second"), *getInput<bool>("allow"));
                };

                //apply planning scene

                request_apply->scene.is_diff = true;
                moveit_msgs::msg::AllowedCollisionMatrix msg;
                acm.getMessage(msg);
                request_apply->scene.allowed_collision_matrix = msg;

            }
        }
        return request_apply; //TODO: if this fails it will give an empty request which should fail but may not
    }

    BT::NodeStatus handle_response(moveit_msgs::srv::ApplyPlanningScene::Response::SharedPtr response) override
    {
        if(response->success){
            RCLCPP_INFO(_node->get_logger(),  "ACM successfully modified");
            return BT::NodeStatus::SUCCESS;
        } else{
            RCLCPP_WARN(_node->get_logger(),  "ACM could not be modified");
            return BT::NodeStatus::FAILURE;
        }
    }

private:
    rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr client_get_planning_scene_;
};

BT_REGISTER_NODES(factory) {
    factory.registerNodeType<ModifyACMComponent>("ModifyingACMComponent");
}