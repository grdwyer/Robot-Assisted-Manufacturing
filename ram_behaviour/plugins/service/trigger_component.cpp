#include "behaviortree_cpp_v3/bt_factory.h"
#include "behavior_tree/BtService.hpp"
#include <std_srvs/srv/trigger.hpp>

class TriggerComponent : public BtService<std_srvs::srv::Trigger>{
public:
    TriggerComponent(const std::string & name, const BT::NodeConfiguration & config)
            : BtService<std_srvs::srv::Trigger>(name, config) {}

    std_srvs::srv::Trigger::Request::SharedPtr populate_request() override{
        return std::make_shared<std_srvs::srv::Trigger::Request>();
    }

    BT::NodeStatus handle_response(std_srvs::srv::Trigger::Response::SharedPtr response) override
    {
        if(response->success){
            RCLCPP_INFO_STREAM(_node->get_logger(),  "Trigger component request completed\n" << response->message);
            return BT::NodeStatus::SUCCESS;
        } else{
            RCLCPP_WARN_STREAM(_node->get_logger(),  "Trigger component request failed\n" << response->message);
            return BT::NodeStatus::FAILURE;
        }
    }
};

BT_REGISTER_NODES(factory) {
        factory.registerNodeType<TriggerComponent>("TriggerComponent");
}