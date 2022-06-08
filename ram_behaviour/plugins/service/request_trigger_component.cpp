#include "behaviortree_cpp_v3/bt_factory.h"
#include "behavior_tree/BtService.hpp"
#include <ram_interfaces/srv/request_trigger.hpp>

class RequestTriggerComponent : public BtService<ram_interfaces::srv::RequestTrigger>{
public:
    RequestTriggerComponent(const std::string & name, const BT::NodeConfiguration & config)
            : BtService<ram_interfaces::srv::RequestTrigger>(name, config) {}

    ram_interfaces::srv::RequestTrigger::Request::SharedPtr populate_request() override{
        auto request = std::make_shared<ram_interfaces::srv::RequestTrigger::Request>();
        return request;
    }

    BT::NodeStatus handle_response(ram_interfaces::srv::RequestTrigger::Response::SharedPtr response) override
    {
        if(response->trigger){
            RCLCPP_DEBUG_STREAM(_node->get_logger(),  "Request Trigger component returned true\n" << response->message);
            return BT::NodeStatus::SUCCESS;
        } else{
            RCLCPP_DEBUG_STREAM(_node->get_logger(),  "Request Trigger component returned false\n" << response->message);
            return BT::NodeStatus::FAILURE;
        }
    }
};

BT_REGISTER_NODES(factory) {
        factory.registerNodeType<RequestTriggerComponent>("RequestTriggerComponent");
}