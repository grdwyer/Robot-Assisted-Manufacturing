#include "behaviortree_cpp_v3/bt_factory.h"
#include "behavior_tree/BtService.hpp"
#include <std_srvs/srv/set_bool.hpp>

class SetBoolComponent : public BtService<std_srvs::srv::SetBool>{
public:
    SetBoolComponent(const std::string & name, const BT::NodeConfiguration & config)
            : BtService<std_srvs::srv::SetBool>(name, config) {}

    static BT::PortsList providedPorts()
    {
        return providedBasicPorts({BT::InputPort<bool>("state")});
    }
    std_srvs::srv::SetBool::Request::SharedPtr populate_request() override{
        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = *getInput<bool>("state");
        return request;
    }

    BT::NodeStatus handle_response(std_srvs::srv::SetBool::Response::SharedPtr response) override
    {
        if(response->success){
            RCLCPP_INFO(_node->get_logger(),  "SetBool component request completed\n" + response->message);
            return BT::NodeStatus::SUCCESS;
        } else{
            RCLCPP_WARN(_node->get_logger(),  "SetBool component request failed\n" + response->message);
            return BT::NodeStatus::FAILURE;
        }
    }
};

BT_REGISTER_NODES(factory) {
        factory.registerNodeType<SetBoolComponent>("SetBoolComponent");
}