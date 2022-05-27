#include "behaviortree_cpp_v3/bt_factory.h"
#include "behavior_tree/BtService.hpp"
#include <ram_interfaces/srv/set_toolpath.hpp>

class SetToolpathComponent : public BtService<ram_interfaces::srv::SetToolpath>{
public:
    SetToolpathComponent(const std::string & name, const BT::NodeConfiguration & config)
            : BtService<ram_interfaces::srv::SetToolpath>(name, config) {}

    static BT::PortsList providedPorts()
    {
        return providedBasicPorts({BT::InputPort<ram_interfaces::msg::Toolpath>("toolpath")});
    }

    ram_interfaces::srv::SetToolpath::Request::SharedPtr populate_request() override{
        auto request = std::make_shared<ram_interfaces::srv::SetToolpath::Request>();
        request->toolpath = *getInput<ram_interfaces::msg::Toolpath>("toolpath");
        return request;
    }

    BT::NodeStatus handle_response(ram_interfaces::srv::SetToolpath::Response::SharedPtr response) override
    {
        if(response->success){
            RCLCPP_INFO_STREAM(_node->get_logger(),  "Set Toolpath request to " << _server_name << " completed\n" << response->message);
            return BT::NodeStatus::SUCCESS;
        } else{
            RCLCPP_WARN_STREAM(_node->get_logger(),  "Set Toolpath request to " << _server_name << " failed\n" << response->message);
            return BT::NodeStatus::FAILURE;
        }
    }
};

BT_REGISTER_NODES(factory) {
    factory.registerNodeType<SetToolpathComponent>("SetToolpathComponent");
}
