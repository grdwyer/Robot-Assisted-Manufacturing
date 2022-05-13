#include "behaviortree_cpp_v3/bt_factory.h"
#include "behavior_tree/BtService.hpp"
#include <ram_interfaces/srv/set_touch_links.hpp>

class ModifyStockTouchLinksComponent : public BtService<ram_interfaces::srv::SetTouchLinks>{
public:
    ModifyStockTouchLinksComponent(const std::string & name, const BT::NodeConfiguration & config)
            : BtService<ram_interfaces::srv::SetTouchLinks>(name, config) {}

    static BT::PortsList providedPorts()
    {
        return providedBasicPorts({BT::InputPort<std::string>("link"),BT::InputPort<bool>("allow")});
    }
    ram_interfaces::srv::SetTouchLinks::Request::SharedPtr populate_request() override{
        auto request = std::make_shared<ram_interfaces::srv::SetTouchLinks::Request>();
        request->links.push_back(*getInput<std::string>("link"));
        request->modify.push_back(*getInput<bool>("allow"));
        return request;
    }

    BT::NodeStatus handle_response(ram_interfaces::srv::SetTouchLinks::Response::SharedPtr response) override
    {
        if(response->success){
            RCLCPP_INFO(_node->get_logger(),  "Modify stock touch links component request completed\n" + response->message);
            return BT::NodeStatus::SUCCESS;
        } else{
            RCLCPP_WARN(_node->get_logger(),  "Modify stock touch links component request failed\n" + response->message);
            return BT::NodeStatus::FAILURE;
        }
    }
};

BT_REGISTER_NODES(factory) {
    factory.registerNodeType<ModifyStockTouchLinksComponent>("ModifyStockTouchLinksComponent");
}