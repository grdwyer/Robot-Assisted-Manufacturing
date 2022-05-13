#include "behaviortree_cpp_v3/bt_factory.h"
#include "behavior_tree/BtService.hpp"
#include <ram_interfaces/srv/get_toolpath.hpp>

class GetToolpathComponent : public BtService<ram_interfaces::srv::GetToolpath>{
public:
    GetToolpathComponent(const std::string & name, const BT::NodeConfiguration & config)
            : BtService<ram_interfaces::srv::GetToolpath>(name, config) {}

    static BT::PortsList providedPorts()
    {
        return providedBasicPorts({BT::OutputPort<ram_interfaces::msg::Toolpath>("toolpath")});
    }

    ram_interfaces::srv::GetToolpath::Request::SharedPtr populate_request() override{
        auto request = std::make_shared<ram_interfaces::srv::GetToolpath::Request>();
        return request;
    }

    BT::NodeStatus handle_response(ram_interfaces::srv::GetToolpath::Response::SharedPtr response) override
    {
        if(!response->toolpath.path.points.empty()){
            RCLCPP_INFO_STREAM(_node->get_logger(),  "Get toolpath component returned true with a toolpath " << response->toolpath.path.points.size() << " points long\n");
            setOutput<ram_interfaces::msg::Toolpath>("toolpath", response->toolpath);
            return BT::NodeStatus::SUCCESS;
        } else{
            RCLCPP_WARN(_node->get_logger(),  "Get toolpath component returned false\n");
            return BT::NodeStatus::FAILURE;
        }
    }
};

BT_REGISTER_NODES(factory) {
    factory.registerNodeType<GetToolpathComponent>("GetToolpathComponent");
}
