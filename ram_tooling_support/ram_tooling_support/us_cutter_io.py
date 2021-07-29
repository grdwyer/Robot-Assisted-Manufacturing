import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
import std_srvs.srv
from daq_interfaces.srv import SetGPIOValues, ModifyGPIOSetup


class USCutterController(Node):
    def __init__(self):
        super().__init__('us_cutter_controller')

        # two service servers with empty requests for open and close.
        self.service_enable = self.create_service(std_srvs.srv.SetBool, '{}/enable'.format(self.get_name()),
                                                  self.callback_enable)

        self.client_daq_output = self.create_client(SetGPIOValues, "/daq_server/set_output")
        self.client_daq_configure = self.create_client(ModifyGPIOSetup, "/daq_server/configure_pins")

        self.declare_parameter("us_enable", 18)

        success = self.client_daq_output.wait_for_service(20)
        if success:
            # configure pins
            request = ModifyGPIOSetup.Request()
            request.pin_numbers.append(self.get_parameter("us_enable").get_parameter_value().integer_value)
            request.operations.append(ModifyGPIOSetup.Request.OUTPUT)
            self.client_daq_configure.call_async(request)
        else:
            self.get_logger().error("Wait for service timed out, DAQ server is not running")

        self.get_logger().info("US Cutter Controller started up")

    def __del__(self):
        self.get_logger().warn("Shutting down, setting us activate to off")
        request = SetGPIOValues.Request()
        request.gpio.name.append(self.get_parameter("us_enable").get_parameter_value().integer_value)
        request.gpio.value.append(0)
        self.client_daq_output.call_async(request)

    def callback_enable(self, request: std_srvs.srv.SetBool.Request, response: std_srvs.srv.SetBool.Response):
        """
        Service callback to open the gripper
        :param request: Service request
        :type request: std_srvs.srv.SetBool.Request
        :param response: returned response
        :type response: std_srvs.srv.SetBool.Response
        :return: acknowledgement it was processed (not successful)
        """
        gpio_request = SetGPIOValues.Request()
        gpio_request.gpio.name.append(self.get_parameter("us_enable").get_parameter_value().integer_value)

        if request.data:
            self.get_logger().info("Request to activate US cutter received")
            gpio_request.gpio.value.append(1)
        else:
            self.get_logger().info("Request to deactivate US cutter received")
            gpio_request.gpio.value.append(0)
        self.client_daq_output.call_async(gpio_request)
        response.success = True
        return response


def main(args=None):
    rclpy.init(args=args)

    controller = USCutterController()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
