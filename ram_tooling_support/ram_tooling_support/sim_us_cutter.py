import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
import std_srvs.srv
from daq_interfaces.srv import SetGPIOValues, ModifyGPIOSetup


class SimUSCutterController(Node):
    def __init__(self):
        super().__init__('us_cutter_controller')

        # two service servers with empty requests for open and close.
        self.service_enable = self.create_service(std_srvs.srv.SetBool, '{}/enable'.format(self.get_name()),
                                                  self.callback_enable)

        self.service_activate = self.create_service(std_srvs.srv.SetBool, '{}/activate'.format(self.get_name()),
                                                  self.callback_activate)

        self.enable = False
        self.activate = False

        self.get_logger().info("US Cutter Controller started up")

    def __del__(self):
        self.get_logger().warn("Shutting down, setting us activate to off")

    def callback_activate(self, request: std_srvs.srv.SetBool.Request, response: std_srvs.srv.SetBool.Response):
        """
        Service callback to activate the US cutter
        :param request: Service request
        :type request: std_srvs.srv.SetBool.Request
        :param response: returned response
        :type response: std_srvs.srv.SetBool.Response
        :return: acknowledgement it was processed (not successful)
        """
        if request.data:
            self.get_logger().info("Request to activate US cutter received")
            if self.enable:
                self.activate = True
            else:
                self.get_logger().warn("US cutter has not been enabled, enable first to allow activation")
                response.success = False
                return response
        else:
            self.get_logger().info("Request to deactivate US cutter received")
            self.activate = True
        response.success = True
        return response

    def callback_enable(self, request: std_srvs.srv.SetBool.Request, response: std_srvs.srv.SetBool.Response):
        """
        Service callback to enable the US cutter (not activate the blade)
        :param request: Service request
        :type request: std_srvs.srv.SetBool.Request
        :param response: returned response
        :type response: std_srvs.srv.SetBool.Response
        :return: acknowledgement it was processed (not successful)
        """
        self.enable = request.data
        response.success = True
        return response


def main(args=None):
    rclpy.init(args=args)

    controller = SimUSCutterController()
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
