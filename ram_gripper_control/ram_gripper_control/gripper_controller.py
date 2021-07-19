import rclpy
from rclpy.node import Node
from rclpy.time import Time
import tf2_ros
from tf2_ros import TransformBroadcaster, TransformListener, Buffer
from geometry_msgs.msg import TransformStamped, Transform
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
from rclpy.action import ActionClient
import builtin_interfaces
import std_srvs.srv
import tf2_kdl
import tf2_py
import PyKDL
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from daq_interfaces.srv import SetGPIOValues, ModifyGPIOSetup


class GripperController(Node):
    def __init__(self):
        super().__init__('gripper_controller')

        # two service servers with empty requests for open and close.
        self.service_open = self.create_service(std_srvs.srv.Trigger, '{}/open'.format(self.get_name()),
                                                self.callback_open)
        self.service_close = self.create_service(std_srvs.srv.Trigger, '{}/close'.format(self.get_name()),
                                                 self.callback_close)
        self.service_attach_implant = self.create_service(std_srvs.srv.Trigger,
                                                          '{}/attach_implant'.format(self.get_name()),
                                                          self.callback_attach_implant)
        self.client_daq_output = self.create_client(SetGPIOValues, "/daq_server/set_output")
        self.client_daq_configure = self.create_client(ModifyGPIOSetup, "/daq_server/configure_pins")

        # publisher for implant handler
        self.publisher_implant_transform = self.create_publisher(TransformStamped, "implant_transform", 10)

        # publisher for gripper joints
        self.publisher_joint_state = self.create_publisher(Float64MultiArray,
                                                           "gripper_forward_command_controller_position/commands", 10)

        self.gripper_open = True

        self.declare_parameter("open_solenoid")
        self.declare_parameter("close_solenoid")

        success = self.client_daq_output.wait_for_service(20)
        if success:
            # configure pins
            request = ModifyGPIOSetup()
            request.pin_numbers.append(self.get_parameter("open_solenoid").get_parameter_value().integer_value)
            request.pin_numbers.append(self.get_parameter("close_solenoid").get_parameter_value().integer_value)
            request.operations.append(ModifyGPIOSetup.Request.OUTPUT)
            request.operations.append(ModifyGPIOSetup.Request.OUTPUT)
            self.client_daq_configure.call_async(request)
        else:
            self.get_logger().error("Wait for service timed out, DAQ server is not running")

        self.get_logger().info("Gripper Controller started up")

    def send_joint_command(self, position):
        msg = Float64MultiArray()
        dim = MultiArrayDimension()
        dim.size = 2
        dim.stride = 1
        msg.layout.dim.append(dim)
        msg.data.append(position)
        msg.data.append(position)
        self.publisher_joint_state.publish(msg)

    def callback_open(self, request, response):
        """
        Service callback to open the gripper
        :param request: Service request
        :type request: std_srvs.srv.Trigger.Request
        :param response: returned response
        :type response: std_srvs.srv.Trigger.Response
        :return: acknowledgement it was processed (not successful)
        """
        self.get_logger().info("request to open gripper received")
        self.send_joint_command(0.004)
        request = SetGPIOValues.Request()
        request.gpio.name.append(self.get_parameter("open_solenoid").get_parameter_value().integer_value)
        request.gpio.value.append(1)
        request.gpio.name.append(self.get_parameter("close_solenoid").get_parameter_value().integer_value)
        request.gpio.value.append(0)
        self.client_daq_output.call_async(request)

        self.gripper_open = True
        response.success = True
        return response

    def callback_close(self, request, response):
        """
        Service callback to close the gripper
        :param request: Service request
        :type request: std_srvs.srv.Trigger.Request
        :param response: returned response
        :type response: std_srvs.srv.Trigger.Response
        :return: acknowledgement it was processed (not successful)
        """
        self.get_logger().info("request to close gripper received")
        self.send_joint_command(0.00)
        request = SetGPIOValues.Request()
        request.gpio.name.append(self.get_parameter("open_solenoid").get_parameter_value().integer_value)
        request.gpio.value.append(0)
        request.gpio.name.append(self.get_parameter("close_solenoid").get_parameter_value().integer_value)
        request.gpio.value.append(1)
        self.client_daq_output.call_async(request)
        self.gripper_open = False
        response.success = True
        return response

    def callback_attach_implant(self, request, response):
        """
        Service callback to attach the implant to the gripper
        :param request: Service request
        :type request: std_srvs.srv.Trigger.Request
        :param response: returned response
        :type response: std_srvs.srv.Trigger.Response
        :return: acknowledgement it was processed (not successful)
        """
        # find offset between
        trans = TransformStamped()
        trans.header.frame_id = "gripper_finger_tip_left"
        trans.header.stamp = self.get_clock().now().to_msg()
        trans.transform.translation.x = 0.001
        trans.transform.translation.y = 0.001
        trans.transform.rotation.x = 1.0
        trans.transform.rotation.w = 1e-17

        self.publisher_implant_transform.publish(trans)
        response.success = True
        return response


def main(args=None):
    rclpy.init(args=args)

    controller = GripperController()
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
