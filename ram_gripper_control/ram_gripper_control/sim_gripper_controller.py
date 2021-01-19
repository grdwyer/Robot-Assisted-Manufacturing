import rclpy
from rclpy.node import Node
from rclpy.time import Time
import tf2_ros
from tf2_ros import TransformBroadcaster, TransformListener, Buffer
from geometry_msgs.msg import TransformStamped, Transform
from sensor_msgs.msg import JointState
from rclpy.action import ActionClient
import builtin_interfaces
import std_srvs.srv
import tf2_kdl
import tf2_py
import PyKDL
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class SimGripperController(Node):
    def __init__(self):
        super().__init__('sim_gripper_controller')

        # two service servers with empty requests for open and close.
        self.service_open = self.create_service(std_srvs.srv.Trigger, '{}/open'.format(self.get_name()),
                                                self.callback_open)
        self.service_close = self.create_service(std_srvs.srv.Trigger, '{}/close'.format(self.get_name()),
                                                 self.callback_close)
        self.service_attach_implant = self.create_service(std_srvs.srv.Trigger,
                                                          '{}/attach_implant'.format(self.get_name()),
                                                          self.callback_attach_implant)

        # publisher for implant handler
        self.publisher_implant_transform = self.create_publisher(TransformStamped, "implant_transform", 10)

        # publisher for gripper joints
        self.publisher_joint_state = self.create_publisher(JointState, "joint_states", 10)

        self.gripper_open = True

        self.timer_joint_states = self.create_timer(1/30.0, self.callback_gripper_joint_state)
        self.get_logger().info("Sim Gripper Controller started up")

    def callback_gripper_joint_state(self):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name.append("gripper_joint_left")
        # js.name.append("gripper_joint_right")

        # TODO: use the urdf for the joint limits
        if self.gripper_open:
            js.position.append(0.005)  # Switched zero position to be closed for PGN-80 (different to PGN-100 config)
        else:
            js.position.append(0.00)

        self.publisher_joint_state.publish(js)

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

    controller = SimGripperController()
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
