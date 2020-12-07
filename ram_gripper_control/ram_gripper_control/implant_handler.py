import rclpy
from rclpy.node import Node
from rclpy.time import Time
import tf2_ros
from tf2_ros import TransformBroadcaster, TransformListener, Buffer
from geometry_msgs.msg import TransformStamped, Transform
import builtin_interfaces
import tf2_kdl
import tf2_py
import PyKDL

def to_msg_transform(frame):
    """Convert a PyKDL Vector to a geometry_msgs PointStamped message.
    :param frame: The transform to convert.
    :type frame: PyKDL.Frame
    :return: The converted transform.
    :rtype: Transform
    """
    msg = Transform()
    msg.translation.x = frame.p[0]
    msg.translation.y = frame.p[1]
    msg.translation.z = frame.p[2]

    quat = frame.M.GetQuaternion()
    msg.rotation.x = quat[0]
    msg.rotation.y = quat[1]
    msg.rotation.z = quat[2]
    msg.rotation.w = quat[3]
    return msg

tf2_ros.ConvertRegistration().add_to_msg(PyKDL.Frame, to_msg_transform)

class ImplantHandler(Node):
    def __init__(self):
        super().__init__('implant_handler')
        self.subscription_transform = self.create_subscription(
            TransformStamped,
            'implant_transform',
            self.callback_implant_transform,
            10)

        self.tf_publisher = TransformBroadcaster(self)

        self.implant_transform = TransformStamped()
        self.implant_transform.header.frame_id = "home_a_implant"
        self.implant_transform.transform = to_msg_transform(PyKDL.Frame())
        # TODO: look into why tf2_ros.convert does not work

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.declare_parameter("base_frame", "world")
        self.declare_parameter("implant_frame", "a_implant")

        self.tf_publisher_timer = self.create_timer(1/30.0, self.transform_publisher)

    def callback_implant_transform(self, msg):
        """
        Gets the set implant frame and what it should be relative to.
        :param msg: Transform of implant required
        :type msg: TransformStamped
        :return:
        """
        self.implant_transform = msg

    async def transform_publisher(self):
        # lookup transform from msg frame to implant base frame (world)
        when = rclpy.time.Time()
        try:
            # Suspends callback until transform becomes available
            to_frame = self.implant_transform.header.frame_id
            from_frame = self.get_parameter("base_frame").get_parameter_value().string_value

            transform = await self.tf_buffer.lookup_transform_async(from_frame, to_frame, when)
            self.get_logger().info('Got {}'.format(repr(transform)))
        except tf2_ros.LookupException as e:
            self.get_logger().error('failed to get transform {}'.format(repr(e)))

        # base_trans = tf2_ros.convert(transform.transform, PyKDL.Frame)
        base_trans = tf2_kdl.do_transform_frame(PyKDL.Frame(), transform)
        attach_trans = tf2_kdl.do_transform_frame(PyKDL.Frame(), self.implant_transform)

        implant_transform_msg = TransformStamped()
        now = self.get_clock().now()
        implant_transform_msg.header.stamp = now.to_msg()
        implant_transform_msg.header.frame_id = self.get_parameter("base_frame").get_parameter_value().string_value
        implant_transform_msg.child_frame_id = self.get_parameter("implant_frame").get_parameter_value().string_value

        implant_transform_msg.transform = to_msg_transform(base_trans * attach_trans)
        self.tf_publisher.sendTransform(implant_transform_msg)


def main(args=None):
    rclpy.init(args=args)

    implant_handler = ImplantHandler()
    try:
        rclpy.spin(implant_handler)
    except KeyboardInterrupt:
        pass
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    implant_handler.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
