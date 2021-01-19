import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import Pose, Vector3, Point
from visualization_msgs.msg import Marker
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy


def make_point(x, y, z):
    point = Point()
    point.x = float(x)
    point.y = float(y)
    point.z = float(z)
    return point


class ToolPathHandler(Node):
    def __init__(self):
        """
        Node for processing toolpaths to be displayed in rviz
        """
        super().__init__("toolpath_handler")

        qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
                         reliability=QoSReliabilityPolicy.RELIABLE)
        self.pub_marker = self.create_publisher(Marker, "/toolpath", 10)
        self.declare_parameter("toolpath_frame", "tool_frame")

        self.timer = self.create_timer(1, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info("subcriber_count: {}".format(self.pub_marker.get_subscription_count()))
        msg = self.create_test_marker()
        self.pub_marker.publish(msg)

    def create_test_marker(self):
        msg = Marker()
        msg.header.frame_id = self.get_parameter("toolpath_frame").get_parameter_value().string_value
        msg.header.stamp = self.get_clock().now().to_msg()

        # msg.ns = "cut"
        msg.id = 0

        msg.type = Marker.LINE_STRIP
        msg.action = Marker.ADD
        msg.pose.orientation.w = 1.0
        # msg.pose.orientation.z = 0.70711

        msg.scale.x = 0.1

        msg.color.a = 0.5
        msg.color.r = 1.0

        # msg.lifetime.sec = 1
        # msg.frame_locked = True

        # start = make_point(-0.01, 0.01, 0)
        # mid = make_point(0., 0.015, 0)
        # end = make_point(0.01, 0.01, 0)
        #
        start = make_point(-1, 1, 1)
        mid = make_point(0., 1.5, 1)
        end = make_point(1, 1, 1)

        msg.points.append(start)
        msg.points.append(mid)
        msg.points.append(end)
        return msg


def main(args=None):
    rclpy.init(args=args)

    handler = ToolPathHandler()
    handler.get_logger().info("setup publisher for marker message")

    handler.get_logger().info("sent marker")
    try:
        rclpy.spin(handler)
    except KeyboardInterrupt:
        pass

    handler.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
