import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import Pose, Vector3, Point, Polygon, Point32
from ram_interfaces.srv import GetToolpath
from ram_interfaces.msg import Toolpath
from visualization_msgs.msg import Marker
from std_srvs.srv import SetBool, Trigger
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from ament_index_python.packages import get_package_share_directory
import yaml
import os


def make_point(x, y=None, z=None, msg_type=Point):
    point = msg_type()

    if isinstance(x, list) and y is None and z is None:
        point.x = float(x[0])
        point.y = float(x[1])
        point.z = float(x[2])

    else:
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

        # qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
        #                  reliability=QoSReliabilityPolicy.RELIABLE)
        self.pub_marker = self.create_publisher(Marker, "/{}/marker_toolpath".format(self.get_name()), 10)

        qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)
        self.srv_toolpath = self.create_service(GetToolpath, "/{}/get_toolpath".format(self.get_name()),
                                                self.callback_get_toolpath)
        self.declare_parameter("toolpath_frame", "implant")

        # Toolpath file
        default_path = os.path.join(get_package_share_directory("ram_tooling_support"), "config/test_toolpath_straight_x.yaml")
        self.declare_parameter("toolpath_file", default_path)
        self.toolpath_config = None
        self.toolpath_msg = Polygon()

        self.timer_rviz_display = self.create_timer(0.2, self.callback_timer_marker_publish)
        self.timer_rviz_display.cancel()

        self.srv_load_toolpath = self.create_service(Trigger, "/{}/load_toolpath".format(self.get_name()),
                                                     self.callback_load_toolpath)
        self.srv_display_toolpath = self.create_service(SetBool, "/{}/display_toolpath".format(self.get_name()),
                                                        self.callback_display_toolpath)

    def callback_display_toolpath(self, request, response):
        if request.data is True:
            # start the timer callback
            if self.timer_rviz_display.is_canceled():  # Make sure it is already stopped
                self.timer_rviz_display.reset()
            response.success = True
            response.message = "Timer reset, toolpath will be displayed"
        else:  # stop the timer
            if not self.timer_rviz_display.is_canceled():
                self.timer_rviz_display.cancel()

            self.send_marker_delete_msg()
            response.success = True
            response.message = "Timer cancelled, will stop displaying toolpath"
        return response

    def send_marker_delete_msg(self):
        msg = Marker()
        msg.header.frame_id = self.get_parameter("toolpath_frame").get_parameter_value().string_value
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.id = 0
        msg.action = Marker.DELETE
        self.pub_marker.publish(msg)

    def callback_load_toolpath(self, request, response):
        self.load_toolpath_file()

        if self.toolpath_config is not None:

            response.success = True
            response.message = "Loaded toolpath from {}, containing {} points".format(
                self.get_parameter("toolpath_file").get_parameter_value().string_value,
                len(self.toolpath_config["cut"]["points"]))
        else:
            response.success = False
            response.message = "Attempted to load toolpath from {} but failed".format(
                self.get_parameter("toolpath_file").get_parameter_value().string_value)
        return response

    def load_toolpath_file(self):
        with open(self.get_parameter("toolpath_file").get_parameter_value().string_value) as file:
            self.toolpath_config = yaml.load(file, Loader=yaml.FullLoader)

    def callback_timer_marker_publish(self):
        self.get_logger().info("subcriber_count: {}".format(self.pub_marker.get_subscription_count()))
        msg = self.create_rviz_marker()
        self.pub_marker.publish(msg)

    def create_toolpath_message(self):
        msg = Toolpath()
        if self.toolpath_config is not None:  # if initialised, there's a better way to check
            msg.header.frame_id = self.get_parameter("toolpath_frame").get_parameter_value().string_value
            msg.header.stamp = self.get_clock().now().to_msg()

            points = self.toolpath_config["cut"]["points"]
            for path_point in points:
                point = make_point(path_point, msg_type=Point32)
                msg.path.points.append(point)

        else:
            self.get_logger().warn("Toolpath has not been loaded. Empty toolpath will be returned")
        return msg

    def callback_get_toolpath(self, request, response):
        if len(request.id) > 0:
            param = rclpy.Parameter('toolpath_file', rclpy.Parameter.Type.STRING, request.id)
            self.set_parameters([param])
            self.load_toolpath_file()
        response.toolpath = self.create_toolpath_message()

        return response

    def create_rviz_marker(self):
        msg = Marker()
        msg.header.frame_id = self.get_parameter("toolpath_frame").get_parameter_value().string_value
        msg.header.stamp = self.get_clock().now().to_msg()

        # msg.ns = "cut"
        msg.id = 0

        msg.type = Marker.LINE_STRIP
        msg.action = Marker.ADD
        msg.pose.orientation.w = 1.0

        # TODO: make these params
        msg.scale.x = 0.003

        msg.color.a = 1.0
        msg.color.r = 1.0

        # msg.lifetime.sec = 0
        # msg.frame_locked = True

        if self.toolpath_config is not None:  # if initialised, there's a better way to check
            points = self.toolpath_config["cut"]["points"]
            for path_point in points:
                point = make_point(path_point)
                msg.points.append(point)
        return msg

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

        msg.scale.x = 0.003

        msg.color.a = 1.0
        msg.color.r = 1.0

        # msg.lifetime.sec = 1
        # msg.frame_locked = True

        start = make_point(-0.02, 0.01, 0.001)
        mid = make_point(0., 0.015, 0.001)
        end = make_point(0.01, 0.01, 0.001)
        #
        # start = make_point(-1, 1, 1)
        # mid = make_point(0., 1.5, 1)
        # end = make_point(1, 1, 1)

        msg.points.append(start)
        msg.points.append(mid)
        msg.points.append(end)
        return msg


def main(args=None):
    rclpy.init(args=args)

    handler = ToolPathHandler()
    handler.get_logger().info("Toolpath handler started up")

    try:
        rclpy.spin(handler)
    except KeyboardInterrupt:
        pass

    handler.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
