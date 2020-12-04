import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster, TransformListener
from geometry_msgs.msg import Transform


class ImplantHandler(Node):
    def __init__(self):
        super().__init__('implant_handler')
        self.subscription_transform = self.create_subscription(
            Transform,
            'implant_transform',
            self.callback_implant_transform,
            10)

        self.tf_publisher = TransformBroadcaster(self)
        self.implant_transform = Transform()

    def callback_implant_transform(self, msg):
        # lookup transform from msg frame to implant base frame (world)

        self.implant_transform = msg

    def transform_publisher(self):
        self.tf_publisher.sendTransform(self.implant_transform)


def main(args=None):
    rclpy.init(args=args)

    implant_handler = ImplantHandler()

    rclpy.spin(implant_handler)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    implant_handler.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
