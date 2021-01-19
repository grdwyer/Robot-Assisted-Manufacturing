import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Transform
import std_msgs.msg
import PyKDL
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
from shape_msgs.msg import Mesh, MeshTriangle, SolidPrimitive
from geometry_msgs.msg import Pose, Point
import pyassimp
import os
from ament_index_python.packages import get_package_share_directory
import numpy as np


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


def make_mesh_msg(filename):
    scene = pyassimp.load(filename)
    mesh = Mesh()
    for face in scene.meshes[0].faces:
        triangle = MeshTriangle()
        triangle.vertex_indices = np.array(face, 'uint32')
        mesh.triangles.append(triangle)

    for vertex in scene.meshes[0].vertices:
        point = Point()
        point.x = float(vertex[0])
        point.y = float(vertex[1])
        point.z = float(vertex[2])
        mesh.vertices.append(point)
    pyassimp.release(scene)
    return mesh


class StockHandler(Node):
    def __init__(self):
        """
        Node to handle the information and placement associated with the stock material
        Currently this is loading the mesh of the stock material as an attached collision object

        """
        super().__init__('stock_handler')

        self.declare_parameter("base_placement_frame", "optical_table_back_right_bolt")  # Frame for the object to be
        # initialised in
        self.declare_parameter("base_attachment_frame", "gripper_link_left") # Frame for the obejct to be attached to
        # the robot
        self.declare_parameter("stock_frame", "implant")
        self.declare_parameter("stock_description")

        stock_path = os.path.join(get_package_share_directory("ram_support"), 'meshes/stock/collision/medpor_large.stl')
        self.declare_parameter("stock_mesh_path", stock_path)

        # Stock description publisher
        qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)
        self.publisher_stock_description = self.create_publisher(std_msgs.msg.String, 'stock_description', qos)
        msg = std_msgs.msg.String()
        msg.data = self.get_parameter("stock_description").get_parameter_value().string_value
        self.publisher_stock_description.publish(msg)

        # Moveit collision objects
        self.pub_moveit_attached_collision = self.create_publisher(AttachedCollisionObject,
                                                                   "/attached_collision_object", 10)
        self.pub_moveit_collision = self.create_publisher(CollisionObject, "/collision_object", 10)

    def create_moveit_collision_object(self):
        collision_object = CollisionObject()
        collision_object.header.frame_id = self.get_parameter("base_placement_frame").get_parameter_value().string_value
        collision_object.id = self.get_parameter("stock_frame").get_parameter_value().string_value

        # Used for initial test. Ignore it now will be deleted soon.
        # box = SolidPrimitive()
        # box.type = SolidPrimitive.BOX
        # box.dimensions.append(0.1)
        # box.dimensions.append(0.4)
        # box.dimensions.append(0.1)
        #
        # box_pose = Pose()
        # box_pose.position.x = 0.4
        # box_pose.position.y = 0.0
        # box_pose.position.z = 0.1
        #
        # collision_object.primitives.append(box)
        # collision_object.primitive_poses.append(box_pose)

        stock_mesh_path = self.get_parameter("stock_mesh_path").get_parameter_value().string_value
        self.get_logger().info("Path: {}".format(stock_mesh_path))
        stock = make_mesh_msg(stock_mesh_path)

        stock_pose = Pose()
        stock_pose.position.x = 0.00
        stock_pose.position.y = 0.00
        stock_pose.position.z = 0.07985
        stock_pose.orientation.x = 1.0
        stock_pose.orientation.w = 1e-17
        collision_object.meshes.append(stock)
        collision_object.mesh_poses.append(stock_pose)

        collision_object.operation = CollisionObject.ADD

        return collision_object


def main(args=None):
    rclpy.init(args=args)

    stock_handler = StockHandler()
    collision_msg = stock_handler.create_moveit_collision_object()
    # stock_handler.pub_moveit_collision.publish(collision_msg)

    stock_handler.get_logger().info("Currently set to just attach the stock directly at the implant")
    attach_msg = AttachedCollisionObject()
    attach_msg.link_name = stock_handler.get_parameter("base_attachment_frame").get_parameter_value().string_value
    attach_msg.object = collision_msg
    stock_handler.pub_moveit_attached_collision.publish(attach_msg)

    # try:
    #     rclpy.spin(stock_handler)
    # except KeyboardInterrupt:
    #     pass
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    stock_handler.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
