import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Transform
import std_msgs.msg
from std_srvs.srv import SetBool
import PyKDL
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
from shape_msgs.msg import Mesh, MeshTriangle, SolidPrimitive
from geometry_msgs.msg import Pose, Point
import pyassimp
import os
from ament_index_python.packages import get_package_share_directory
import numpy as np


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
        super().__init__('stock_handler', allow_undeclared_parameters=True)

        # Node interfaces
        self.srv_stock_loader = self.create_service(SetBool, '{}/load_stock'.format(self.get_name()), self.callback_load_stock)
        self.srv_attach_stock = self.create_service(SetBool, '{}/attach_stock'.format(self.get_name()),
                                                    self.callback_attach_stock)

        # Node Parameters
        self.setup_params()

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

        self.stock_in_scene = False
        self.stock_attached = False

    def setup_params(self):
        # Frame for the object to be initialised in
        self.declare_parameter("start.frame", "optical_table_back_right_bolt")
        self.declare_parameters("start.pose", [("x", -0.475), ("y", -0.3), ("z", 0.0), ("qx", 0.0), ("qy", 0.0),
                                               ("qz", 0.0), ("qw", 1.0)])

        self.declare_parameter("attach.frame", "gripper_jaw_centre")  # Frame for the object to be attached to
        self.declare_parameters("attach.pose", [("x", 0.0), ("y", 0.0), ("z", 0.001), ("qx", 0.0), ("qy", 1.0),
                                                ("qz", 0.0), ("qw", 6.12e-17)])

        # the robot
        self.declare_parameter("stock_frame", "implant")
        self.declare_parameter("stock_description")

        stock_path = os.path.join(get_package_share_directory("ram_support"), 'meshes/stock/collision/medpor_large.stl')
        self.declare_parameter("stock_mesh_path", stock_path)

    def load_pose_from_param(self, namespace):
        pose = Pose()
        pose.position.x = self.get_parameter("{}.x".format(namespace)).get_parameter_value().double_value
        pose.position.y = self.get_parameter("{}.y".format(namespace)).get_parameter_value().double_value
        pose.position.z = self.get_parameter("{}.z".format(namespace)).get_parameter_value().double_value

        pose.orientation.x = self.get_parameter("{}.qx".format(namespace)).get_parameter_value().double_value
        pose.orientation.y = self.get_parameter("{}.qy".format(namespace)).get_parameter_value().double_value
        pose.orientation.z = self.get_parameter("{}.qz".format(namespace)).get_parameter_value().double_value
        pose.orientation.w = self.get_parameter("{}.qw".format(namespace)).get_parameter_value().double_value
        return pose

    def callback_load_stock(self, req, res):
        """
        Service callback for the stock material to be loaded (or unloaded) in the scene

        :param req: Request with bool stating for the material to be loaded (true) or unloaded (false)
        :type req: std_srvs.srv._set_bool.SetBool_Request
        :param res: Response if the request has been achieved and a message if needed
        :type res: std_srvs.srv._set_bool.SetBool_Response
        :return: if the request is sucessful
        """
        if req.data is True and not self.stock_in_scene:
            #  Load the stock into the scene
            stock_object = self.create_collision_object_with_stock()
            stock_object.header.frame_id = self.get_parameter("start.frame").get_parameter_value().string_value
            stock_object.mesh_poses.append(self.load_pose_from_param("start.pose"))
            self.get_logger().info("Object  Pose: {}".format(stock_object.pose))
            stock_object.operation = CollisionObject.ADD
            self.get_logger().info("Adding stock to the scene")
            self.pub_moveit_collision.publish(stock_object)
            self.stock_in_scene = True

            res.message = "stock added to scene"
            res.success = True

        elif req.data is False and self.stock_in_scene:
            # Remove from scene
            stock_object = CollisionObject()
            stock_object.operation = CollisionObject.REMOVE
            stock_object.id = self.get_parameter("stock_frame").get_parameter_value().string_value
            self.get_logger().info("Removing stock from the scene")
            self.pub_moveit_collision.publish(stock_object)
            self.stock_in_scene = False

            res.message = "stock removed from scene"
            res.success = True

        elif req.data is True and self.stock_in_scene:
            message = "attempting to load stock already in the scene, try unload it first"
            res.success = False
            res.message = message

        elif req.data is False and not self.stock_in_scene:
            message = "attempting to remove stock that hasn't been unloaded"
            res.success = False
            res.message = message

        return res

    def callback_attach_stock(self, req, res):
        """
        Service callback to attach the stock to the frame given in the attachment frame

        :param req:
        :param res:
        :return:
        """
        if req.data is True and self.stock_in_scene and not self.stock_attached:
            #  Attach stock
            msg = AttachedCollisionObject()

            stock_object = self.create_collision_object_with_stock()
            stock_object.header.frame_id = self.get_parameter("attach.frame").get_parameter_value().string_value
            stock_object.mesh_poses.append(self.load_pose_from_param("attach.pose"))
            stock_object.operation = CollisionObject.ADD

            msg.object = stock_object
            msg.link_name = self.get_parameter("attach.frame").get_parameter_value().string_value
            self.get_logger().info("attaching stock to link")
            self.pub_moveit_attached_collision.publish(msg)
            self.stock_attached = True

            res.message = "stock attached"
            res.success = True

        elif req.data is False and self.stock_in_scene:
            # Remove from scene
            msg = AttachedCollisionObject()
            msg.object.operation = CollisionObject.REMOVE
            msg.object.id = self.get_parameter("stock_frame").get_parameter_value().string_value
            msg.link_name = self.get_parameter("attach.frame").get_parameter_value().string_value
            self.get_logger().info("detaching stock from link")
            self.pub_moveit_attached_collision.publish(msg)
            self.stock_attached = False

            res.message = "stock detached"
            res.success = True

        elif req.data is True and self.stock_attached:
            message = "stock is already attached"
            res.success = False
            res.message = message

        elif req.data is False and not self.stock_attached:
            message = "no stock to detach"
            res.success = False
            res.message = message

        return res

    def create_collision_object_with_stock(self):
        """
        Simple helper function to make the collision object with the mesh initialised within
        :return: collision object with mesh
        :rtype: CollisionObject
        """
        collision_object = CollisionObject()
        collision_object.id = self.get_parameter("stock_frame").get_parameter_value().string_value

        stock_mesh_path = self.get_parameter("stock_mesh_path").get_parameter_value().string_value
        self.get_logger().info("Path: {}".format(stock_mesh_path))
        stock = make_mesh_msg(stock_mesh_path)
        collision_object.meshes.append(stock)

        stock_pose = Pose()
        stock_pose.position.x = 0.00
        stock_pose.position.y = 0.00
        # stock_pose.position.z = 0.07985
        stock_pose.position.z = 0.0
        stock_pose.orientation.w = 1.0

        # collision_object.mesh_poses.append(stock_pose)

        return collision_object


def main(args=None):
    rclpy.init(args=args)

    stock_handler = StockHandler()
    # collision_msg = stock_handler.create_moveit_collision_object()
    # # stock_handler.pub_moveit_collision.publish(collision_msg)
    #
    # stock_handler.get_logger().info("Currently set to just attach the stock directly at the implant")
    # attach_msg = AttachedCollisionObject()
    # attach_msg.link_name = stock_handler.get_parameter("attachment_frame").get_parameter_value().string_value
    # attach_msg.object = collision_msg
    # stock_handler.pub_moveit_attached_collision.publish(attach_msg)

    try:
        rclpy.spin(stock_handler)
    except KeyboardInterrupt:
        pass
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    stock_handler.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
