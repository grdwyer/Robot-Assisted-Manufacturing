import rclpy
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped, Transform
import std_msgs.msg
from std_srvs.srv import SetBool
import PyKDL
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene
from shape_msgs.msg import Mesh, MeshTriangle, SolidPrimitive
from geometry_msgs.msg import Pose, Point
from ram_interfaces.srv import SetTouchLinks
import pyassimp
import os
from ament_index_python.packages import get_package_share_directory
import numpy as np


class ACMModifier(Node):
    def __init__(self):
        """
        Node to handle the information and placement associated with the stock material
        Currently this is loading the mesh of the stock material as an attached collision object

        """
        super().__init__('acm_modifier', allow_undeclared_parameters=True)
        self.client_planning_scene = self.create_client(GetPlanningScene, "/get_planning_scene")

    def get_acm(self):
        request = GetPlanningScene.Request()
        request.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        future = self.client_planning_scene.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        print(response.scene.allowed_collision_matrix)


def main(args=None):
    rclpy.init(args=args)

    node = ACMModifier()
    node.get_acm()
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

