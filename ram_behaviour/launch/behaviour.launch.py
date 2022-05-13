from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    declared_arguments = [DeclareLaunchArgument("groot", default_value="false",
                                                description="launch groot"),
                          DeclareLaunchArgument("tree_file", default_value="switching_gripper.xml",
                                                description="xml file defining the tree found in ram_behaviour package "
                                                            "in trees directory with file suffix")]

    loop_timeout = LaunchConfiguration('loop_timeout', default=10)
    arg_groot = LaunchConfiguration('groot')
    arg_tree_file = LaunchConfiguration('tree_file')

    nodes = []
    nodes.append(Node(
        package='behavior_tree',
        namespace='',
        executable='bt_engine',
        # Do not declare a node name otherwise it messes with the action node names and will result in
        # duplicate nodes!
        arguments=['--log-level debug'],
        output='screen',
        parameters=[
            {'loop_timeout': loop_timeout},
            {'bt_file_path': PathJoinSubstitution([get_package_share_directory('ram_behaviour'),
                                          'trees/', arg_tree_file])},
            {'plugins': ['trigger_component_bt_node', 'set_bool_component_bt_node', 'request_trigger_component_bt_node',
                         'modify_stock_touch_links_component_bt_node', 'modify_acm_component_bt_node',
                         'get_toolpath_component_bt_node', 'set_toolpath_component_bt_node']}
        ]
    ))

    nodes.append(ExecuteProcess(cmd=["ros2 run groot Groot --mode monitor"],
                                shell=True,
                                condition=IfCondition(arg_groot)))

    return LaunchDescription(declared_arguments + nodes)