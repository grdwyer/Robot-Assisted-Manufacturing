import os
import subprocess
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from time import sleep

from ram_interfaces.srv import SetToolpathParameters, GetToolpathParameters
from rqt_reconfigure.param_api import AsyncServiceCallFailed, ParamClient
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtCore import QAbstractListModel, QFile, QIODevice, Qt, Signal
from python_qt_binding.QtGui import QIcon, QImage, QPainter
from python_qt_binding.QtWidgets import QCompleter, QFileDialog, QGraphicsScene, QWidget, QLabel


class ToolpathParams(Plugin):
    def __init__(self, context):
        super(ToolpathParams, self).__init__(context)
        self._widget = QWidget()
        self._node = context.node

        ui_file = os.path.join(get_package_share_directory('ram_gui'), 'resource', 'toolpath_params.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('Toolpath Parameters')
        self.setObjectName('Toolpath Parameters')
        self._widget.setWindowTitle('Toolpath Parameters')
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + ('(%d)' % context.serial_number()))
            # Add widget to the user interface
        context.add_widget(self._widget)

        self._param_list = ["desired_cartesian_velocity", "desired_cartesian_acceleration", "toolpath_height_offset",
                            "approach_offset", "retreat_offset", "retreat_height", "debug_mode", "debug_wait_time"]
        self._remote_node_name = "toolpath_planner"

        self.param_client = ParamClient(self._node, self._remote_node_name)
        self.set_parameters = self._node.create_client(SetToolpathParameters, "/toolpath_planner/set_parameters")
        self.get_parameters = self._node.create_client(SetToolpathParameters, "/toolpath_planner/get_parameters")
        self.load_initial_values()
        self.setup_callbacks()

    def load_initial_values(self):
        """
        Function to load the current parameters within the toolpath planner node.
        :return:
        """
        future = self.get_parameters.call_async(GetToolpathParameters.Request())
        rclpy.spin_until_future_complete(self._node, future)
        response = future.result()
        response = GetToolpathParameters.Response()
        self._widget.cartesianVelocityDoubleSpinBox.setValue(response.parameters.cartesian_velocity
                                                             * 1000.0)
        self._widget.cartesianAccelerationDoubleSpinBox.setValue(response.parameters.cartesian_acceleration
                                                                 * 1000.0)
        self._widget.toolpathHeightOffsetDoubleSpinBox.setValue(response.parameters.toolpath_height_offset
                                                                * 1000.0)
        self._widget.approachOffsetDoubleSpinBox.setValue(response.parameters.approach_offset * 1000.0)
        self._widget.retreatOffsetDoubleSpinBox.setValue(response.parameters.retreat_offset * 1000.0)
        self._widget.retreatHeightDoubleSpinBox.setValue(response.parameters.retreat_height * 1000.0)
        # self._widget.debugModeCheckBox.setChecked(initial_values[6].get_parameter_value().bool_value)
        # self._widget.debugWaitTimeSpinBox.setValue(initial_values[7].get_parameter_value().integer_value)

    def setup_callbacks(self):
        self._widget.cartesianVelocityDoubleSpinBox.valueChanged.connect(self.cb_cart_vel)
        self._widget.cartesianAccelerationDoubleSpinBox.valueChanged.connect(self.cb_cart_acc)
        self._widget.toolpathHeightOffsetDoubleSpinBox.valueChanged.connect(self.cb_tool_offset)
        self._widget.approachOffsetDoubleSpinBox.valueChanged.connect(self.cb_approach_offset)
        self._widget.retreatOffsetDoubleSpinBox.valueChanged.connect(self.cb_retreat_offset)
        self._widget.retreatHeightDoubleSpinBox.valueChanged.connect(self.cb_retreat_height)
        # self._widget.debugModeCheckBox.stateChanged.connect(self.cb_debug_mode)
        # self._widget.debugWaitTimeSpinBox.valueChanged.connect(self.cb_debug_wait)

    def cb_cart_vel(self):
        try:
            val = self._widget.cartesianVelocityDoubleSpinBox.value() * 0.001
            param = Parameter(name=self._param_list[0], value=val)
            self.param_client.set_parameters([param])
        except Exception as e:
            self._node.get_logger().warn(e)

    def cb_cart_acc(self):
        try:
            param = Parameter(name=self._param_list[1], value=self._widget.cartesianAccelerationDoubleSpinBox.value()
                                                              * 0.001)
            self.param_client.set_parameters([param])
        except Exception as e:
            self._node.get_logger().warn(e)

    def cb_tool_offset(self):
        try:
            param = Parameter(name=self._param_list[2], value=self._widget.toolpathHeightOffsetDoubleSpinBox.value()
                                                              * 0.001)
            self.param_client.set_parameters([param])
        except Exception as e:
            self._node.get_logger().warn(e)

    def cb_approach_offset(self):
        try:
            param = Parameter(name=self._param_list[3], value=self._widget.approachOffsetDoubleSpinBox.value() * 0.001)
            self.param_client.set_parameters([param])
        except Exception as e:
            self._node.get_logger().warn(e)

    def cb_retreat_offset(self):
        try:
            param = Parameter(name=self._param_list[4], value=self._widget.retreatOffsetDoubleSpinBox.value() * 0.001)
            self.param_client.set_parameters([param])
        except Exception as e:
            self._node.get_logger().warn(e)

    def cb_retreat_height(self):
        try:
            param = Parameter(name=self._param_list[5], value=self._widget.retreatHeightDoubleSpinBox.value() * 0.001)
            self.param_client.set_parameters([param])
        except Exception as e:
            self._node.get_logger().warn(e)

    def cb_debug_mode(self):
        try:
            param = Parameter(name=self._param_list[6], value=self._widget.debugModeCheckBox.isChecked())
            self.param_client.set_parameters([param])
        except Exception as e:
            self._node.get_logger().warn(e)

    def cb_debug_wait(self):
        try:
            param = Parameter(name=self._param_list[7], value=self._widget.debugWaitTimeSpinBox.value())
            self.param_client.set_parameters([param])
        except Exception as e:
            self._node.get_logger().warn(e)

    def __del__(self):
        self._node.get_logger().warn("Destroying toolpath parameter plugin")
        self.param_client.close()
