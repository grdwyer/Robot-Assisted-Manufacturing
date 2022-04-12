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
        assert isinstance(self._node, Node)

        ui_file = os.path.join(get_package_share_directory('ram_gui'), 'resource', 'toolpath_params.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('Toolpath Parameters')
        self.setObjectName('Toolpath Parameters')
        self._widget.setWindowTitle('Toolpath Parameters')
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + ('(%d)' % context.serial_number()))
            # Add widget to the user interface
        context.add_widget(self._widget)

        self.set_parameters = self._node.create_client(SetToolpathParameters,
                                                       "/toolpath_planner/set_toolpath_parameters")
        self.get_parameters = self._node.create_client(GetToolpathParameters,
                                                       "/toolpath_planner/get_toolpath_parameters")

        self.cb_get_params()
        self.setup_callbacks()

    def setup_callbacks(self):
        self._widget.pushGetParams.clicked.connect(self.cb_get_params)
        self._widget.pushSetParams.clicked.connect(self.cb_set_params)

    def cb_set_params(self):
        request = SetToolpathParameters.Request()
        request.parameters.toolpath_height_offset = self._widget.toolpathHeightOffsetDoubleSpinBox.value() * 0.001
        request.parameters.retreat_height = self._widget.retreatHeightDoubleSpinBox.value() * 0.001
        request.parameters.approach_offset = self._widget.approachOffsetDoubleSpinBox.value() * 0.001
        request.parameters.retreat_offset = self._widget.retreatOffsetDoubleSpinBox.value() * 0.001
        request.parameters.cartesian_velocity = self._widget.cartesianVelocityDoubleSpinBox.value() * 0.001
        request.parameters.cartesian_acceleration = self._widget.cartesianAccelerationDoubleSpinBox.value() * 0.001

        if self.set_parameters.wait_for_service(1.0):
            future = self.set_parameters.call_async(request)
            rclpy.spin_until_future_complete(self._node, future, timeout_sec=2.5)
            if future.done():
                response = future.result()
                if response.success:
                    self._node.get_logger().info("Set toolpath parameters")
                else:
                    self._node.get_logger().warn("failed to set the parameters")
            else:
                self._node.get_logger().warn("failed to call service")
        else:
            self._node.get_logger().warn("Service is not available")

    def cb_get_params(self):
        if self.set_parameters.wait_for_service(1.0):
            request = GetToolpathParameters.Request()
            future = self.get_parameters.call_async(request)
            rclpy.spin_until_future_complete(self._node, future, timeout_sec=2.5)
            if future.done():
                response = future.result()
                if response.success:
                    self._widget.cartesianVelocityDoubleSpinBox.setValue(response.parameters.cartesian_velocity
                                                                         * 1000.0)
                    self._widget.cartesianAccelerationDoubleSpinBox.setValue(response.parameters.cartesian_acceleration
                                                                             * 1000.0)
                    self._widget.toolpathHeightOffsetDoubleSpinBox.setValue(response.parameters.toolpath_height_offset
                                                                            * 1000.0)
                    self._widget.approachOffsetDoubleSpinBox.setValue(response.parameters.approach_offset * 1000.0)
                    self._widget.retreatOffsetDoubleSpinBox.setValue(response.parameters.retreat_offset * 1000.0)
                    self._widget.retreatHeightDoubleSpinBox.setValue(response.parameters.retreat_height * 1000.0)
                else:
                    self._node.get_logger().warn("failed to get the parameters")
            else:
                self._node.get_logger().warn("failed to call service")
        else:
            self._node.get_logger().warn("Service is not available")

    def __del__(self):
        self._node.get_logger().warn("Destroying toolpath parameter plugin")
        self.set_parameters.destroy()
        self.get_parameters.destroy()
