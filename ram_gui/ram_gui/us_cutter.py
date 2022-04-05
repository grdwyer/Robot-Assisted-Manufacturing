import os
import subprocess
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from time import sleep

from std_srvs.srv import SetBool
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtCore import QAbstractListModel, QFile, QIODevice, Qt, Signal
from python_qt_binding.QtGui import QIcon, QImage, QPainter
from python_qt_binding.QtWidgets import QCompleter, QFileDialog, QGraphicsScene, QWidget, QLabel
from python_qt_binding.QtSvg import QSvgRenderer


class USCutter(Plugin):
    def __init__(self, context):
        super(USCutter, self).__init__(context)
        self._widget = QWidget()
        self._node = context.node

        ui_file = os.path.join(get_package_share_directory('ram_gui'), 'resource', 'us_cutter.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('US Cutter Interface')
        self.setObjectName('US Cutter Interface')
        self._widget.setWindowTitle('US Cutter Interface')
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + ('(%d)' % context.serial_number()))
            # Add widget to the user interface
        context.add_widget(self._widget)

        self._widget.push_activate.pressed.connect(self.cb_activate_pressed)
        self._widget.push_activate.released.connect(self.cb_activate_released)
        self._widget.check_enable.toggled.connect(self.cb_enable)

        self.client_enable = self._node.create_client(SetBool, "/us_cutter_controller/enable")
        self.client_activate = self._node.create_client(SetBool, "/us_cutter_controller/activate")

    def cb_activate_pressed(self):
        request = SetBool.Request()
        request.data = True
        self.client_activate.call_async(request)

    def cb_activate_released(self):
        request = SetBool.Request()
        request.data = False
        self.client_activate.call_async(request)

    def cb_enable(self):
        request = SetBool.Request()
        request.data = self._widget.check_enable.isChecked()
        self.client_enable.call_async(request)

    def __del__(self):
        self._node.get_logger().warn("Destroying toolpath loader plugin")
        self.client_enable.destroy()
        self.client_activate.destroy()
