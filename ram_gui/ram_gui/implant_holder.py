import os
import subprocess
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from time import sleep

from std_srvs.srv import Trigger
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtCore import QAbstractListModel, QFile, QIODevice, Qt, Signal
from python_qt_binding.QtGui import QIcon, QImage, QPainter
from python_qt_binding.QtWidgets import QCompleter, QFileDialog, QGraphicsScene, QWidget, QLabel
from python_qt_binding.QtSvg import QSvgRenderer


class ImplantHolder(Plugin):
    def __init__(self, context):
        super(ImplantHolder, self).__init__(context)
        self._widget = QWidget()
        self._node = context.node

        ui_file = os.path.join(get_package_share_directory('ram_gui'), 'resource', 'implant_holder.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('Implant Holder Interface')
        self.setObjectName('Implant Holder Interface')
        self._widget.setWindowTitle('Implant Holder Interface')
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + ('(%d)' % context.serial_number()))
            # Add widget to the user interface
        context.add_widget(self._widget)

        self._widget.push_open.clicked.connect(self.cb_open)
        self._widget.push_close.clicked.connect(self.cb_close)

        self.client_open = self._node.create_client(Trigger, "/gripper_controller/open")
        self.client_close = self._node.create_client(Trigger, "/gripper_controller/close")

    def cb_open(self):
        # Trigger setup to the toolpath handler
        trigger = Trigger.Request()
        self.client_open.call_async(trigger)

    def cb_close(self):
        # Trigger setup to the toolpath handler
        trigger = Trigger.Request()
        self.client_close.call_async(trigger)

    def __del__(self):
        self._node.get_logger().warn("Destroying toolpath loader plugin")
        self.client_open.destroy()
        self.client_close.destroy()
