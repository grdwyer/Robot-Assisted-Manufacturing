import os
import subprocess
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from time import sleep
import threading

from std_srvs.srv import Trigger
from ram_interfaces.srv import GetToolpath, RequestTrigger
from ram_interfaces.msg import Toolpath
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtCore import QAbstractListModel, QFile, QIODevice, Qt, Signal
from python_qt_binding.QtGui import QIcon, QImage, QPainter
from python_qt_binding.QtWidgets import QCompleter, QFileDialog, QGraphicsScene, QWidget, QLabel
from python_qt_binding.QtSvg import QSvgRenderer


class ProcedureController(Plugin):
    def __init__(self, context):
        super(ProcedureController, self).__init__(context)
        self._widget = QWidget()
        self._node = Node('procedure_controller', context=context.node.context)
        # self._node = context.node

        ui_file = os.path.join(get_package_share_directory('ram_gui'), 'resource', 'procedure_controller.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('Procedure Controller')
        self.setObjectName('Procedure Controller')
        self._widget.setWindowTitle('Procedure Controller')
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + ('(%d)' % context.serial_number()))
            # Add widget to the user interface
        context.add_widget(self._widget)

        self._widget.button_setup.clicked.connect(self.cb_setup_toolpath)
        self._widget.button_execute.clicked.connect(self.cb_execute_toolpath)

        self.server_request_setup = self._node.create_service(RequestTrigger, '/behaviour/request_setup',
                                                              self.callback_request_setup)
        self.server_request_execute = self._node.create_service(RequestTrigger, '/behaviour/request_execute',
                                                                self.callback_request_execute)

        self.executor = rclpy.executors.MultiThreadedExecutor(4)
        self.executor.add_node(self._node)

        self.thread_executor = threading.Thread(target=self.executor.spin)
        self.thread_executor.start()

    def set_status(self, message):
        self._widget.label_status.setText(message)
        self._node.get_logger().info(message)

    def cb_execute_toolpath(self):
        # Trigger execute to the toolpath handler
        self.set_status("Planner will now execute toolpath")

    def cb_setup_toolpath(self):
        # Trigger setup to the toolpath handler
        self.set_status("Planner will now setup toolpath")

    def callback_request_setup(self, request: RequestTrigger.Request, response: RequestTrigger.Response):
        if self._widget.button_setup.isChecked():
            response.trigger = True
        else:
            response.trigger = False
        self._widget.button_setup.setChecked(False)
        return response

    def callback_request_execute(self, request: RequestTrigger.Request, response: RequestTrigger.Response):
        if self._widget.button_execute.isChecked():
            response.trigger = True
        else:
            response.trigger = False
        self._widget.button_execute.setChecked(False)
        return response

    def __del__(self):
        self._node.get_logger().warn("Destroying procedure plugin")
        self.executor.shutdown(0.2)
        self.thread_executor.join(0.2)
        self.server_request_setup.destroy()
        self.server_request_execute.destroy()
