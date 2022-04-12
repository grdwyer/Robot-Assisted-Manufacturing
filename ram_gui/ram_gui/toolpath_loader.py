import os
import subprocess
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from time import sleep

from std_srvs.srv import Trigger
from ram_interfaces.srv import GetToolpath
from ram_interfaces.msg import Toolpath
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtCore import QAbstractListModel, QFile, QIODevice, Qt, Signal
from python_qt_binding.QtGui import QIcon, QImage, QPainter
from python_qt_binding.QtWidgets import QCompleter, QFileDialog, QGraphicsScene, QWidget, QLabel
from python_qt_binding.QtSvg import QSvgRenderer


class ToolpathLoader(Plugin):
    def __init__(self, context):
        super(ToolpathLoader, self).__init__(context)
        self._widget = QWidget()
        self._node = context.node

        ui_file = os.path.join(get_package_share_directory('ram_gui'), 'resource', 'toolpath_loader.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('Toolpath Interface')
        self.setObjectName('Toolpath Interface')
        self._widget.setWindowTitle('Toolpath Interface')
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + ('(%d)' % context.serial_number()))
            # Add widget to the user interface
        context.add_widget(self._widget)

        self._widget.button_refresh_list.clicked.connect(self.cb_refresh_list)
        self._widget.button_set_directory.clicked.connect(self.cb_set_directory)
        self._widget.button_load.clicked.connect(self.cb_load_toolpath)
        self._widget.button_setup.clicked.connect(self.cb_setup_toolpath)
        self._widget.button_execute.clicked.connect(self.cb_execute_toolpath)

        self.client_load_toolpath = self._node.create_client(GetToolpath, "/toolpath_handler/get_toolpath")
        self.client_setup_toolpath = self._node.create_client(Trigger, "/toolpath_planner/toolpath_setup")
        self.client_execute_toolpath = self._node.create_client(Trigger, "/toolpath_planner/toolpath_execute")

        self.future_load_toolpath = rclpy.Future()
        self.future_setup_toolpath = rclpy.Future()
        self.future_execute_toolpath = rclpy.Future()

        self._node.declare_parameter("toolpath_directory", os.path.join(
            get_package_share_directory('ram_tooling_support'), 'config/'))

        self.files = []
        self.loaded_toolpath = Toolpath()
        self.cb_refresh_list()

        self.timer = self._node.create_timer(0.5, self.cb_timer_spin)

        # Display stock toolpath
        # svg_file = os.path.join(get_package_share_directory('ram_gui'), 'resource', 'medpor_large.svg')
        # self._widget.graphics_view = QSvgRenderer(svg_file)
        #
        # self.painter = QPainter()
        # self._widget.graphics_view.render(self.painter)

    def cb_timer_spin(self):
        rclpy.spin_once(self._node)

    def set_status(self, message):
        self._widget.label_status.setText(message)
        self._node.get_logger().info(message)

    def cb_refresh_list(self):
        # get param for source directory
        # get a list of each yaml file in there
        # add as option in the list combobox
        self.set_status("Refreshing list of toolpaths")
        path = self._node.get_parameter("toolpath_directory")
        self.files = [toolpath for toolpath in os.listdir(path.get_parameter_value().string_value) if
                      'toolpath' in toolpath]
        self._widget.list_toolpaths.clear()
        self._widget.list_toolpaths.addItems(self.files)

    def cb_set_directory(self):
        # Open popup directory to select the folder
        self.set_status("Setting the directory for toolpaths but not implemented")

    def cb_load_toolpath(self):
        # get path to selected toolpath
        # set the param for the toolpath handler
        # trigger load to the toolpath handler
        self.set_status("Loading tool path")
        selected_toolpath = self.files[self._widget.list_toolpaths.currentIndex()]
        full_path = self._node.get_parameter("toolpath_directory").get_parameter_value().string_value + \
                    selected_toolpath
        self.set_status("Setting toolpath file to {}".format(full_path))
        request = GetToolpath.Request()
        request.id = full_path
        self.future_load_toolpath = self.client_load_toolpath.call_async(request)
        self.future_load_toolpath.add_done_callback(self.fcb_load_toolpath)

    def cb_setup_toolpath(self):
        # Trigger setup to the toolpath handler
        self.set_status("Planner will now setup toolpath")
        trigger = Trigger.Request()
        self.future_setup_toolpath = self.client_setup_toolpath.call_async(trigger)
        self.future_setup_toolpath.add_done_callback(self.fcb_setup_toolpath)

    def cb_execute_toolpath(self):
        # Trigger execute to the toolpath handler
        self.set_status("Planner will now execute toolpath")
        trigger = Trigger.Request()
        self.future_execute_toolpath = self.client_execute_toolpath.call_async(trigger)
        self.future_execute_toolpath.add_done_callback(self.fcb_execute_toolpath)

    def fcb_load_toolpath(self, future):
        if len(self.future_load_toolpath.result().toolpath.path.points) > 0:
            self.loaded_toolpath = self.future_load_toolpath.result().toolpath
            self.set_status("Toolpath handler has loaded the toolpath with {} points"
                            .format(len(self.loaded_toolpath.path.points)))
        else:
            self.set_status("Toolpath handler failed to load the toolpath")

    def fcb_setup_toolpath(self, fut):
        if self.future_setup_toolpath.result().success:
            self.set_status("Toolpath planner has setup the toolpath")
        else:
            self.set_status("Toolpath planner failed to setup the toolpath")

    def fcb_execute_toolpath(self, fut):
        if self.future_execute_toolpath.result().success:
            self.set_status("Toolpath planner has executed the toolpath")
        else:
            self.set_status("Toolpath planner failed to execute the toolpath")

    def __del__(self):
        self._node.get_logger().warn("Destroying toolpath loader plugin")
        self.client_setup_toolpath.destroy()
        self.client_execute_toolpath.destroy()
        self.client_load_toolpath.destroy()
        self.client_set_path.destroy()
