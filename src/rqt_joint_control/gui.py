import rosparam
import rospkg
import rospy
import std_msgs.msg
import sensor_msgs.msg

import os.path
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi as _loadUi
from python_qt_binding.QtCore import Qt, QTimer
from python_qt_binding.QtWidgets import QWidget, QLabel, QLineEdit, QSlider, \
    QHBoxLayout, QGridLayout
from itertools import izip_longest as zip_longest

from controller_manager_msgs.utils import ControllerManagerLister, ControllerLister
from lxml import etree
import traceback


def is_permutation(items1, items2):
    return len(items1) == len(items2) and sorted(items1) == sorted(items2)


def find(predicate, items, default=None):
    for item in items:
        if predicate(item):
            return item
    return default


def loadUi(path, widget=None):
    if widget is None:
        widget = QWidget()

    ui_file = os.path.join(
        rospkg.RosPack().get_path('rqt_joint_control'), 'resource', path
    )
    _loadUi(ui_file, widget)
    return widget


def update_combo(combo, new_items):
    selected = combo.currentText()
    old_items = [combo.itemText(i) for i in range(combo.count())]

    if is_permutation(old_items, new_items):
        return

    try:
        selected_id = new_items.index(selected)
    except ValueError:
        selected_id = -1

    combo.blockSignals(True)
    combo.clear()
    combo.insertItems(0, new_items)
    combo.setCurrentIndex(selected_id)
    combo.blockSignals(False)


def get_robot_description_ns_list():
    result = []
    for param in rosparam.list_params('/'):
        value = rosparam.get_param(param)
        if not isinstance(value, str):
            continue

        if '<?xml' not in value:
            continue

        try:
            assert etree.fromstring(value).xpath('/robot') != None
        except:
            traceback.print_exc()
            continue

        result.append(param)

    return result


class JointPositionControllerWidget(QWidget):
    def __init__(self, node_ns, controller, urdf=None, *args, **kwargs):
        super(JointPositionControllerWidget, self).__init__(*args, **kwargs)
        self._controller = controller
        self._urdf = urdf

        loadUi('Controller.ui', self)
        self.controller_name.setText(controller.name)
        self.controller_type.setText(controller.type)

        controller_ns = rospy.resolve_name(controller.name, node_ns)

        joint_name_widget = QLabel()

        joint_position_widget = QLineEdit()
        joint_position_widget.setReadOnly(True)
        joint_position_widget.setAlignment(Qt.AlignRight)
        joint_position_widget.setText(str(0.0))

        extras_layout = QGridLayout()
        extras_layout.addWidget(joint_name_widget, 0, 0)
        extras_layout.addWidget(joint_position_widget, 0, 1)

        extras = QWidget()
        extras.setLayout(extras_layout)
        self.layout().addWidget(extras)

        slider_range = (0, 10000)
        limits = {'min': 0.0, 'max': 3.14}

        if self._urdf is not None:
            joint_name = next(iter([
                resource
                for interface in controller.claimed_resources
                if interface.hardware_interface == \
                    'hardware_interface::PositionJointInterface'
                for resource in interface.resources
            ]), None)

            if joint_name is not None:
                joint_name_widget.setText(joint_name)

                joint_limit = self._urdf.xpath('/robot/joint[@name=$name]/limit', name=joint_name)
                if joint_limit:
                    l = joint_limit[0]
                    limits = {'min': float(joint_limit[0].get('lower')),
                                'max': float(joint_limit[0].get('upper'))}

        pub = rospy.Publisher(controller_ns + '/command', std_msgs.msg.Float64, queue_size=10)

        def on_value_changed(value):
            value = limits['min'] + value / float(slider_range[1] - slider_range[0]) * (limits['max'] - limits['min'])
            joint_position_widget.setText(str(value))
            pub.publish(value)

        slider = QSlider(Qt.Horizontal)
        slider.setRange(slider_range[0], slider_range[1])
        slider.valueChanged.connect(on_value_changed)

        extras_layout.addWidget(slider, 1, 0, 1, 2)


class JointControlPlugin(Plugin):
    CM_UPDATE_FREQ = 1

    CONTROLLERS = {
        'position_controllers/JointPositionController': JointPositionControllerWidget,
    }

    def __init__(self, context):
        super(JointControlPlugin, self).__init__(context)
        self.setObjectName('JointControlPlugin')

        self._widget = loadUi('JointControl.ui')
        self._widget.setObjectName('JointControlPluginUi')

        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                self._widget.windowTitle() + (' (%d)' % context.serial_number())
            )

        context.add_widget(self._widget)

        self._widget.robot_descriptions_list.currentIndexChanged[str].connect(self._on_robot_descriptions_list_change)
        self._widget.cm_list.currentIndexChanged[str].connect(self._on_cm_change)

        self._urdf = None
        self._cm_ns = None
        self._controllers = []

        self._update_robot_descriptions()

        self._list_cm = ControllerManagerLister()
        self._update_cm_list_timer = QTimer(self)
        self._update_cm_list_timer.setInterval(1000.0 / self.CM_UPDATE_FREQ)
        self._update_cm_list_timer.timeout.connect(self._update_cm_list)
        self._update_cm_list_timer.start()

        self._list_controllers = lambda: []

    def shutdown_plugin(self):
        pass

    def save_settings(self, plugin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        pass

    def _update_robot_descriptions(self):
        update_combo(self._widget.robot_descriptions_list, get_robot_description_ns_list())

    def _update_cm_list(self):
        update_combo(self._widget.cm_list, self._list_cm())

    def _on_robot_descriptions_list_change(self, robot_description_ns):
        desc = rosparam.get_param(robot_description_ns)
        try:
            xml = etree.fromstring(desc)
            self._urdf = xml.xpath('/robot')[0]
        except:
            traceback.print_exc()
            self._urdf = None
        self._update_controllers_list()

    def _on_cm_change(self, cm_ns):
        self._cm_ns = cm_ns
        self._list_controllers = ControllerLister(cm_ns) if cm_ns else (lambda: [])
        self._update_controllers_list()

    def _update_controllers_list(self):
        controllers = self._list_controllers()

        if controllers == self._controllers:
            return

        for _ in xrange(self._widget.controllers.count()):
            self._widget.controllers.takeAt(0)

        self._controllers = controllers

        joint_state_controller = find(
            lambda c: c.type == 'joint_state_controller/JointStateController',
            controllers
        )

        for controller in controllers:
            self._add_controller(controller)

    def _add_controller(self, controller):
        if controller.type not in self.CONTROLLERS:
            return

        widget = self.CONTROLLERS[controller.type](self._cm_ns, controller, self._urdf)
        self._widget.controllers.addWidget(widget)
