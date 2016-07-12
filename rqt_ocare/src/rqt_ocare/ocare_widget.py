#!/usr/bin/env python
# -*- coding: utf-8 -*-


import os
import time

import rospy
import rospkg

from std_msgs.msg import String
from std_msgs.msg import Int32
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, qWarning, Signal
from python_qt_binding.QtGui import QFileDialog, QGraphicsView, QIcon, QWidget, QButtonGroup, QAbstractButton


class OcareWidget(QWidget):
    set_status_text = Signal(str)

    def __init__(self, context):
        super(OcareWidget, self).__init__()
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('rqt_ocare'), 'resource', 'ocare_ui.ui')
        loadUi(ui_file, self)
        self._pub = rospy.Publisher('chatter', String, queue_size=10)
        self._stage_pub = rospy.Publisher('stage_set_cmd', Int32, queue_size=10)

        self.setObjectName('OcareWidget')
        self.pushButtonStart.clicked[bool].connect(self._handle_plus_1)

        self.buttonGroup = QButtonGroup()

        self.buttonGroup.addButton(self.num_0, 0)
        self.buttonGroup.addButton(self.num_1, 1)
        self.buttonGroup.addButton(self.num_2, 2)
        self.buttonGroup.addButton(self.num_3, 3)
        self.buttonGroup.addButton(self.num_4, 4)
        self.buttonGroup.addButton(self.num_5, 5)
        self.buttonGroup.addButton(self.num_6, 6)
        self.buttonGroup.addButton(self.num_7, 7)
        self.buttonGroup.addButton(self.num_8, 8)
        self.buttonGroup.addButton(self.num_9, 9)
        self.buttonGroup.buttonClicked[int].connect(self._handle_num)

        self.num_clear.clicked[bool].connect(self._handle_num_clear)
        self.num_enter.clicked[bool].connect(self._handle_num_enter)

        self._num = 0

    def _handle_plus_1(self):

        if self.checkBoxArmOpen.isChecked():
            self._num = self._num + 1
        else:
            self._num = self._num - 1

        self.lcdNumberDiffMode.display(self._num)
        self._pub.publish("Test")

    def _handle_num(self, button):
        old_num = self.lcdNumberDiffMode.intValue()
        self.lcdNumberDiffMode.display(old_num * 10 + button)

    def _handle_num_clear(self):
        self.lcdNumberDiffMode.display(0)

    def _handle_num_enter(self):
        current_num = self.lcdNumberDiffMode.intValue()
        self._stage_pub.publish(current_num)
