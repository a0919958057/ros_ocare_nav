#!/usr/bin/env python
# -*- coding: utf-8 -*-


import os
import time

import rospy
import rospkg

from math import *

from std_msgs.msg import String,Int32,UInt16MultiArray,MultiArrayLayout,MultiArrayDimension
from python_qt_binding import loadUi
from python_qt_binding.QtCore import *
from python_qt_binding.QtGui import *
from sensor_msgs.msg import Imu, LaserScan



class OcareWidget(QWidget):

    def __init__(self, context):
        super(OcareWidget, self).__init__()
        self.rp = rospkg.RosPack()

        ui_file = os.path.join(self.rp.get_path('rqt_ocare'), 'resource', 'ocare_ui.ui')
        
        loadUi(ui_file, self)

        self.setObjectName('OcareWidget')

        self.progressBarList = [self.progressBar_1,
                           self.progressBar_2,
                           self.progressBar_3,
                           self.progressBar_4,
                           self.progressBar_5,
                           self.progressBar_6,
                           self.progressBar_7,
                           self.progressBar_8,
                           self.progressBar_9,
                           self.progressBar_10,
                           self.progressBar_11,
                           self.progressBar_12,
                           self.progressBar_13]

        self._setup_publisher()
        self._setup_subscriber()
        self._setup_callback()
        self._setup_graph()

        self.connect(self, SIGNAL('updateSensor'), self._update_sensor)
        self.connect(self, SIGNAL('updateIMU'), self._update_imu)
        self.connect(self, SIGNAL('updateSensorStatus'), self._update_status)
        self.connect(self, SIGNAL('updateStage'), self._update_stage)

        self._orient = 0
        self._num = 0
        self._sensor_value = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self._is_imu_ready = False
        self._is_lrf_ready = False
        self._is_line_sensor_ready = False
        self._current_stage = 0


    def _setup_publisher(self):
        self._pub = rospy.Publisher('chatter', String, queue_size=10)
        self._stage_pub = rospy.Publisher('stage_set_cmd', Int32, queue_size=10)
        self._control_pub = rospy.Publisher('diff_mode_controller_cmd', Int32, queue_size=10)

    def _setup_subscriber(self):
        self._imu_sub = rospy.Subscriber('/imu', Imu, self.callback_imu)
        self._sensor_sub = rospy.Subscriber('/track_line_sensor', UInt16MultiArray, self.callback_track_sensor)
        self._lrf_sub = rospy.Subscriber('/scan', LaserScan, self.callback_lrf)
        self._stage_sub = rospy.Subscriber('/stage_mode', Int32, self.callback_stage)


    def _setup_callback(self):
        self.pushButtonStart.clicked[bool].connect(self._handle_start_clicked)
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
        self.checkBoxArmOpen.toggled.connect(self._handle_arm_open)
        self.orientReset.clicked[bool].connect(self._handle_reset_orent_clicked)
        self.refreshTopicStatus.clicked[bool].connect(self._handle_refresh_clicked)

    def _setup_graph(self):
        path = os.path.join(self.rp.get_path('rqt_ocare'), 'resource', 'compass.png')
        self.pixmap = QPixmap(path)


        self.graphicsPixmapItem = QGraphicsPixmapItem(self.pixmap)


        self.graphicsScene = QGraphicsScene()
        self.graphicsScene.addItem(self.graphicsPixmapItem)

        self.graphicsViewOrient.setScene(self.graphicsScene)
        self.graphicsViewOrient.scale(\
            self.graphicsViewOrient.width()/self.graphicsScene.width(), \
            self.graphicsViewOrient.height() / self.graphicsScene.height()
        )

        # self.graphicsScene.setSceneRect(0, 0, 100, 100);

    def callback_imu(self, msg):
        import tf
        """
        :type msg: Imu
        """

        (r, p, y) = tf.transformations.euler_from_quaternion( \
            [msg.orientation.x, msg.orientation.y, msg.orientation.z, \
             msg.orientation.w])

        self._orient = y
        self._is_imu_ready = True
        self.emit(SIGNAL("updateIMU"))
        self.emit(SIGNAL("updateSensorStatus"))
        # self.update()

    def callback_track_sensor(self, msg):

        """

        :type msg:UInt16MultiArray
        """

        for i in range(0,13):
            self._sensor_value[i] = (100 - msg.data[i])

        self._is_line_sensor_ready = True

        self.emit(SIGNAL("updateSensor"))
        self.emit(SIGNAL("updateSensorStatus"))
        # self.update()


    def callback_lrf(self, msg):
        self._is_lrf_ready = True
        self.emit(SIGNAL("updateSensorStatus"))

    def callback_stage(self, msg):
        """

        :type msg:Int32
        """
        self._current_stage = msg.data
        self.emit(SIGNAL("updateStage"))

    #def callback_stage(self, msg):

    def _update_sensor(self):
        for i in range(0,13):
            self.progressBarList[i].setValue(self._sensor_value[i])

        weight_sum = 0.0
        for (weight, i) in zip(range(-6,7),range(0,13)):
            weight_sum = weight_sum + float(weight) * float(self._sensor_value[i])
        self.sliderBK.setValue(weight_sum*100 / (sum(self._sensor_value)+1))

        sensor_wh_value = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        for i in range(0, 13):
            sensor_wh_value[i] = 100.0 - self._sensor_value[i]
        weight_sum = 0.0
        for (weight, i) in zip(range(-6,7),range(0,13)):
            weight_sum = weight_sum + float(weight) * float(sensor_wh_value[i])
        self.sliderWH.setValue(weight_sum*100 / (sum(sensor_wh_value)+1))

    def _update_imu(self):
        self._set_icon_radian(self._orient)
        self.lcdNumberOrient.display(self._orient*180/3.1415926)

    def _update_status(self):
        self.statusLRF.setChecked(self._is_lrf_ready)
        self.statusIMU.setChecked(self._is_imu_ready)
        self.statusLS.setChecked(self._is_line_sensor_ready)

    def _update_stage(self):
        self.lcdNumberDiffMode_2.display(self._current_stage)

    # def paintEvent(self, event):

    def _handle_start_clicked(self):
        # 1 is start
        # -1 is stop
        if self.pushButtonStart.isChecked():
            if self.btnModeAuto.isChecked():
                self._control_pub.publish(1)
            elif self.btnModeRemote.isChecked():
                self._control_pub.publish(2)
        else:
            self._control_pub.publish(-1)


    def _handle_reset_orent_clicked(self):
        import std_srvs.srv
        set_zero = rospy.ServiceProxy('/set_zero_orientation', std_srvs.srv.Empty)
        set_zero.call()

    def _handle_num(self, button):
        old_num = self.lcdNumberDiffMode.intValue()
        self.lcdNumberDiffMode.display(old_num * 10 + button)

    def _handle_num_clear(self):
        self.lcdNumberDiffMode.display(0)

    def _handle_num_enter(self):
        current_num = self.lcdNumberDiffMode.intValue()
        self._stage_pub.publish(current_num)
        self.lcdNumberDiffMode.display(0)

    def _handle_arm_open(self):
        if self.checkBoxArmOpen.isChecked():
            self.lcdNumberDiffMode_2.display('1')
        else:
            self.lcdNumberDiffMode_2.display('2')

    def _handle_refresh_clicked(self):
        self._is_imu_ready = False
        self._is_lrf_ready = False
        self._is_line_sensor_ready = False
        self.emit(SIGNAL("updateSensorStatus"))

    def _set_icon_radian(self, radian):
        q = QTransform()
        q.translate(self.pixmap.width() / 2, self.pixmap.height() / 2)
        q.rotate(radian * 180 / 3.1415926)
        q.translate(-self.pixmap.width() / 2, -self.pixmap.height() / 2)
        self.graphicsPixmapItem.setTransform(q)



