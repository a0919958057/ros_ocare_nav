#!/usr/bin/env python
# -*- coding: utf-8 -*-


import os
import time

import rospy
import rospkg

from math import *

from std_msgs.msg import String,Int32,UInt16MultiArray,MultiArrayLayout,MultiArrayDimension
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from python_qt_binding import loadUi
from python_qt_binding.QtCore import *
from python_qt_binding.QtGui import *
from sensor_msgs.msg import Imu, LaserScan

import sys
import gi
gi.require_version('Gst', '1.0')
from gi.repository import GObject, Gst
GObject.threads_init()
from gi.repository import GLib


class OcareWidget(QWidget):

    # DiffMode
    MODE_AUTO_START = 1
    MODE_REMOTE_START = 2
    MODE_STOP = -1

    # Arm Mode
    MODE_ARM_SLIDER_HOME = -1
    MODE_ARM_SLIDER_OPEN = 1
    MODE_ARM_HOME_POSE = -2
    MODE_ARM_BTN_POSE = 2
    MODE_ARM_FREE_CONTROL = 3

    def __init__(self, context):
        super(OcareWidget, self).__init__()

        self._orient = 0
        self._num = 0
        self._sensor_value = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self._is_imu_ready = False
        self._is_lrf_ready = False
        self._is_line_sensor_ready = False
        self._current_stage = 0



        self.rp = rospkg.RosPack()

        import platform
        if(platform.machine() == 'x86_64'):
            # If there is PC
            ui_file = os.path.join(self.rp.get_path('rqt_ocare'), 'resource', 'ocare_ui_pc.ui')
        else:
            # If there is raspberry Pi
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

        self._setup_gstreamer()
        self._setup_publisher()
        self._setup_subscriber()
        self._setup_callback()
        self._setup_graph()

        self.connect(self, SIGNAL('updateSensor'), self._update_sensor)
        self.connect(self, SIGNAL('updateIMU'), self._update_imu)
        self.connect(self, SIGNAL('updateSensorStatus'), self._update_status)
        self.connect(self, SIGNAL('updateStage'), self._update_stage)

    def _setup_gstreamer(self):

        Gst.init()

        # Setup the Debug mode for GStreamer
        Gst.debug_set_active(False)
        Gst.debug_set_default_threshold(3)

        import platform
        if(platform.machine() == 'x86_64'):
            # If there is PC
            self._init_audio_pub_stream()
        else:
            # If there is raspberry Pi
            self._init_audio_rec_stream()

        # self._init_audio_pub_stream()
        # self._init_audio_rec_stream()
        # self._init_video_pub_stream()
        # self._init_video_rec_stream()

    # There is useless
    def _init_video_pub_stream(self):
        self.pipe_video_pub = Gst.Pipeline.new("Streamer")

        self.camera = Gst.ElementFactory.make("v4l2src", "camera")
        self.camera.set_property('device', '/dev/video0')

        self.videocaps = Gst.Caps.from_string("video/x-raw,width=(int)640,height=(int)480,framerate=(fraction)30/1")
        self.videofilter1 = Gst.ElementFactory.make("capsfilter", "filter1")
        self.videofilter1.set_property("caps", self.videocaps)

        self.videov = Gst.ElementFactory.make("videoconvert", "converter1")

        import platform

        if(platform.machine() == 'x86_64'):
            # If there is PC
            self.encoder = Gst.ElementFactory.make("x264enc", "encoder")
            print('Using the x264enc')
        else:
            # If there is raspberry Pi
            self.encoder = Gst.ElementFactory.make("omxh264enc", "encoder")
            self.encoder.set_property('target-bitrate', 15000000)
            self.encoder.set_property('control-rate', 'variable')
            print('Using the omxh264enc')

        self.videoecaps = Gst.Caps.from_string("video/x-h264, profile=high")
        self.videofilter2 = Gst.ElementFactory.make("capsfilter", "filter2")
        self.videofilter2.set_property("caps", self.videoecaps)

        self.videoparse = Gst.ElementFactory.make("h264parse", "parse")

        self.rtph = Gst.ElementFactory.make("rtph264pay", "rtph")
        self.rtph.set_property('config-interval', 10)
        self.rtph.set_property('pt', 96)

        self.udpsink = Gst.ElementFactory.make("udpsink", "udpsink")
        self.udpsink.set_property('host', 'localhost')
        self.udpsink.set_property('port', 25650)
        self.udpsink.set_property('auto-multicast', False)

        self.pipe_video_pub.add(self.camera)
        self.pipe_video_pub.add(self.videofilter1)
        self.pipe_video_pub.add(self.videov)
        self.pipe_video_pub.add(self.videofilter2)
        self.pipe_video_pub.add(self.videoparse)
        self.pipe_video_pub.add(self.rtph)
        self.pipe_video_pub.add(self.udpsink)

        # self.test_sink = Gst.ElementFactory.make('autovideosink', 'testsink')
        # self.pipe_video_pub.add(self.test_sink)
        # self.camera.link(self.test_sink)

        self.camera.link(self.videofilter1)
        self.videofilter1.link(self.videov)
        self.videov.link(self.encoder)
        self.encoder.link(self.videofilter2)
        self.videofilter2.link(self.videoparse)
        self.videoparse.link(self.rtph)
        self.rtph.link(self.udpsink)

        self.pipe_video_pub.set_state(Gst.State.PLAYING)

    def _init_audio_pub_stream(self):
        self.pipe_audio_pub = Gst.parse_launch(
            'alsasrc ! '
            'audioconvert ! '
            'audio/x-raw, format=U16LE, channels=1, rate=44100 ! '
            'audioconvert ! '
            'rtpL16pay ! '
            'udpsink host=ubuntu port=5000')

        self.pipe_audio_pub.set_state(Gst.State.PAUSED)

    def _init_video_rec_stream(self):
        self.pipe_video_rec = Gst.parse_launch(
            'udpsrc port=25650 ! '
            'application/x-rtp, payload=96 ! '
            'rtpjitterbuffer ! '
            'rtph264depay ! '
            'avdec_h264 ! '
            'autovideosink'
        )

        # Receive default Playing
        self.pipe_video_rec.set_state(Gst.State.PLAYING)

    def _init_audio_rec_stream(self):
        self.pipe_audio_rec = Gst.parse_launch(
            'udpsrc port=5000 ! '
            'application/x-rtp,media=audio, clock-rate=44100, width=16, height=16, encoding-name=L16, encoding-params=1, channels=1, channel-positions=1, payload=96 ! '
            'rtpL16depay ! '
            'audioconvert ! '
            'alsasink sync=false'
        )

        # Receive default Playing
        self.pipe_audio_rec.set_state(Gst.State.PLAYING)

    def _setup_publisher(self):
        self._pub = rospy.Publisher('chatter', String, queue_size=10)
        self._stage_pub = rospy.Publisher('stage_set_cmd', Int32, queue_size=10)
        self._control_pub = rospy.Publisher('diff_mode_controller_cmd', Int32, queue_size=10)
        self._arm_mode_pub = rospy.Publisher('arm_mode_controller_cmd', Int32, queue_size=10)
        self._arm_pos_pub = rospy.Publisher('arm_position_cmd', JointTrajectoryPoint, queue_size=10)
        self._arm_catch_pub = rospy.Publisher('arm_catch_level', Int32, queue_size=10)

    def _setup_subscriber(self):
        self._imu_sub = rospy.Subscriber('/imu', Imu, self.callback_imu)
        self._sensor_sub = rospy.Subscriber('/track_line_sensor', UInt16MultiArray, self.callback_track_sensor)
        self._lrf_sub = rospy.Subscriber('/scan', LaserScan, self.callback_lrf)
        self._stage_sub = rospy.Subscriber('/stage_mode', Int32, self.callback_stage)


    def _setup_callback(self):
        self.pushButtonStart.clicked[bool].connect(self._handle_start_clicked)
        self.numGroup = QButtonGroup()
        self.numGroup.addButton(self.num_0, 0)
        self.numGroup.addButton(self.num_1, 1)
        self.numGroup.addButton(self.num_2, 2)
        self.numGroup.addButton(self.num_3, 3)
        self.numGroup.addButton(self.num_4, 4)
        self.numGroup.addButton(self.num_5, 5)
        self.numGroup.addButton(self.num_6, 6)
        self.numGroup.addButton(self.num_7, 7)
        self.numGroup.addButton(self.num_8, 8)
        self.numGroup.addButton(self.num_9, 9)
        self.numGroup.buttonClicked[int].connect(self._handle_num)
        # Because Raspberry's pyqt version don't support auto add QButtonGroup
        # We need add buttonGroup ourself
        self.buttonGroup = QButtonGroup()
        self.buttonGroup.addButton(self.btnModeBP)
        self.buttonGroup.addButton(self.btnModeFC)
        self.buttonGroup.addButton(self.btnModeHP)
        self.buttonGroup.buttonClicked.connect(self._handle_arm_mode)
        self.num_clear.clicked[bool].connect(self._handle_num_clear)
        self.num_enter.clicked[bool].connect(self._handle_num_enter)
        self.checkBoxArmOpen.toggled.connect(self._handle_arm_open)
        self.orientReset.clicked[bool].connect(self._handle_reset_orent_clicked)
        self.refreshTopicStatus.clicked[bool].connect(self._handle_refresh_clicked)
        self.arm_1_pos.valueChanged[int].connect(self._handle_arm_1_pos)
        self.arm_2_pos.valueChanged[int].connect(self._handle_arm_2_pos)
        self.arm_catch_level.valueChanged[int].connect(self._handle_arm_catch_level)

        # self.video_pub_start.clicked[bool].connect(self._handle_video_pub_start)
        # self.video_pub_stop.clicked[bool].connect(self._handle_video_pub_stop)

        self.audio_pub_start.clicked[bool].connect(self._handle_audio_pub_start)
        self.audio_pub_stop.clicked[bool].connect(self._handle_audio_pub_stop)

        # self.video_rec_start.clicked[bool].connect(self._handle_video_rec_start)
        # self.video_rec_stop.clicked[bool].connect(self._handle_video_rec_stop)

        self.audio_rec_start.clicked[bool].connect(self._handle_audio_rec_start)
        self.audio_rec_stop.clicked[bool].connect(self._handle_audio_rec_stop)

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

    def _handle_video_pub_start(self):
        self.pipe_video_pub.set_state(Gst.State.PLAYING)

    def _handle_video_pub_stop(self):
        self.pipe_video_pub.set_state(Gst.State.PAUSED)

    def _handle_video_rec_start(self):
        self.pipe_video_pub.set_state(Gst.State.PLAYING)

    def _handle_video_rec_stop(self):
        self.pipe_video_pub.set_state(Gst.State.PAUSED)

    def _handle_audio_pub_start(self):
        self.pipe_audio_pub.set_state(Gst.State.PLAYING)

    def _handle_audio_pub_stop(self):
        self.pipe_audio_pub.set_state(Gst.State.PAUSED)

    def _handle_audio_rec_start(self):
        self.pipe_audio_rec.set_state(Gst.State.PLAYING)

    def _handle_audio_rec_stop(self):
        self.pipe_audio_rec.set_state(Gst.State.PAUSED)

    def _handle_start_clicked(self):
        # 1 is start
        # -1 is stop
        if self.pushButtonStart.isChecked():
            if self.btnModeAuto.isChecked():
                self._control_pub.publish(self.MODE_AUTO_START)
            elif self.btnModeRemote.isChecked():
                self._control_pub.publish(self.MODE_REMOTE_START)
        else:
            self._control_pub.publish(self.MODE_STOP)


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
        msg = Int32()
        if self.checkBoxArmOpen.isChecked():
            msg.data = self.MODE_ARM_SLIDER_OPEN
        else:
            msg.data = self.MODE_ARM_SLIDER_HOME

        self._arm_mode_pub.publish(msg)

    def _handle_arm_mode(self):
        msg = Int32()
        if self.btnModeHP.isChecked():
            msg.data = self.MODE_ARM_HOME_POSE
        elif self.btnModeBP.isChecked():
            msg.data = self.MODE_ARM_BTN_POSE
        elif self.btnModeFC.isChecked():
            msg.data = self.MODE_ARM_FREE_CONTROL

        self._arm_mode_pub.publish(msg)

    def _handle_refresh_clicked(self):
        self._is_imu_ready = False
        self._is_lrf_ready = False
        self._is_line_sensor_ready = False
        self.emit(SIGNAL("updateSensorStatus"))

    def _handle_arm_1_pos(self, value):
        msg = JointTrajectoryPoint()
        msg.positions.append(self.arm_1_pos.value())
        msg.positions.append(self.arm_2_pos.value())
        self._arm_pos_pub.publish(msg)

    def _handle_arm_2_pos(self, value):
        msg = JointTrajectoryPoint()
        msg.positions.append(self.arm_1_pos.value())
        msg.positions.append(self.arm_2_pos.value())
        self._arm_pos_pub.publish(msg)

    def _handle_arm_catch_level(self, value):
        msg = Int32()
        msg.data = value
        self._arm_catch_pub.publish(msg)

    def _set_icon_radian(self, radian):
        q = QTransform()
        q.translate(self.pixmap.width() / 2, self.pixmap.height() / 2)
        q.rotate(radian * 180 / 3.1415926)
        q.translate(-self.pixmap.width() / 2, -self.pixmap.height() / 2)
        self.graphicsPixmapItem.setTransform(q)



